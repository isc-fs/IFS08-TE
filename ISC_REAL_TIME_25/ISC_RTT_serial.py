"""
ISC RTT Serial — v2 (Fragmented Snapshot Protocol)
Replaces Excel/Influx with Flat CSV Logging & Marple Data Upload.
Retains all original Serial management and binary framing logic.

On-air protocol (STM32 → NRF24 → Arduino Nano → USB-Serial → here):
  Each 32-byte NRF24 payload is one fragment of a 102-byte snapshot.

  Fragment layout:
    [0]     magic      0xEC
    [1]     version    0x03
    [2]     frag_idx   0 … 4
    [3]     frag_tot   5
    [4..5]  seq        uint16 LE  — snapshot sequence number
    [6]     kind       0x06  (kRadioKindSnapshot)
    [7]     reserved   0x00
    [8..31] data       24 bytes — slice of the 102-byte snapshot

  Five fragments reconstruct the full 102-byte snapshot wire buffer.
  See serialize_radio_snapshot() in app_tasks.cpp for the exact layout.

Serial framing from Arduino (unchanged):
  AA 55 20 <32 raw bytes> <XOR checksum>
"""

from __future__ import annotations
import time
import struct
import logging
import csv
from datetime import datetime
from typing import Optional, Dict, List
from pathlib import Path

import pandas as pd
import serial
import serial.tools.list_ports
import numpy as np

import isc_marple

# ================== CONFIG RF ==================
RF_EXPECTED = {
    "PIPE_ADDR": "0xE7E7E7E7E7",
    "CHANNEL":   76,
    "PAYLOAD":   32,
    "DATA_RATE": "1Mbps",
    "AUTO_ACK":  False,
    "CRC":       "CRC_16",
    "PA":        "PA_MAX",
}

# ================== FRAGMENT PROTOCOL CONSTANTS ==================
# Must mirror the STM32 TX (app_tasks.cpp)
MAGIC         = 0xEC   # buf[0]
VERSION       = 0x03   # buf[1]
KIND_SNAP     = 0x06   # buf[6]  kRadioKindSnapshot
FRAG_TOTAL    = 5      # buf[3]  kRadioSnapshotFragmentCount
HDR_SIZE      = 8      # bytes 0-7 are the fragment header
DATA_SIZE     = 24     # bytes 8-31 are the data slice  (kRadioFragmentPayloadSize)
SNAPSHOT_SIZE = 102    # kRadioSnapshotWireSize

# ================== SERIAL FRAMING CONSTANTS ==================
SOF1        = 0xAA
SOF2        = 0x55
PAYLOAD_LEN = 32       # NRF24 fixed payload size

# ================== AMS CONSTANTS ==================
NUM_MODULES      = 5
CELLS_PER_MODULE = 19   # granular per-cell data not in radio snapshot; kept for future
TEMPS_PER_MODULE = 38   # same

# ================== LOG DIR ==================
LOG_DIR = Path("logs")
LOG_DIR.mkdir(exist_ok=True)

# ================== DEFAULTS ==================
DEFAULT_BAUD = 115200
DEFAULT_PORT = None

INFLUX_ENABLE_DEFAULT = False
DEBUG_ENABLE_DEFAULT  = False

# ================== GLOBALS (UI / STATUS) ==================
data_str       = ""
new_data_flag  = 0
latest_data_dict: dict = {}

_status: dict = {"badge": "STALE", "reason": "inicio", "ts": 0}
_last_seq: Optional[int] = None
_last_seq_advance_ts = 0.0
_STALE_T = 0.20

_excel_logger: Optional["SerialCSVLogger"] = None

logger = logging.getLogger("ISC_RTT_USB")
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

# ================== FRAGMENT REASSEMBLY STATE ==================
# seq (uint16) → {frag_idx: bytes(DATA_SIZE)}
_frag_buffers: Dict[int, Dict[int, bytes]] = {}
_MAX_PENDING_SEQS = 4   # evict oldest when this is exceeded

# ================== PER-MODULE AMS DATA ==================
class AMSModule:
    def __init__(self, module_id: int):
        self.module_id        = module_id
        self.cell_voltages_mv = [0] * CELLS_PER_MODULE
        self.temps_c          = [np.nan] * TEMPS_PER_MODULE
        self.min_cell_mv      = 0
        self.max_cell_mv      = 0
        self.max_temp_c       = np.nan
        self.last_update_ts   = 0.0

ams_modules: List[AMSModule] = [AMSModule(i) for i in range(NUM_MODULES)]

# ================== CSV LOGGER ==================
class SerialCSVLogger:
    """
    Flat CSV logger compatible with Marple Data.
    Columns mirror the 102-byte snapshot wire format from serialize_radio_snapshot().
    """
    def __init__(self, bucket_id: str, piloto: str, circuito: str):
        self.bucket_id  = bucket_id
        self.piloto     = piloto
        self.circuito   = circuito
        self.start_time = time.time()

        self.filename = LOG_DIR / f"{bucket_id}.csv"
        self.file     = open(self.filename, 'w', newline='')
        self.writer   = csv.writer(self.file)
        self.record_count = 0

        # "time" is the mandatory Marple timestamp column
        self.headers = [
            # ── Timing ──────────────────────────────────────────────────────────
            "time", "time_elapsed_s",

            # ── Snapshot meta ───────────────────────────────────────────────────
            "seq", "tick_ms",

            # ── Driver inputs  [snap bytes 6-12] ────────────────────────────────
            "start_button",
            "apps1_raw", "apps2_raw", "brake_raw",

            # ── Control  [snap bytes 13-17] ─────────────────────────────────────
            "torque_pct", "ev_2_3", "t11_8_9", "ctrl_state",

            # ── AMS / BMS  [snap bytes 18-58] ───────────────────────────────────
            "ok_precharge", "ams_fsm_state",
            "v_cell_min_mV", "soc",
            "vmin_mod0", "vmin_mod1", "vmin_mod2", "vmin_mod3", "vmin_mod4",
            "vmax_mod0", "vmax_mod1", "vmax_mod2", "vmax_mod3", "vmax_mod4",
            "corriente_accu", "corriente_dcdc", "temp_dcdc",
            "tmax_mod0", "tmax_mod1", "tmax_mod2", "tmax_mod3", "tmax_mod4",

            # ── Inverter  [snap bytes 59-81] ────────────────────────────────────
            "inv_state", "inv_vconfig_active", "inv_error",
            "inv_dc_bus_V",
            "inv_temp_motor1", "inv_temp_pwrstg", "inv_temp_board",
            "inv_rpm", "inv_speed_actual", "inv_current_actual",
        ]

        self.writer.writerow(self.headers)
        logger.info(f"CSV Logger iniciado: {self.filename}")

    def log_snapshot(self, data_dict: dict) -> None:
        """Write one row from the latest decoded snapshot."""
        current_ts = datetime.now().isoformat()
        elapsed    = time.time() - self.start_time
        s          = data_dict.get('snapshot', {})

        vmin = s.get('vmin_modulo',     [0] * 5)
        vmax = s.get('vmax_modulo',     [0] * 5)
        tmax = s.get('temp_max_modulo', [0] * 5)

        row = [
            current_ts, f"{elapsed:.3f}",
            s.get('seq',          0), s.get('tick_ms', 0),
            s.get('start_button', 0),
            s.get('apps1_raw',    0), s.get('apps2_raw', 0), s.get('brake_raw', 0),
            s.get('torque_pct',   0), s.get('ev_2_3', 0), s.get('t11_8_9', 0), s.get('state', 0),
            s.get('ok_precharge', 0), s.get('ams_fsm_state', 0),
            s.get('v_cell_min_mV', 0), s.get('soc', 0),
            *(vmin[i] if i < len(vmin) else 0 for i in range(5)),
            *(vmax[i] if i < len(vmax) else 0 for i in range(5)),
            s.get('corriente_accu', 0),
            s.get('corriente_dcdc', 0),
            s.get('temp_dcdc',      0),
            *(tmax[i] if i < len(tmax) else 0 for i in range(5)),
            s.get('inv_state',          0), s.get('last_vconfig_tick', 0), s.get('inv_error', 0),
            s.get('inv_dc_bus_V',       0),
            s.get('inv_temp_motor1',    0), s.get('inv_temp_pwrstg', 0), s.get('inv_temp_board', 0),
            s.get('inv_rpm',            0), s.get('inv_speed_actual', 0), s.get('inv_current_actual', 0),
        ]

        self.writer.writerow(row)
        self.record_count += 1
        if self.record_count % 50 == 0:
            self.file.flush()

    def close(self) -> Optional[str]:
        if self.file:
            self.file.close()
            logger.info(f"CSV cerrado. Registros: {self.record_count}")
            return str(self.filename)
        return None

# ================== SERIAL UTILITIES ==================
def _dump_hex(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)

def list_serial_ports():
    return [(p.device, p.description) for p in serial.tools.list_ports.comports()]

def list_excel_sessions():
    """List CSV session files (sorted newest first)."""
    if LOG_DIR.exists():
        return sorted(LOG_DIR.glob("*.csv"), key=lambda x: x.stat().st_mtime, reverse=True)
    return []

def load_excel_session(filepath: Path) -> Dict[str, pd.DataFrame]:
    """Load a CSV session for the UI viewer."""
    try:
        df = pd.read_csv(filepath)
        return {'Main': df}
    except Exception as e:
        logger.error(f"Error loading CSV session: {e}")
        return {}

def _auto_detect_port() -> Optional[str]:
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = (p.description or "").upper()
        if "CH340" in desc or "USB-SERIAL" in desc or "CP210" in desc:
            return p.device
    return ports[0].device if ports else None

def _open_serial(port: str, baud: int) -> serial.Serial:
    ser = serial.Serial(port=port, baudrate=baud, timeout=0.2)
    time.sleep(1.5)
    ser.reset_input_buffer()
    return ser

def _set_badge(badge: str, reason: str) -> None:
    global _status
    _status = {"badge": badge, "reason": reason, "ts": int(time.time() * 1000)}
    latest_data_dict["__STATUS__"] = _status

def _mod16_diff(curr: int, prev: int) -> int:
    return (curr - prev) & 0xFFFF

def _read_frame(ser: serial.Serial, counters: Optional[dict] = None):
    """
    Robust SOF-framed reader.  Returns (payload_bytes, error_str|None).
    Frame format from Arduino:  AA 55 20 <32 bytes> <XOR>
    """
    b = ser.read(1)
    if not b:
        if counters is not None: counters["timeout"] += 1
        return None, "timeout"
    if b[0] != SOF1:
        return None, None

    b2 = ser.read(1)
    if not b2:
        if counters is not None: counters["timeout"] += 1
        return None, "timeout"
    if b2[0] != SOF2:
        return None, None

    ln = ser.read(1)
    if not ln:
        if counters is not None: counters["timeout"] += 1
        return None, "timeout"
    if ln[0] != PAYLOAD_LEN:
        if counters is not None: counters["len"] += 1
        ser.read(min(ln[0], 255))
        ser.read(1)
        return None, "len"

    payload = ser.read(PAYLOAD_LEN)
    if len(payload) != PAYLOAD_LEN:
        if counters is not None: counters["short"] += 1
        return None, "short"

    chk = ser.read(1)
    if not chk:
        if counters is not None: counters["timeout"] += 1
        return None, "timeout"

    xorv = 0
    for bb in payload:
        xorv ^= bb
    if xorv != chk[0]:
        if counters is not None: counters["chk"] += 1
        return None, "chk"

    return payload, None

# ================== FRAGMENT VALIDATION & REASSEMBLY ==================
def _validate_fragment(payload: bytes) -> bool:
    """
    Return True if the 32-byte payload is a valid snapshot fragment.
    Checks: magic, version, kind, frag_tot, frag_idx bounds.
    """
    if len(payload) != PAYLOAD_LEN:
        return False
    if payload[0] != MAGIC:
        logger.debug(f"[DROP] bad magic: 0x{payload[0]:02X} (expected 0x{MAGIC:02X})")
        return False
    if payload[1] != VERSION:
        logger.debug(f"[DROP] bad version: {payload[1]} (expected {VERSION})")
        return False
    if payload[6] != KIND_SNAP:
        logger.debug(f"[DROP] unknown kind: {payload[6]} (expected {KIND_SNAP})")
        return False
    if payload[3] != FRAG_TOTAL:
        logger.debug(f"[DROP] unexpected frag_tot: {payload[3]} (expected {FRAG_TOTAL})")
        return False
    if payload[2] >= FRAG_TOTAL:
        logger.debug(f"[DROP] frag_idx out of range: {payload[2]}")
        return False
    return True

def _process_fragment(payload: bytes) -> Optional[bytes]:
    """
    Store a validated fragment in the reassembly buffer keyed by seq.
    Returns the complete 102-byte snapshot bytes when all FRAG_TOTAL
    fragments for the same seq have arrived; otherwise returns None.

    Partial buffers for stale seqs are evicted when _MAX_PENDING_SEQS
    is exceeded (oldest seq discarded).
    """
    global _frag_buffers

    frag_idx: int = payload[2]
    seq: int      = struct.unpack_from('<H', payload, 4)[0]
    data: bytes   = bytes(payload[HDR_SIZE: HDR_SIZE + DATA_SIZE])

    # Evict oldest partial buffer if too many seqs are in flight
    if seq not in _frag_buffers and len(_frag_buffers) >= _MAX_PENDING_SEQS:
        oldest = min(_frag_buffers.keys(), key=lambda s: _mod16_diff(seq, s))
        del _frag_buffers[oldest]
        logger.debug(f"[FRAG] evicted stale partial seq={oldest}")

    _frag_buffers.setdefault(seq, {})[frag_idx] = data

    if len(_frag_buffers[seq]) == FRAG_TOTAL:
        buf = _frag_buffers.pop(seq)
        snapshot = bytearray(SNAPSHOT_SIZE)
        for idx in range(FRAG_TOTAL):
            start    = idx * DATA_SIZE
            chunk    = buf[idx]
            copy_len = min(len(chunk), SNAPSHOT_SIZE - start)
            if copy_len > 0:
                snapshot[start: start + copy_len] = chunk[:copy_len]
        return bytes(snapshot)

    return None

# ================== SNAPSHOT DECODER ==================
def _u16(data: bytes, offset: int) -> int:
    """Unsigned little-endian 16-bit integer."""
    return struct.unpack_from('<H', data, offset)[0]

def _i16(data: bytes, offset: int) -> int:
    """Signed little-endian 16-bit integer (stored as uint16, reinterpreted)."""
    v = struct.unpack_from('<H', data, offset)[0]
    return v if v < 0x8000 else v - 0x10000

def _i32(data: bytes, offset: int) -> int:
    return struct.unpack_from('<i', data, offset)[0]

def _u32(data: bytes, offset: int) -> int:
    return struct.unpack_from('<I', data, offset)[0]

def _decode_snapshot(data: bytes) -> dict:
    """
    Parse a 102-byte serialized snapshot into a Python dict.

    Matches serialize_radio_snapshot() in app_tasks.cpp exactly:

      [0..3]   tick_ms               uint32 LE
      [4..5]   seq                   uint16 LE
      [6]      start_button          uint8  (bool)
      [7..8]   apps1_raw             uint16 LE
      [9..10]  apps2_raw             uint16 LE
      [11..12] brake_raw             uint16 LE
      [13..14] torque_pct            uint16 LE  (uint8 zero-extended by put_le16)
      [15]     ev_2_3                uint8
      [16]     t11_8_9               uint8
      [17]     state                 uint8
      [18]     ok_precharge          uint8  (bool)
      [19]     ams_fsm_state         uint8
      [20..21] v_cell_min_mV         uint16 LE
      [22]     soc                   uint8
      [23..32] vmin_modulo[0..4]     5 × uint16 LE
      [33..42] vmax_modulo[0..4]     5 × uint16 LE
      [43..44] corriente_accu        int16  LE  (cast to uint16 on TX)
      [45..46] corriente_dcdc        int16  LE  (cast to uint16 on TX)
      [47..48] temp_dcdc             int16  LE  (cast to uint16 on TX)
      [49..58] temp_max_modulo[0..4] 5 × int16 LE (cast to uint16 on TX)
      [59]     inv_state             uint8
      [60]     inv_vconfig_active    uint8  (bool: last_vconfig_tick != 0)
      [61]     inv_error             uint8
      [62..63] inv_dc_bus_V          uint16 LE
      [64..65] inv_temp_motor1       uint16 LE
      [66..67] inv_temp_pwrstg       uint16 LE
      [68..69] inv_temp_board        uint16 LE
      [70..73] inv_rpm               int32  LE
      [74..77] inv_speed_actual      int32  LE
      [78..81] inv_current_actual    int32  LE
      [82..101] reserved / zero
    """
    if len(data) < SNAPSHOT_SIZE:
        logger.warning(f"_decode_snapshot: short buffer ({len(data)} < {SNAPSHOT_SIZE})")
        return {}

    return {
        'tick_ms':             _u32(data, 0),
        'seq':                 _u16(data, 4),
        'start_button':        data[6],
        'apps1_raw':           _u16(data, 7),
        'apps2_raw':           _u16(data, 9),
        'brake_raw':           _u16(data, 11),
        'torque_pct':          _u16(data, 13),   # uint8 stored in 2 bytes
        'ev_2_3':              data[15],
        't11_8_9':             data[16],
        'state':               data[17],
        'ok_precharge':        data[18],
        'ams_fsm_state':       data[19],
        'v_cell_min_mV':       _u16(data, 20),
        'soc':                 data[22],
        'vmin_modulo':         [_u16(data, 23 + 2 * i) for i in range(5)],
        'vmax_modulo':         [_u16(data, 33 + 2 * i) for i in range(5)],
        'corriente_accu':      _i16(data, 43),
        'corriente_dcdc':      _i16(data, 45),
        'temp_dcdc':           _i16(data, 47),
        'temp_max_modulo':     [_i16(data, 49 + 2 * i) for i in range(5)],
        'inv_state':           data[59],
        'last_vconfig_tick':   data[60],
        'inv_error':           data[61],
        'inv_dc_bus_V':        _u16(data, 62),
        'inv_temp_motor1':     _u16(data, 64),
        'inv_temp_pwrstg':     _u16(data, 66),
        'inv_temp_board':      _u16(data, 68),
        'inv_rpm':             _i32(data, 70),
        'inv_speed_actual':    _i32(data, 74),
        'inv_current_actual':  _i32(data, 78),
    }

# ================== SNAPSHOT → GLOBAL STATE ==================
def parse_snapshot(snap: dict) -> None:
    """
    Update latest_data_dict and AMSModule objects from a freshly decoded snapshot.
    Also maintains backward-compatible 'ams_summary' / 'ams_current' / 'ams_temp_summary'
    keys so any existing UI code keeps working.
    """
    global data_str, ams_modules

    if not snap:
        return

    latest_data_dict['snapshot'] = snap

    # ── Per-module AMS state ──────────────────────────────────────────────────
    vmin = snap.get('vmin_modulo',     [])
    vmax = snap.get('vmax_modulo',     [])
    tmax = snap.get('temp_max_modulo', [])

    for i, mod in enumerate(ams_modules):
        if i < len(vmin): mod.min_cell_mv = vmin[i]
        if i < len(vmax): mod.max_cell_mv = vmax[i]
        if i < len(tmax): mod.max_temp_c  = float(tmax[i])
        mod.last_update_ts = time.time()

    # ── Backward-compat AMS summary keys ─────────────────────────────────────
    valid_tmax = [t for t in tmax if t != 0]
    latest_data_dict['ams_summary'] = {
        'min_cell_mv': snap.get('v_cell_min_mV', 0),
        'max_cell_mv': max(vmax) if vmax else 0,
        'stack_mv':    0,  # not in snapshot wire format
    }
    latest_data_dict['ams_current'] = {
        'current_A': snap.get('corriente_accu', 0),
    }
    latest_data_dict['ams_temp_summary'] = {
        'max_temp_c': max(valid_tmax) if valid_tmax else 0,
        'min_temp_c': min(valid_tmax) if valid_tmax else 0,
        'avg_temp_c': (sum(valid_tmax) / len(valid_tmax)) if valid_tmax else 0,
    }

    # ── UI status string ──────────────────────────────────────────────────────
    badge = _status.get('badge', '?')
    data_str = (
        f"[{badge}] SEQ={snap.get('seq', 0):5d}  "
        f"rpm={snap.get('inv_rpm', 0):6d}  "
        f"Vbus={snap.get('inv_dc_bus_V', 0):3d}V  "
        f"apps1={snap.get('apps1_raw', 0):4d}  "
        f"apps2={snap.get('apps2_raw', 0):4d}  "
        f"soc={snap.get('soc', 0):3d}%"
    )

# ================== API FOR UI ==================
def get_ams_module_data(module_idx: int) -> Optional[AMSModule]:
    """Return AMSModule for a specific module (0-4)."""
    if 0 <= module_idx < NUM_MODULES:
        return ams_modules[module_idx]
    return None

def get_all_temps_array() -> np.ndarray:
    """Return array of per-module max temperatures (5 values, °C)."""
    return np.array([m.max_temp_c for m in ams_modules])

def create_bucket(piloto: str, circuito: str, use_influx: bool = False) -> str:
    """Create a unique session bucket name."""
    ts            = datetime.now().strftime("%Y%m%d_%H%M%S")
    safe_piloto   = piloto.replace(" ", "_")
    safe_circuito = circuito.replace(" ", "_")
    return f"ISC_{ts}_{safe_piloto}_{safe_circuito}"

def get_latest_data(data_id: Optional[str] = None):
    if data_id:
        return latest_data_dict.get(data_id, {})
    return latest_data_dict.copy()

# ================== MAIN RECEIVE LOOP ==================
def receive_data(bucket_id: str,
                 piloto: str,
                 circuito: str,
                 port: Optional[str] = DEFAULT_PORT,
                 baud: int = DEFAULT_BAUD,
                 use_influx: bool = False,   # use_influx=True triggers Marple upload
                 debug: bool = False) -> None:

    global new_data_flag, _last_seq, _last_seq_advance_ts, _excel_logger, DEBUG_ENABLE_DEFAULT

    DEBUG_ENABLE_DEFAULT = debug
    logger.setLevel(logging.DEBUG if debug else logging.INFO)
    logger.info("Recepción USB-Serial iniciada (protocolo fragmentado v2). Marple Upload: %s", use_influx)

    _excel_logger = SerialCSVLogger(bucket_id, piloto, circuito)

    if port is None:
        port = _auto_detect_port()
    if port is None:
        raise RuntimeError("No se encontró un puerto serie RF-NANO.")

    ser = _open_serial(port, baud)
    logger.info("[CONFIG] port=%s  baud=%d  log=%s", port, baud, _excel_logger.filename)

    counters = {
        "rx":        0,   # raw frames that passed XOR check
        "frag_ok":   0,   # frames that passed fragment validation
        "frag_drop": 0,   # frames dropped by _validate_fragment
        "snapshot":  0,   # complete snapshots reassembled
        "timeout":   0,
        "len":       0,
        "short":     0,
        "chk":       0,   # XOR mismatch
    }
    last_stats_t = time.time()
    last_log_t   = time.time()

    _set_badge("STALE", "esperando primer frame")

    try:
        while new_data_flag != -1:
            now = time.time()

            payload, err = _read_frame(ser, counters=counters)

            # ── Handle frame-level errors ─────────────────────────────────────
            if err == "timeout":
                if _last_seq is not None and (now - _last_seq_advance_ts) > _STALE_T:
                    _set_badge("STALE", "sin avance de SEQ")
                if debug and now - last_stats_t >= 2.0:
                    logger.debug("[STATS] rx=%d frag_ok=%d snap=%d chk=%d drop=%d",
                                 counters["rx"], counters["frag_ok"],
                                 counters["snapshot"], counters["chk"], counters["frag_drop"])
                    last_stats_t = now
                continue
            elif err in ("len", "short", "chk"):
                _set_badge("BAD", err)
                continue

            if payload is None:
                continue

            counters["rx"] += 1

            # ── Fragment validation ───────────────────────────────────────────
            if not _validate_fragment(payload):
                counters["frag_drop"] += 1
                _set_badge("BAD", "frag_invalid")
                continue

            counters["frag_ok"] += 1

            # ── SEQ badge tracking (per fragment for fast LIVE detection) ─────
            frag_seq: int = struct.unpack_from('<H', payload, 4)[0]
            if _last_seq is None:
                _last_seq            = frag_seq
                _last_seq_advance_ts = now
                _set_badge("LIVE", "primer fragmento")
            else:
                diff = _mod16_diff(frag_seq, _last_seq)
                if diff > 0:
                    _last_seq            = frag_seq
                    _last_seq_advance_ts = now
                    _set_badge("LIVE", f"SEQ +{diff}")
                elif (now - _last_seq_advance_ts) > _STALE_T:
                    _set_badge("STALE", "SEQ detenido")

            # ── Reassembly ────────────────────────────────────────────────────
            snapshot_bytes = _process_fragment(payload)
            if snapshot_bytes is None:
                continue   # waiting for remaining fragments

            # ── Complete snapshot received ────────────────────────────────────
            counters["snapshot"] += 1
            snap = _decode_snapshot(snapshot_bytes)
            if not snap:
                _set_badge("BAD", "decode_fail")
                continue

            parse_snapshot(snap)
            new_data_flag = 1

            # Log at ~10 Hz to keep CSV manageable
            if now - last_log_t >= 0.1:
                if _excel_logger:
                    _excel_logger.log_snapshot(latest_data_dict)
                last_log_t = now

            if debug and now - last_stats_t >= 2.0:
                logger.debug("[STATS] rx=%d frag_ok=%d snap=%d chk=%d drop=%d",
                             counters["rx"], counters["frag_ok"],
                             counters["snapshot"], counters["chk"], counters["frag_drop"])
                last_stats_t = now

    finally:
        try:
            ser.close()
        except Exception:
            pass

        file_path: Optional[str] = None
        if _excel_logger:
            file_path = _excel_logger.close()

        logger.info("Recepción USB-Serial finalizada. Snapshots: %d", counters["snapshot"])

        # ── Marple upload ─────────────────────────────────────────────────────
        if use_influx and file_path:
            logger.info("Iniciando subida a Marple Data...")
            isc_marple.upload_session_csv(file_path, {
                "piloto":   piloto,
                "circuito": circuito,
                "type":     "Real_Telemetry",
                "date":     datetime.now().isoformat(),
            })