"""
ISC RTT Serial — v3 (Optimized + Post-Race Data Injection)
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

Post-race data injection:
  GPS coordinates and AMS temperatures are logged separately on the car
  (micro-SD card), then merged into the session CSV after the race via
  merge_gps_into_session() and merge_ams_temps_into_session().
"""

from __future__ import annotations
from collections import deque
import csv
import logging
import struct
import threading
import time
from datetime import datetime, timedelta
from functools import reduce
from operator import xor
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import pandas as pd
import serial
import serial.tools.list_ports

import isc_marple

# ================== CONFIG RF ==================
RF_EXPECTED = {
    "PIPE_ADDR": "0xE7E7E7E7E7",
    "CHANNEL":   76,
    "PAYLOAD":   32,
    "DATA_RATE": "1Mbps",
    "AUTO_ACK":  False,
    "CRC":       "CRC_8",   # TX CONFIG=EN_CRC|PWR_UP -> 8-bit
    "PA":        "PA_LOW",  # TX RF_SETUP=0x06 -> 0 dBm
}

# ================== FRAGMENT PROTOCOL CONSTANTS ==================
# Must mirror the STM32 TX (telemetry_task.cpp on feat/telemetry-port)
MAGIC            = 0xEC   # buf[0]
VERSION_LEGACY   = 0x02   # buf[1]
VERSION_SNAPSHOT = 0x03   # buf[1]
VERSION          = 0x02   # legacy expected version default
KIND_FAST        = 0x03   # buf[6]
KIND_SLOW        = 0x04   # buf[6]
KIND_SNAPSHOT    = 0x06   # buf[6]
KIND_STATUS      = 0x99   # buf[6] custom status code from Arduino
FRAG_FAST        = 2
FRAG_SLOW        = 5
FRAG_SNAPSHOT    = 5
HDR_SIZE      = 8      # bytes 0-7 are the fragment header
DATA_SIZE     = 24     # bytes 8-31 are the data slice
SNAPSHOT_SIZE = 102    # backward compatibility for CSV headers/IMU


# ================== SERIAL FRAMING CONSTANTS ==================
SOF1        = 0xAA
SOF2        = 0x55
PAYLOAD_LEN = 32       # NRF24 fixed payload size

# ================== AMS CONSTANTS ==================
NUM_MODULES      = 5
CELLS_PER_MODULE = 19   # granular per-cell data not in radio snapshot; kept for future
TEMPS_PER_MODULE = 38   # same

# ================== POST-RACE DATA CONSTANTS ==================
# GPS columns added to session CSV during post-race injection
GPS_MERGE_COLS: List[str] = [
    'gps_lat_deg', 'gps_lon_deg', 'gps_sog_knots',
    'gps_cog_deg', 'gps_sats', 'gps_fix',
]

# AMS per-cell temperature column names (format: ams_t_mod{m}_cell{c})
# 19 cells × 5 modules = 95 columns (populated during post-race injection)
AMS_TEMP_COLS: List[str] = [
    f'ams_t_mod{m}_cell{c}'
    for m in range(NUM_MODULES)
    for c in range(CELLS_PER_MODULE)
]

# ================== LOG DIR ==================
def get_user_dir() -> Path:
    import os
    try:
        if os.name == 'nt':
            import ctypes
            from ctypes import wintypes
            CSIDL_PERSONAL = 5
            SHGFP_TYPE_CURRENT = 0
            buf = ctypes.create_unicode_buffer(wintypes.MAX_PATH)
            ctypes.windll.shell32.SHGetFolderPathW(None, CSIDL_PERSONAL, None, SHGFP_TYPE_CURRENT, buf)
            if buf.value:
                return Path(buf.value) / "ISCmetrics"
    except Exception:
        pass
    return Path.home() / "Documents" / "ISCmetrics"

USER_DIR = get_user_dir()
LOG_DIR = USER_DIR / "logs"
LOG_DIR.mkdir(parents=True, exist_ok=True)

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
_receiver_status: dict = {"hw_status": "OK", "last_update": 0.0}
_last_received_snap_seq: Optional[int] = None
_lqi_history: deque = deque(maxlen=50)
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
    # Column order — must match log_snapshot()
    HEADERS: List[str] = [
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
        # ── IMU (simulated or parsed from bytes 82-101 of snapshot) ──────────
        "imu_ax_g", "imu_ay_g", "imu_az_g",
        "imu_gx_dps", "imu_gy_dps", "imu_gz_dps",
        "imu_roll_deg", "imu_pitch_deg",
        # ── Pit-Wall Notes ──────────────────────────────────────────────────
        "notes",
    ]

    def __init__(self, bucket_id: str, piloto: str, circuito: str, flush_every: int = 50):
        self.bucket_id    = bucket_id
        self.piloto       = piloto
        self.circuito     = circuito
        self.start_time   = time.time()
        self.flush_every  = flush_every

        self.filename     = LOG_DIR / f"{bucket_id}.csv"
        self.file         = open(self.filename, 'w', newline='')
        self.writer       = csv.writer(self.file)
        self.record_count = 0

        self.writer.writerow(self.HEADERS)
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
            s.get('seq',            0), s.get('tick_ms',        0),
            s.get('start_button',   0),
            s.get('apps1_raw',      0), s.get('apps2_raw',      0), s.get('brake_raw', 0),
            s.get('torque_pct',     0), s.get('ev_2_3',         0),
            s.get('t11_8_9',        0), s.get('state',          0),
            s.get('ok_precharge',   0), s.get('ams_fsm_state',  0),
            s.get('v_cell_min_mV',  0), s.get('soc',            0),
            *(vmin[i] if i < len(vmin) else 0 for i in range(5)),
            *(vmax[i] if i < len(vmax) else 0 for i in range(5)),
            s.get('corriente_accu', 0),
            s.get('corriente_dcdc', 0),
            s.get('temp_dcdc',      0),
            *(tmax[i] if i < len(tmax) else 0 for i in range(5)),
            s.get('inv_state',             0), s.get('last_vconfig_tick', 0),
            s.get('inv_error',             0), s.get('inv_dc_bus_V',      0),
            s.get('inv_temp_motor1',       0), s.get('inv_temp_pwrstg',   0),
            s.get('inv_temp_board',        0), s.get('inv_rpm',           0),
            s.get('inv_speed_actual',      0), s.get('inv_current_actual', 0),
            # IMU columns
            s.get('imu_ax_g',            0.0), s.get('imu_ay_g',         0.0), s.get('imu_az_g',            0.0),
            s.get('imu_gx_dps',          0.0), s.get('imu_gy_dps',       0.0), s.get('imu_gz_dps',          0.0),
            s.get('imu_roll_deg',        0.0), s.get('imu_pitch_deg',    0.0),
            "",  # Empty note for telemetry snapshots
        ]

        self.writer.writerow(row)
        self.record_count += 1
        if self.record_count % self.flush_every == 0:
            self.file.flush()

    def write_note(self, text: str) -> None:
        """Write a custom comment/note row into the CSV file."""
        try:
            current_ts = datetime.now().isoformat()
            elapsed    = time.time() - self.start_time
            # Create a row with timestamp and note, leaving all telemetry columns empty
            row = [current_ts, f"{elapsed:.3f}"] + [""] * (len(self.HEADERS) - 3) + [text]
            self.writer.writerow(row)
            self.file.flush()
        except Exception as e:
            logger.error(f"Error writing note to CSV: {e}")

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
    """List CSV session files (sorted newest first, excluding _gps suffix files)."""
    if LOG_DIR.exists():
        return sorted(
            [f for f in LOG_DIR.glob("*.csv") if not f.stem.endswith('_gps')],
            key=lambda x: x.stat().st_mtime,
            reverse=True,
        )
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

def _xor_check(payload: bytes) -> int:
    """Return XOR of all bytes in payload (checksum byte)."""
    return reduce(xor, payload, 0)

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

    if _xor_check(payload) != chk[0]:
        if counters is not None: counters["chk"] += 1
        return None, "chk"

    return payload, None


# ================== FRAGMENT VALIDATION & REASSEMBLY ==================
def _validate_fragment(payload: bytes) -> bool:
    """
    Return True if the 32-byte payload is a valid snapshot fragment or status packet.
    Checks: magic, version, kind, frag_tot, frag_idx bounds.
    """
    if len(payload) != PAYLOAD_LEN:
        return False
    if payload[0] != MAGIC:
        logger.debug(f"[DROP] bad magic: 0x{payload[0]:02X} (expected 0x{MAGIC:02X})")
        return False
    
    ver  = payload[1]
    kind = payload[6]
    ftot = payload[3]
    fidx = payload[2]

    if kind == KIND_STATUS:
        return True

    # --- Current protocol: version 3, kind 6, 5 fragments ---
    if ver == VERSION_SNAPSHOT and kind == KIND_SNAPSHOT:
        if ftot != FRAG_SNAPSHOT:
            logger.debug(f"[DROP] bad SNAPSHOT frag_tot: {ftot}")
            return False
        if fidx >= FRAG_SNAPSHOT:
            logger.debug(f"[DROP] bad SNAPSHOT frag_idx: {fidx}")
            return False
        return True

    # --- Legacy protocol: version 2, kind FAST/SLOW ---
    if ver == VERSION_LEGACY:
        if kind == KIND_FAST:
            if ftot != FRAG_FAST:
                logger.debug(f"[DROP] bad FRAG_FAST frag_tot: {ftot}")
                return False
            if fidx >= FRAG_FAST:
                logger.debug(f"[DROP] bad FRAG_FAST frag_idx: {fidx}")
                return False
            return True
        if kind == KIND_SLOW:
            if ftot != FRAG_SLOW:
                logger.debug(f"[DROP] bad FRAG_SLOW frag_tot: {ftot}")
                return False
            if fidx >= FRAG_SLOW:
                logger.debug(f"[DROP] bad FRAG_SLOW frag_idx: {fidx}")
                return False
            return True

    logger.debug(f"[DROP] unknown version/kind: ver={ver} kind={kind}")
    return False



def _handle_status_packet(payload: bytes) -> None:
    global _receiver_status
    status_code = payload[7]
    ts = time.time()
    if status_code == 0x01:
        _receiver_status = {"hw_status": "NO_RADIO_HW", "last_update": ts}
    elif status_code == 0x02:
        _receiver_status = {"hw_status": "NO_SIGNAL", "last_update": ts}
    else:
        _receiver_status = {"hw_status": "OK", "last_update": ts}
    latest_data_dict["__RECEIVER_STATUS__"] = _receiver_status


def _process_fragment(payload: bytes) -> Optional[Tuple[int, int, bytes]]:
    """
    Store a validated fragment in the reassembly buffer.
    Returns (kind, seq, snapshot_bytes) when complete; otherwise None.
    """
    global _frag_buffers, _receiver_status

    # Check for custom status frame
    if payload[6] == KIND_STATUS:
        _handle_status_packet(payload)
        return None

    # Reset status warning when actual radio fragments start arriving
    if _receiver_status.get("hw_status") != "OK":
        _receiver_status = {"hw_status": "OK", "last_update": time.time()}
        latest_data_dict["__RECEIVER_STATUS__"] = _receiver_status

    kind: int     = payload[6]
    frag_idx: int = payload[2]
    frag_tot: int = payload[3]
    seq: int      = struct.unpack_from('<H', payload, 4)[0]
    data: bytes   = bytes(payload[HDR_SIZE: HDR_SIZE + DATA_SIZE])

    key = (kind, seq)
    if key not in _frag_buffers and len(_frag_buffers) >= _MAX_PENDING_SEQS:
        oldest_key = min(_frag_buffers.keys(), key=lambda k: _mod16_diff(seq, k[1]))
        del _frag_buffers[oldest_key]
        logger.debug(f"[FRAG] evicted stale partial key={oldest_key}")

    _frag_buffers.setdefault(key, {})[frag_idx] = data

    if len(_frag_buffers[key]) == frag_tot:
        buf_dict = _frag_buffers.pop(key)
        assembled_size = frag_tot * DATA_SIZE
        snapshot = bytearray(assembled_size)
        for idx in range(frag_tot):
            start = idx * DATA_SIZE
            chunk = buf_dict[idx]
            copy_len = min(len(chunk), assembled_size - start)
            if copy_len > 0:
                snapshot[start: start + copy_len] = chunk[:copy_len]
        return kind, seq, bytes(snapshot)

    return None


# ================== SNAPSHOT DECODER ==================
# Pre-compiled struct formats matching telemetry_task.cpp on feat/telemetry-port
_SNAP_FMT_FAST = struct.Struct("<BBBBBBBBBHHHHHH3xHHHi14x")
_SNAP_FMT_SLOW = struct.Struct("<Bhhh8xI5x24x5H14x5H14x5h14x")


def _decode_fast_snapshot(data: bytes, seq: int) -> dict:
    if len(data) < 48:
        logger.warning(f"_decode_fast_snapshot: short buffer ({len(data)} < 48)")
        return {}
    unpacked = _SNAP_FMT_FAST.unpack(data)
    return {
        'seq': seq,
        'ctrl_state':        unpacked[0],
        'inv_state':         unpacked[1],
        'ams_fsm_state':     unpacked[2],
        'start_button':      unpacked[3],
        'ok_precharge':      unpacked[4],
        'ev_2_3':            unpacked[5],
        't11_8_9':           unpacked[6],
        'last_vconfig_tick': unpacked[7],
        'inv_error':         unpacked[8],
        'torque_pct':        unpacked[9],
        'inv_dc_bus_V':      unpacked[10],
        'v_cell_min_mV':     unpacked[11],
        'apps1_raw':         unpacked[12],
        'apps2_raw':         unpacked[13],
        'brake_raw':         unpacked[14],
        'inv_temp_motor1':   unpacked[15],
        'inv_temp_pwrstg':   unpacked[16],
        'inv_temp_board':    unpacked[17],
        'inv_rpm':           int(round(unpacked[18] / 10.0)),
    }


def _decode_slow_snapshot(data: bytes, seq: int) -> dict:
    if len(data) < 120:
        logger.warning(f"_decode_slow_snapshot: short buffer ({len(data)} < 120)")
        return {}
    unpacked = _SNAP_FMT_SLOW.unpack(data)
    return {
        'seq': seq,
        'soc':             unpacked[0],
        'corriente_accu':  unpacked[1] / 10.0,
        'corriente_dcdc':  unpacked[2] / 10.0,
        'temp_dcdc':       unpacked[3],
        'tick_ms':         unpacked[4],
        'vmin_modulo':     list(unpacked[5:10]),
        'vmax_modulo':     list(unpacked[10:15]),
        'temp_max_modulo': list(unpacked[15:20]),
    }


_SNAP_FMT_FLAT = struct.Struct("<I H B H H H H B B B B B H B 5H 5H h h h 5h B B B H H H H i i i 20x")

def _decode_flat_snapshot(data: bytes, seq: int) -> dict:
    if len(data) < 102:
        logger.warning(f"_decode_flat_snapshot: short buffer ({len(data)} < 102)")
        return {}
    unpacked = _SNAP_FMT_FLAT.unpack(data[:102])
    
    # Extract list values
    vmin_modulo = list(unpacked[14:19])
    vmax_modulo = list(unpacked[19:24])
    temp_max_modulo = list(unpacked[27:32])
    
    return {
        'tick_ms':            unpacked[0],
        'seq':                seq, # unpacked[1] is seq as well, but we pass it
        'start_button':       unpacked[2],
        'apps1_raw':          unpacked[3],
        'apps2_raw':          unpacked[4],
        'brake_raw':          unpacked[5],
        'torque_pct':         unpacked[6],
        'ev_2_3':             unpacked[7],
        't11_8_9':            unpacked[8],
        'ctrl_state':         unpacked[9], # mapping ctrl.state to ctrl_state
        'ok_precharge':       unpacked[10],
        'ams_fsm_state':      unpacked[11],
        'v_cell_min_mV':      unpacked[12],
        'soc':                unpacked[13],
        'vmin_modulo':        vmin_modulo,
        'vmax_modulo':        vmax_modulo,
        'corriente_accu':     unpacked[24] / 10.0,
        'corriente_dcdc':     unpacked[25] / 10.0,
        'temp_dcdc':          unpacked[26],
        'temp_max_modulo':    temp_max_modulo,
        'inv_state':          unpacked[32],
        'last_vconfig_tick':  unpacked[33],
        'inv_error':          unpacked[34],
        'inv_dc_bus_V':       unpacked[35],
        'inv_temp_motor1':    unpacked[36],
        'inv_temp_pwrstg':    unpacked[37],
        'inv_temp_board':     unpacked[38],
        'inv_rpm':            int(round(unpacked[39] / 10.0)),
        'inv_speed_actual':   unpacked[40],
        'inv_current_actual': unpacked[41],
    }



# ================== SNAPSHOT → GLOBAL STATE ==================
def parse_snapshot(snap: dict, kind: Optional[int] = None) -> None:
    """
    Update latest_data_dict and AMSModule objects from a freshly decoded snapshot.
    Maintains backward-compatible keys so existing UI code keeps working.
    """
    global data_str, ams_modules, _last_received_snap_seq, _lqi_history

    if not snap:
        return

    latest_data_dict['snapshot'] = snap

    vmin = snap.get('vmin_modulo',     [])
    vmax = snap.get('vmax_modulo',     [])
    tmax = snap.get('temp_max_modulo', [])

    ts = time.time()
    for i, mod in enumerate(ams_modules):
        if i < len(vmin): mod.min_cell_mv = vmin[i]
        if i < len(vmax): mod.max_cell_mv = vmax[i]
        if i < len(tmax): mod.max_temp_c  = float(tmax[i])
        mod.last_update_ts = ts

    # Single pass over tmax for all derived stats
    valid_tmax = [t for t in tmax if t != 0]
    if valid_tmax:
        tmax_max = max(valid_tmax)
        tmax_min = min(valid_tmax)
        tmax_avg = sum(valid_tmax) / len(valid_tmax)
    else:
        tmax_max = tmax_min = tmax_avg = 0

    latest_data_dict['ams_summary'] = {
        'min_cell_mv': snap.get('v_cell_min_mV', 0),
        'max_cell_mv': max(vmax) if vmax else 0,
        'stack_mv':    0,
    }
    latest_data_dict['ams_current'] = {
        'current_A': snap.get('corriente_accu', 0),
    }
    latest_data_dict['ams_temp_summary'] = {
        'max_temp_c': tmax_max,
        'min_temp_c': tmax_min,
        'avg_temp_c': tmax_avg,
    }

    # Calculate LQI rolling success rate for both fast and flat-snapshot packets
    if kind in (KIND_FAST, KIND_SNAPSHOT):
        seq = snap.get('seq', 0)
        if _last_received_snap_seq is not None:
            diff = (seq - _last_received_snap_seq) & 0xFFFF
            if 0 < diff < 100:
                for _ in range(diff - 1):
                    _lqi_history.append(False)
                _lqi_history.append(True)
            else:
                _lqi_history.append(True)
        else:
            _lqi_history.append(True)
        _last_received_snap_seq = seq

    lqi = (sum(_lqi_history) / len(_lqi_history) * 100.0) if _lqi_history else 100.0
    latest_data_dict['lqi'] = round(lqi, 1)

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


# ================== POST-RACE DATA INJECTION ==================
# ─────────────────────────────────────────────────────────────
# GPS: parse NMEA 0183 log from micro-SD card and merge into session CSV
# AMS: per-cell temperature injection (format TBD — stub provided)
# ─────────────────────────────────────────────────────────────

def _parse_latlon(ddmm: str, hemi: str, is_lon: bool) -> Optional[float]:
    """
    Convert NMEA ddmm.mmmm + hemisphere char to signed decimal degrees.
    Returns None on parse error.
    """
    if not ddmm or not hemi:
        return None
    try:
        dot = ddmm.index('.')
    except ValueError:
        return None
    deg_digits = 3 if is_lon else 2
    if dot < (deg_digits + 2):
        return None
    try:
        degrees = float(ddmm[:deg_digits])
        minutes = float(ddmm[deg_digits:])
    except ValueError:
        return None
    dec = degrees + minutes / 60.0
    if hemi in ('S', 'W'):
        dec = -dec
    return dec


def parse_nmea_log(filepath: Path) -> pd.DataFrame:
    """
    Parse an NMEA 0183 log file (one sentence per line, from micro-SD logger).
    Accepts .nmea / .txt / .log / .csv files.
    Returns a DataFrame with columns:
      [datetime_utc, gps_lat_deg, gps_lon_deg, gps_sog_knots,
       gps_cog_deg, gps_sats, gps_fix]

    GPRMC / GNRMC provide position, SOG, COG and the full UTC date+time.
    GPGGA / GNGGA provide satellite count (cached between RMC sentences).
    """
    records   = []
    sats_cache = 0
    date_cache: Optional[tuple] = None  # (year, month, day)

    with open(filepath, 'r', errors='ignore') as fh:
        for raw_line in fh:
            line = raw_line.strip()
            if not line.startswith('$'):
                continue

            # Strip NMEA checksum
            star = line.rfind('*')
            clean = line[:star] if star != -1 else line
            parts = clean.split(',')
            if not parts:
                continue

            sid = parts[0].upper()

            # ── GPRMC / GNRMC ────────────────────────────────────────────────
            if sid in ('$GPRMC', '$GNRMC') and len(parts) >= 10:
                try:
                    time_str = parts[1]           # HHMMSS.ss
                    status   = parts[2].upper()   # A = valid fix
                    lat_raw, lat_hem = parts[3], parts[4]
                    lon_raw, lon_hem = parts[5], parts[6]
                    sog      = float(parts[7]) if parts[7] else 0.0
                    cog      = float(parts[8]) if parts[8] else 0.0
                    date_str = parts[9]           # DDMMYY

                    # Decode date
                    if len(date_str) == 6:
                        date_cache = (
                            2000 + int(date_str[4:6]),
                            int(date_str[2:4]),
                            int(date_str[0:2]),
                        )

                    # Decode time
                    hh = int(time_str[0:2])
                    mm = int(time_str[2:4])
                    ss_f = float(time_str[4:]) if len(time_str) > 4 else 0.0
                    us   = int((ss_f % 1) * 1_000_000)
                    ss   = int(ss_f)

                    yr, mo, dy = date_cache if date_cache else (2000, 1, 1)
                    dt_utc = datetime(yr, mo, dy, hh, mm, ss, us)

                    fix = (status == 'A')
                    lat = _parse_latlon(lat_raw, lat_hem, is_lon=False) if fix else None
                    lon = _parse_latlon(lon_raw, lon_hem, is_lon=True)  if fix else None

                    records.append({
                        'datetime_utc':  dt_utc,
                        'gps_lat_deg':   lat  if lat is not None else float('nan'),
                        'gps_lon_deg':   lon  if lon is not None else float('nan'),
                        'gps_sog_knots': sog,
                        'gps_cog_deg':   cog,
                        'gps_sats':      sats_cache,
                        'gps_fix':       int(fix),
                    })
                except (ValueError, IndexError, TypeError):
                    continue

            # ── GPGGA / GNGGA — satellite count ──────────────────────────────
            elif sid in ('$GPGGA', '$GNGGA') and len(parts) >= 8:
                try:
                    sats_cache = int(parts[7]) if parts[7] else 0
                except ValueError:
                    pass

    if not records:
        return pd.DataFrame(columns=['datetime_utc'] + GPS_MERGE_COLS)

    df = pd.DataFrame(records)
    df.sort_values('datetime_utc', inplace=True)
    df.reset_index(drop=True, inplace=True)
    return df


def merge_gps_into_session(
    session_path: Path,
    gps_file_path: Path,
    utc_offset_hours: float = 0.0,
) -> Tuple[bool, str]:
    """
    Merge GPS coordinates from an NMEA log (micro-SD) into an existing
    session CSV.

    Strategy: parse the session 'time' column (local datetime) and the
    NMEA UTC datetimes.  Apply utc_offset_hours to the GPS times to convert
    them to local time, then use pandas merge_asof (nearest, ≤5 s tolerance)
    to align rows.

    Adds/overwrites columns: GPS_MERGE_COLS
    Returns (success: bool, message: str).
    """
    try:
        session_df = pd.read_csv(session_path)
        if 'time' not in session_df.columns:
            return False, "Session CSV missing 'time' column."

        # Parse session timestamps (local time, timezone-naive)
        session_df['_dt'] = pd.to_datetime(session_df['time'], errors='coerce')
        if session_df['_dt'].isna().all():
            return False, "Could not parse 'time' column as datetime."

        # Parse GPS file
        gps_df = parse_nmea_log(gps_file_path)
        if gps_df.empty:
            return False, "No valid GPS sentences found in the NMEA file."

        total_fixes = int(gps_df['gps_fix'].sum())
        if total_fixes == 0:
            return False, "NMEA file found but contained 0 valid fixes (status=V)."

        # Apply UTC offset to convert GPS UTC → local time
        offset = timedelta(hours=utc_offset_hours)
        gps_df['_dt'] = gps_df['datetime_utc'] + offset
        gps_df.sort_values('_dt', inplace=True)
        gps_df.reset_index(drop=True, inplace=True)

        # Sort session by timestamp for merge_asof
        orig_order = session_df.index.copy()
        session_df.sort_values('_dt', inplace=True)

        # Drop any previously injected GPS columns to avoid duplicates
        for col in GPS_MERGE_COLS:
            if col in session_df.columns:
                session_df.drop(columns=[col], inplace=True)

        # Nearest-neighbour time join (tolerance = 5 seconds)
        merged = pd.merge_asof(
            session_df,
            gps_df[['_dt'] + GPS_MERGE_COLS],
            on='_dt',
            direction='nearest',
            tolerance=pd.Timedelta(seconds=5),
        )

        # Restore original row order and drop helper column
        merged = merged.loc[orig_order.values] if False else merged  # keep sorted
        merged.drop(columns=['_dt'], inplace=True)
        merged.to_csv(session_path, index=False)

        matched = int(merged['gps_fix'].notna().sum()) if 'gps_fix' in merged.columns else 0
        return True, (
            f"GPS merged — {total_fixes} fixes in file, "
            f"{matched}/{len(merged)} session rows matched (≤5 s)."
        )

    except Exception as exc:
        logger.exception("[POST-RACE] GPS merge error")
        return False, f"Error: {exc}"


def merge_ams_temps_into_session(
    session_path: Path,
    ams_file_path: Path,
) -> Tuple[bool, str]:
    """
    Inject per-cell AMS temperature data (19 cells × 5 modules = 95 values)
    from a micro-SD log file into the existing session CSV.
    Searches for the optimal tick_ms offset to align the two streams.
    """
    try:
        if not session_path.exists():
            return False, f"Session CSV not found: {session_path}"
        if not ams_file_path.exists():
            return False, f"AMS log file not found: {ams_file_path}"

        session_df = pd.read_csv(session_path)
        ams_df = pd.read_csv(ams_file_path)

        if 'tick_ms' not in session_df.columns:
            return False, "Session CSV missing 'tick_ms' column."
        if 'tick_ms' not in ams_df.columns:
            return False, "AMS SD-card log missing 'tick_ms' column."

        s_ticks = session_df['tick_ms'].values
        a_ticks = ams_df['tick_ms'].values

        s_val = None
        a_val = None

        # Check for accumulator current or min cell voltage to use as alignment signal
        if 'corriente_accu' in session_df.columns and 'I_filt_mA' in ams_df.columns:
            s_val = session_df['corriente_accu'].values * 1000.0  # A to mA
            a_val = ams_df['I_filt_mA'].values
        elif 'v_cell_min_mV' in session_df.columns and 'vmin_mV' in ams_df.columns:
            s_val = session_df['v_cell_min_mV'].values
            a_val = ams_df['vmin_mV'].values

        best_offset = 0
        if s_val is not None and a_val is not None and len(s_val) > 0 and len(a_val) > 0:
            min_mae = float('inf')
            offsets = np.arange(-60000, 60000, 100)
            for offset in offsets:
                shifted_ticks = s_ticks + offset
                interp_val = np.interp(shifted_ticks, a_ticks, a_val, left=a_val[0], right=a_val[-1])
                mae = np.mean(np.abs(s_val - interp_val))
                if mae < min_mae:
                    min_mae = mae
                    best_offset = offset
            logger.info(f"[POST-RACE] Found best tick_ms offset: {best_offset} ms (MAE={min_mae:.2f})")
        else:
            logger.info("[POST-RACE] Common signal not found or empty. Assuming offset = 0.")

        # Shift ams_df tick_ms by best_offset to align with session_df
        ams_df['_aligned_tick'] = ams_df['tick_ms'] - best_offset

        # Drop any previously injected AMS temperature columns to avoid duplicates
        for col in AMS_TEMP_COLS:
            if col in session_df.columns:
                session_df.drop(columns=[col], inplace=True)

        # Prepare ams_df columns to merge (map t{m}_{c} to ams_t_mod{m}_cell{c})
        cols_to_merge = ['_aligned_tick']
        rename_map = {}
        for m in range(NUM_MODULES):
            for c in range(CELLS_PER_MODULE):
                src_col = f't{m}_{c}'
                dest_col = f'ams_t_mod{m}_cell{c}'
                if src_col in ams_df.columns:
                    cols_to_merge.append(src_col)
                    rename_map[src_col] = dest_col

        ams_subset = ams_df[cols_to_merge].rename(columns=rename_map)
        ams_subset.sort_values('_aligned_tick', inplace=True)

        # Sort session_df by tick_ms for merge_asof
        orig_order = session_df.index.copy()
        session_df.sort_values('tick_ms', inplace=True)

        # Nearest-neighbour join based on tick_ms (tolerance of 5 seconds = 5000 ms)
        merged = pd.merge_asof(
            session_df,
            ams_subset,
            left_on='tick_ms',
            right_on='_aligned_tick',
            direction='nearest',
            tolerance=5000,
        )

        merged.drop(columns=['_aligned_tick'], inplace=True, errors='ignore')
        # Restore original order
        merged = merged.loc[orig_order.values]
        merged.to_csv(session_path, index=False)

        matched = int(merged[AMS_TEMP_COLS[0]].notna().sum()) if AMS_TEMP_COLS[0] in merged.columns else 0
        return True, (
            f"AMS Temps merged — Offset: {best_offset} ms. "
            f"{matched}/{len(merged)} session rows matched (<=5 s)."
        )

    except Exception as exc:
        logger.exception("[POST-RACE] AMS merge error")
        return False, f"Error: {exc}"


# ================== MAIN RECEIVE LOOP ==================
def receive_data(bucket_id: str,
                 piloto: str,
                 circuito: str,
                 port: Optional[str] = DEFAULT_PORT,
                 baud: int = DEFAULT_BAUD,
                 use_influx: bool = False,
                 debug: bool = False) -> None:

    global new_data_flag, _last_seq, _last_seq_advance_ts, _excel_logger, DEBUG_ENABLE_DEFAULT, _last_received_snap_seq, _lqi_history

    new_data_flag = 0
    _last_received_snap_seq = None
    _lqi_history.clear()
    latest_data_dict['lqi'] = 100.0
    latest_data_dict['snapshot'] = {
        'tick_ms':            0,
        'seq':                0,
        'start_button':       0,
        'apps1_raw':          0,
        'apps2_raw':          0,
        'brake_raw':          0,
        'torque_pct':         0,
        'ev_2_3':             0,
        't11_8_9':            0,
        'state':              0,
        'ok_precharge':       0,
        'ams_fsm_state':      0,
        'v_cell_min_mV':      0,
        'soc':                0,
        'vmin_modulo':        [0] * 5,
        'vmax_modulo':        [0] * 5,
        'corriente_accu':     0,
        'corriente_dcdc':     0,
        'temp_dcdc':          0,
        'temp_max_modulo':    [0] * 5,
        'inv_state':          0,
        'last_vconfig_tick':  0,
        'inv_error':          0,
        'inv_dc_bus_V':       0,
        'inv_temp_motor1':    0,
        'inv_temp_pwrstg':    0,
        'inv_temp_board':     0,
        'inv_rpm':            0,
        'inv_speed_actual':   0,
        'inv_current_actual': 0,
        'imu_ax_g':           0.0,
        'imu_ay_g':           0.0,
        'imu_az_g':           0.0,
        'imu_gx_dps':         0.0,
        'imu_gy_dps':         0.0,
        'imu_gz_dps':         0.0,
        'imu_roll_deg':       0.0,
        'imu_pitch_deg':      0.0,
    }

    DEBUG_ENABLE_DEFAULT = debug
    logger.setLevel(logging.DEBUG if debug else logging.INFO)
    logger.info("Recepción USB-Serial iniciada (protocolo fragmentado v3). Marple Upload: %s", use_influx)

    _excel_logger = SerialCSVLogger(bucket_id, piloto, circuito)

    counters = {
        "rx":        0,
        "frag_ok":   0,
        "frag_drop": 0,
        "snapshot":  0,
        "timeout":   0,
        "len":       0,
        "short":     0,
        "chk":       0,
    }
    last_stats_t = time.time()
    last_log_t   = time.time()

    _set_badge("STALE", "esperando primer frame")

    ser = None
    try:
        while new_data_flag != -1:
            now = time.time()

            # Attempt to establish / restore connection
            if ser is None:
                _set_badge("STALE", "reconectando...")
                target_port = port if port else _auto_detect_port()
                if target_port is None:
                    logger.warning("[RECONNECT] No port detected. Retrying in 2.0s...")
                    time.sleep(2.0)
                    continue
                try:
                    logger.info("[RECONNECT] Attempting to open port %s @ %d...", target_port, baud)
                    ser = _open_serial(target_port, baud)
                    logger.info("[RECONNECT] Port %s opened successfully.", target_port)
                    _set_badge("LIVE", "re-conectado")
                except Exception as e:
                    logger.warning("[RECONNECT] Failed to open port: %s. Retrying in 2.0s...", e)
                    time.sleep(2.0)
                    continue

            # Read frame from active serial connection
            try:
                payload, err = _read_frame(ser, counters=counters)
            except Exception as rx_ex:
                logger.warning("[RX ERROR] Serial communication error: %s. Reconnecting...", rx_ex)
                _set_badge("STALE", "reconectando...")
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                time.sleep(1.0)
                continue

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

            if not _validate_fragment(payload):
                counters["frag_drop"] += 1
                _set_badge("BAD", "frag_invalid")
                continue

            counters["frag_ok"] += 1

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

            result = _process_fragment(payload)
            if result is None:
                continue

            kind, seq, snapshot_bytes = result
            counters["snapshot"] += 1

            snap_update = {}
            if kind == KIND_SNAPSHOT:
                snap_update = _decode_flat_snapshot(snapshot_bytes, seq)
            elif kind == KIND_FAST:
                snap_update = _decode_fast_snapshot(snapshot_bytes, seq)
            elif kind == KIND_SLOW:
                snap_update = _decode_slow_snapshot(snapshot_bytes, seq)

            if not snap_update:
                _set_badge("BAD", "decode_fail")
                continue

            latest_data_dict['snapshot'].update(snap_update)
            parse_snapshot(latest_data_dict['snapshot'], kind=kind)
            new_data_flag = 1

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
            if ser is not None:
                ser.close()
        except Exception:
            pass

        file_path: Optional[str] = None
        if _excel_logger:
            file_path = _excel_logger.close()

        logger.info("Recepción USB-Serial finalizada. Snapshots: %d", counters["snapshot"])

        if use_influx and file_path:
            logger.info("Iniciando subida a Marple Data en segundo plano...")
            def _upload_bg():
                try:
                    isc_marple.upload_session_csv(file_path, {
                        "piloto":   piloto,
                        "circuito": circuito,
                        "type":     "Real_Telemetry",
                        "date":     datetime.now().isoformat(),
                    })
                except Exception as upload_err:
                    logger.error("Error en la subida a Marple en segundo plano: %s", upload_err)

            threading.Thread(target=_upload_bg, daemon=True).start()