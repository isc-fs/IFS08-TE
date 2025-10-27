"""
ISC RTT Serial - Enhanced with comprehensive Excel logging
"""

from __future__ import annotations
import os
import time
import struct
import logging
from datetime import datetime
from typing import Optional, Dict, Any, List
from pathlib import Path

import serial
import serial.tools.list_ports

import pandas as pd
import numpy as np

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

# ================== AMS CONSTANTS ==================
NUM_MODULES = 5
CELLS_PER_MODULE = 19
TEMPS_PER_MODULE = 38
TOTAL_TEMPS = NUM_MODULES * TEMPS_PER_MODULE  # 190

# ================== EXCEL LOGGING ==================
EXCEL_SESSIONS_DIR = Path("logs")
EXCEL_SESSIONS_DIR.mkdir(exist_ok=True)

# ================== CONFIG DERIVADOS ==================
GEAR_RATIO: Optional[float]     = None
FINAL_DRIVE: Optional[float]    = None
WHEEL_RADIUS_M: Optional[float] = None

# ================== INFLUX (OPCIONAL) ==================
INFLUX_CONFIG = {
    "url":   "http://localhost:8086",
    "token": "TOKEN",
    "org":   "TORG",
}
INFLUX_ENABLE_DEFAULT = False

_client = None
_influx_ok = False

def _init_influx():
    global _client, _influx_ok
    if _client is not None:
        return
    try:
        from influxdb_client import InfluxDBClient
        _client = InfluxDBClient(**INFLUX_CONFIG)
        _influx_ok = True
        logging.getLogger("ISC_RTT_USB").info("InfluxDB inicializado")
    except Exception as e:
        _client = None
        _influx_ok = False
        logging.getLogger("ISC_RTT_USB").warning(f"Influx deshabilitado: {e}")

def _get_write_api():
    if not _influx_ok or _client is None:
        return None
    try:
        from influxdb_client.client.write_api import SYNCHRONOUS
        return _client.write_api(write_options=SYNCHRONOUS)
    except Exception as e:
        logging.getLogger("ISC_RTT_USB").warning(f"No se pudo crear write_api: {e}")
        return None

# ================== MARCO SERIAL ==================
SOF1 = 0xAA
SOF2 = 0x55
PAYLOAD_LEN = 32
DEFAULT_BAUD = 115200
DEFAULT_PORT = None

# ================== ESTADO PARA LA UI ==================
data_str = ""
new_data_flag = 0
latest_data_dict: dict = {}

# Badge status
_status = {
    "badge": "STALE",
    "reason": "inicio",
    "ts": 0,
}
_last_seq = None
_last_seq_advance_ts = 0.0
_STALE_T = 0.20

logger = logging.getLogger("ISC_RTT_USB")
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

# ================== PER-MODULE AMS DATA ==================
class AMSModule:
    def __init__(self, module_id: int):
        self.module_id = module_id
        self.cell_voltages_mv = [0] * CELLS_PER_MODULE
        self.temps_c = [np.nan] * TEMPS_PER_MODULE
        self.min_cell_mv = 0
        self.max_cell_mv = 0
        self.min_temp_c = 0
        self.max_temp_c = 0
        self.last_update_ts = 0.0

ams_modules: List[AMSModule] = [AMSModule(i) for i in range(NUM_MODULES)]

# Global AMS summary
ams_global_min_mv = 0
ams_global_max_mv = 0
ams_stack_total_mv = 0
ams_current_dA = 0

# ================== EXCEL SESSION LOGGER ==================
class ExcelSessionLogger:
    """Comprehensive Excel logger for telemetry sessions"""
    def __init__(self, bucket_id: str, piloto: str, circuito: str):
        self.bucket_id = bucket_id
        self.piloto = piloto
        self.circuito = circuito
        self.session_start = datetime.now()
        
        # Create session filename in logs folder
        self.filename = EXCEL_SESSIONS_DIR / f"{bucket_id}.xlsx"
        
        # Data buffers for each sheet
        self.data_buffers = {
            'Main': [],
            'Motor_Inverter': [],
            'AMS_Summary': [],
            'AMS_Modules': [],
            'Driver': [],
            'Temperatures': []
        }
        
        self.last_write = time.time()
        self.write_interval = 5.0  # Write to Excel every 5 seconds
        
        logger.info(f"Excel logger initialized: {self.filename}")
    
    def log_data(self, data_dict: dict):
        """Log data from latest_data_dict"""
        timestamp = datetime.now()
        
        # Main sheet - Overview data
        main_row = {
            'timestamp': timestamp,
            'piloto': self.piloto,
            'circuito': self.circuito,
        }
        
        # Extract data from 0x600
        data_600 = data_dict.get('0x600', {})
        main_row.update({
            'dc_bus_voltage': data_600.get('dc_bus_voltage', 0),
            'rpm': data_600.get('rpm', 0),
            'torque_total': data_600.get('torque_total', 0),
            'cell_min_v': data_600.get('cell_min_v', 0),
            'throttle_raw1': data_600.get('throttle_raw1', 0),
            'throttle_raw2': data_600.get('throttle_raw2', 0),
        })
        self.data_buffers['Main'].append(main_row)
        
        # Motor/Inverter sheet - 0x610
        data_610 = data_dict.get('0x610', {})
        motor_row = {
            'timestamp': timestamp,
            'motor_temp': data_610.get('motor_temp', 0),
            'pwrstg_temp': data_610.get('pwrstg_temp', 0),
            'air_temp': data_610.get('air_temp', 0),
            'n_actual': data_610.get('n_actual', 0),
            'i_actual': data_610.get('i_actual', 0),
        }
        self.data_buffers['Motor_Inverter'].append(motor_row)
        
        # Driver sheet - 0x620 and 0x630
        data_620 = data_dict.get('0x620', {})
        data_630 = data_dict.get('0x630', {})
        driver_row = {
            'timestamp': timestamp,
            's1_raw': data_620.get('s1_raw', 0),
            's2_raw': data_620.get('s2_raw', 0),
            'brake_raw': data_620.get('brake_raw', 0),
            'throttle_pct': data_630.get('throttle', 0),
            'brake_pct': data_630.get('brake', 0),
            'torque_req': data_630.get('torque_req', 0),
            'torque_est': data_630.get('torque_est', 0),
            'start_button': data_620.get('start_button', 0),
            'precharge_button': data_620.get('precharge_button', 0),
        }
        self.data_buffers['Driver'].append(driver_row)
        
        # AMS Summary sheet
        ams_summary = data_dict.get('ams_summary', {})
        ams_temp_summary = data_dict.get('ams_temp_summary', {})
        ams_row = {
            'timestamp': timestamp,
            'global_min_mv': ams_global_min_mv,
            'global_max_mv': ams_global_max_mv,
            'stack_mv': ams_stack_total_mv,
            'current_A': ams_current_dA / 10.0,
            'max_temp_c': ams_temp_summary.get('max_temp_c', 0),
            'min_temp_c': ams_temp_summary.get('min_temp_c', 0),
            'avg_temp_c': ams_temp_summary.get('avg_temp_c', 0),
        }
        self.data_buffers['AMS_Summary'].append(ams_row)
        
        # AMS Modules sheet - detailed per-module data
        for i in range(NUM_MODULES):
            mod = ams_modules[i]
            module_row = {
                'timestamp': timestamp,
                'module_id': i,
                'min_cell_mv': mod.min_cell_mv,
                'max_cell_mv': mod.max_cell_mv,
                'min_temp_c': mod.min_temp_c,
                'max_temp_c': mod.max_temp_c,
            }
            self.data_buffers['AMS_Modules'].append(module_row)
        
        # Temperatures sheet - all 190 temperatures
        temp_row = {'timestamp': timestamp}
        all_temps = get_all_temps_array()
        for i in range(min(len(all_temps), TOTAL_TEMPS)):
            module_id = i // TEMPS_PER_MODULE
            sensor_id = i % TEMPS_PER_MODULE
            temp_row[f'M{module_id}_T{sensor_id}'] = all_temps[i]
        self.data_buffers['Temperatures'].append(temp_row)
        
        # Write to Excel periodically
        if time.time() - self.last_write > self.write_interval:
            self.write_to_excel()
            self.last_write = time.time()
    
    def write_to_excel(self):
        """Write all buffered data to Excel file"""
        if not any(len(buf) > 0 for buf in self.data_buffers.values()):
            return
        
        try:
            with pd.ExcelWriter(self.filename, engine='openpyxl', mode='a' if self.filename.exists() else 'w') as writer:
                for sheet_name, data_list in self.data_buffers.items():
                    if len(data_list) > 0:
                        df = pd.DataFrame(data_list)
                        
                        # If file exists, append to existing sheet
                        if self.filename.exists() and sheet_name in pd.ExcelFile(self.filename).sheet_names:
                            existing_df = pd.read_excel(self.filename, sheet_name=sheet_name)
                            df = pd.concat([existing_df, df], ignore_index=True)
                        
                        df.to_excel(writer, sheet_name=sheet_name, index=False)
            
            # Clear buffers after successful write
            for key in self.data_buffers:
                self.data_buffers[key] = []
            
            logger.info(f"Data written to Excel: {self.filename}")
        
        except Exception as e:
            logger.error(f"Error writing to Excel: {e}")
    
    def finalize(self):
        """Final write and close"""
        self.write_to_excel()
        
        # Write session metadata sheet
        try:
            metadata = {
                'Session ID': [self.bucket_id],
                'Piloto': [self.piloto],
                'Circuito': [self.circuito],
                'Start Time': [self.session_start],
                'End Time': [datetime.now()],
                'Duration (min)': [(datetime.now() - self.session_start).total_seconds() / 60],
            }
            df_meta = pd.DataFrame(metadata)
            
            with pd.ExcelWriter(self.filename, engine='openpyxl', mode='a') as writer:
                df_meta.to_excel(writer, sheet_name='Metadata', index=False)
            
            logger.info(f"Session finalized: {self.filename}")
        except Exception as e:
            logger.error(f"Error writing metadata: {e}")

# Global logger instance
_excel_logger: Optional[ExcelSessionLogger] = None

# ================== UTILIDADES ==================
def _dump_hex(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)

def list_serial_ports():
    out = []
    for p in serial.tools.list_ports.comports():
        out.append((p.device, p.description))
    return out

def list_excel_sessions():
    """List all available Excel session files"""
    sessions = []
    if EXCEL_SESSIONS_DIR.exists():
        for file in EXCEL_SESSIONS_DIR.glob("*.xlsx"):
            if not file.name.startswith('~'):  # Skip temp files
                sessions.append(file)
    return sorted(sessions, key=lambda x: x.stat().st_mtime, reverse=True)

def load_excel_session(filepath: Path) -> Dict[str, pd.DataFrame]:
    """Load all sheets from an Excel session file"""
    try:
        excel_file = pd.ExcelFile(filepath)
        sheets = {}
        for sheet_name in excel_file.sheet_names:
            sheets[sheet_name] = pd.read_excel(filepath, sheet_name=sheet_name)
        return sheets
    except Exception as e:
        logger.error(f"Error loading Excel session: {e}")
        return {}

def _auto_detect_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = (p.description or "").upper()
        hwid = (p.hwid or "").upper()
        if "CH340" in desc or "USB-SERIAL" in desc or "CP210" in desc:
            return p.device
    return ports[0].device if ports else None

def _open_serial(port, baud):
    ser = serial.Serial(port=port, baudrate=baud, timeout=0.2)
    time.sleep(1.5)
    ser.reset_input_buffer()
    return ser

def _set_badge(badge: str, reason: str):
    global _status
    _status = {
        "badge": badge,
        "reason": reason,
        "ts": int(time.time() * 1000),
    }
    latest_data_dict["__STATUS__"] = _status

def _mod16_diff(curr: int, prev: int) -> int:
    return (curr - prev) & 0xFFFF

TEST_PATTERN = bytes(range(0xA0, 0xA0 + PAYLOAD_LEN))

def _is_consecutive_ramp(payload: bytes) -> bool:
    if len(payload) != PAYLOAD_LEN:
        return False
    return all(((payload[i] - payload[i-1]) & 0xFF) == 1 for i in range(1, PAYLOAD_LEN))

def is_test_payload(payload: bytes) -> bool:
    return payload == TEST_PATTERN or _is_consecutive_ramp(payload)

def _read_frame(ser, counters=None):
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
        _ = ser.read(min(ln[0], 255))
        _ = ser.read(1)
        return None, "len"

    payload = ser.read(PAYLOAD_LEN)
    if len(payload) != PAYLOAD_LEN:
        if counters is not None: counters["short"] += 1
        return None, "short"

    chk = ser.read(1)
    if not chk:
        if counters is not None: counters["timeout"] += 1
        return None, "timeout"

    c = 0
    for bb in payload:
        c ^= bb
    if c != chk[0]:
        if counters is not None: counters["chk"] += 1
        return None, "chk"

    return payload, None

# ================== DECODIFICACIÓN PAYLOAD ==================
def _decode_payload(payload: bytes):
    assert len(payload) == PAYLOAD_LEN

    try:
        id_u16, seq_u16, v1, v2, v3, v4, v5, v6, v7 = struct.unpack("<HHfffffff", payload)
        return {
            "id": int(id_u16),
            "seq": int(seq_u16),
            "v1": float(v1), "v2": float(v2), "v3": float(v3),
            "v4": float(v4), "v5": float(v5), "v6": float(v6), "v7": float(v7),
            "raw_floats": (float(id_u16), float(seq_u16), v1, v2, v3, v4, v5, v6, v7),
            "fmt": "telframe",
        }
    except struct.error:
        pass

    try:
        f = struct.unpack("<ffffffff", payload)
        id_guess = int(f[0]) & 0xFFFF
        return {
            "id": id_guess,
            "seq": None,
            "v1": float(f[1]), "v2": float(f[2]), "v3": float(f[3]),
            "v4": float(f[4]), "v5": float(f[5]), "v6": float(f[6]), "v7": float(f[7]),
            "raw_floats": f,
            "fmt": "legacy",
        }
    except struct.error:
        return None

# ================== PARSER LÓGICO Y MAPEOS ==================
def _id_hex(id_int: int) -> str:
    return f"0x{id_int:X}"

def parse_ams_extended(id_int: int, frame_dict: dict):
    """Parse específico para IDs del AMS (0x202-0x20D)"""
    global ams_global_min_mv, ams_global_max_mv, ams_stack_total_mv, ams_current_dA
    
    v1, v2, v3, v4, v5, v6, v7 = (frame_dict["v1"], frame_dict["v2"], frame_dict["v3"],
                                   frame_dict["v4"], frame_dict["v5"], frame_dict["v6"], frame_dict["v7"])
    
    if id_int == 0x202:
        ams_global_max_mv = int(v1)
        ams_global_min_mv = int(v2)
        ams_stack_total_mv = int(v3)
        latest_data_dict["ams_summary"] = {
            "max_cell_mv": ams_global_max_mv,
            "min_cell_mv": ams_global_min_mv,
            "stack_mv": ams_stack_total_mv,
        }
    
    elif 0x203 <= id_int <= 0x207:
        block_idx = id_int - 0x203
        module_idx = block_idx // 5
        cell_offset = (block_idx % 5) * 4
        
        if module_idx < NUM_MODULES:
            cells = [int(v1), int(v2), int(v3), int(v4)]
            for i, cell_mv in enumerate(cells):
                if cell_offset + i < CELLS_PER_MODULE and cell_mv > 0:
                    ams_modules[module_idx].cell_voltages_mv[cell_offset + i] = cell_mv
            
            valid_cells = [c for c in ams_modules[module_idx].cell_voltages_mv if c > 0]
            if valid_cells:
                ams_modules[module_idx].min_cell_mv = min(valid_cells)
                ams_modules[module_idx].max_cell_mv = max(valid_cells)
            ams_modules[module_idx].last_update_ts = time.time()
    
    elif id_int == 0x208:
        latest_data_dict["ams_temp_summary"] = {
            "max_temp_c": int(v1),
            "min_temp_c": int(v2),
            "avg_temp_c": float(v3) / 10.0,
            "valid_count": int(v4),
        }
    
    elif 0x209 <= id_int <= 0x20D:
        block_idx = id_int - 0x209
        module_idx = block_idx // 5
        temp_offset = (block_idx % 5) * 8
        
        if module_idx < NUM_MODULES:
            temps = [v1, v2, v3, v4, v5, v6, v7, 0]
            for i, temp_c in enumerate(temps[:8]):
                if temp_offset + i < TEMPS_PER_MODULE:
                    if 0 < temp_c < 150:
                        ams_modules[module_idx].temps_c[temp_offset + i] = float(temp_c)
            
            valid_temps = [t for t in ams_modules[module_idx].temps_c if not np.isnan(t) and t > 0]
            if valid_temps:
                ams_modules[module_idx].min_temp_c = min(valid_temps)
                ams_modules[module_idx].max_temp_c = max(valid_temps)
            ams_modules[module_idx].last_update_ts = time.time()
    
    elif id_int == 0x201:
        ams_current_dA = int(v1)
        latest_data_dict["ams_current"] = {"current_A": float(ams_current_dA) / 10.0}

def parse_telemetry_data_frame(frame_dict: dict):
    global data_str, latest_data_dict

    if not frame_dict:
        return None

    id_int = frame_dict["id"]
    id_hex = _id_hex(id_int)
    seq    = frame_dict["seq"]
    v1, v2, v3, v4, v5, v6, v7 = (frame_dict["v1"], frame_dict["v2"], frame_dict["v3"],
                                   frame_dict["v4"], frame_dict["v5"], frame_dict["v6"], frame_dict["v7"])

    if seq is not None:
        data_str = f"[RX] ID={id_hex} SEQ={seq} badge={_status.get('badge','?')}\n[RX] {v1:.2f}, {v2:.2f}, {v3:.2f}, {v4:.2f}, {v5:.2f}, {v6:.2f}, {v7:.2f}"
    else:
        data_str = f"[RX] ID={id_hex} badge={_status.get('badge','?')}\n[RX] {v1:.2f}, {v2:.2f}, {v3:.2f}, {v4:.2f}, {v5:.2f}, {v6:.2f}, {v7:.2f}"

    latest_data_dict[id_hex] = {
        "id": id_int, "seq": seq,
        "v1": v1, "v2": v2, "v3": v3, "v4": v4, "v5": v5, "v6": v6, "v7": v7,
    }

    # AMS extended parsing
    if 0x201 <= id_int <= 0x20D:
        parse_ams_extended(id_int, frame_dict)

    # Legacy mappings
    if id_int == 0x600:
        latest_data_dict[id_hex].update({
            "dc_bus_voltage": v1,
            "dc_bus_power":   v2,
            "rpm":            v3,
            "torque_total":   v4,
            "cell_min_v":     v5,
            "throttle_raw1":  v6,
            "throttle_raw2":  v7,
        })

    elif id_int == 0x610:
        latest_data_dict[id_hex].update({
            "motor_temp":   v1,
            "pwrstg_temp":  v2,
            "air_temp":     v3,
            "n_actual":     v4,
            "i_actual":     v5,
        })

    elif id_int == 0x620:
        latest_data_dict[id_hex].update({
            "s1_raw": v1, "s2_raw": v2,
            "brake_raw": v3,
            "precharge_button": v4,
            "start_button": v5,
        })

    elif id_int == 0x630:
        latest_data_dict[id_hex].update({
            "torque_req": v1,
            "torque_est": v2,
            "throttle":   max(0.0, min(100.0, v3)),
            "brake":      max(0.0, min(100.0, v4)),
        })

    elif id_int == 0x640:
        latest_data_dict[id_hex].update({
            "current_sensor": v1,
            "cell_min_v":     v2,
            "cell_max_temp":  v3,
        })

    try:
        from influxdb_client import Point
        pt = (
            Point("telemetry")
            .tag("id_hex", id_hex)
            .field("v1", float(v1))
            .field("v2", float(v2))
            .field("v3", float(v3))
            .field("v4", float(v4))
            .field("v5", float(v5))
            .field("v6", float(v6))
            .field("v7", float(v7))
        )
        if seq is not None:
            pt = pt.field("seq", int(seq))
        return pt
    except Exception:
        return None

# ================== API FOR UI ==================
def get_ams_module_data(module_idx: int) -> Optional[AMSModule]:
    """Returns data for a specific module (0-4)"""
    if 0 <= module_idx < NUM_MODULES:
        return ams_modules[module_idx]
    return None

def get_all_temps_array() -> np.ndarray:
    """Returns flattened array of all 190 temperatures organized by module"""
    temps = []
    for mod in ams_modules:
        temps.extend(mod.temps_c)
    return np.array(temps)

def create_bucket(piloto: str, circuito: str, use_influx: bool = INFLUX_ENABLE_DEFAULT) -> str:
    global INFLUX_ENABLE_DEFAULT
    INFLUX_ENABLE_DEFAULT = bool(use_influx)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    safe_piloto = piloto.replace(" ", "_")
    safe_circuito = circuito.replace(" ", "_")
    bucket_name = f"ISC_{ts}_{safe_piloto}_{safe_circuito}"

    if use_influx:
        _init_influx()

    return bucket_name

def get_latest_data(data_id: str = None):
    if data_id:
        return latest_data_dict.get(data_id, {})
    return latest_data_dict.copy()

# ================== RECEPCIÓN PRINCIPAL ==================
def receive_data(bucket_id: str,
                 piloto: str,
                 circuito: str,
                 port: str = DEFAULT_PORT,
                 baud: int = DEFAULT_BAUD,
                 use_influx: bool = INFLUX_ENABLE_DEFAULT,
                 debug: bool = False):
    global new_data_flag, _last_seq, _last_seq_advance_ts, _excel_logger

    logger.setLevel(logging.DEBUG if debug else logging.INFO)
    logger.info("Recepción USB-Serial iniciada con soporte para 5 módulos AMS")

    write_api = None
    if use_influx:
        _init_influx()
        write_api = _get_write_api()

    # Initialize Excel logger
    _excel_logger = ExcelSessionLogger(bucket_id, piloto, circuito)

    if port is None:
        port = _auto_detect_port()
    if port is None:
        raise RuntimeError("No se encontró un puerto serie RF-NANO.")

    ser = _open_serial(port, baud)

    logger.info("[CONFIG] Serial: port=%s, baud=%d", port, baud)
    logger.info("[CONFIG] AMS: %d módulos, %d celdas/mod, %d temps/mod", 
                NUM_MODULES, CELLS_PER_MODULE, TEMPS_PER_MODULE)
    logger.info("[CONFIG] Excel: %s", _excel_logger.filename)

    counters = {"rx": 0, "decode": 0, "timeout": 0, "len": 0, "short": 0, "chk": 0, "decode_fail": 0, "test": 0}
    last_check_t = time.time()
    last_stats_t = last_check_t
    last_excel_log = last_check_t

    _set_badge("STALE", "esperando primer frame")

    logger.info("Leyendo de %s @ %d bps", port, baud)

    try:
        while new_data_flag != -1:
            now = time.time()

            payload, err = _read_frame(ser, counters=counters)

            if err == "timeout":
                if _last_seq is not None and (now - _last_seq_advance_ts) > _STALE_T:
                    _set_badge("STALE", "sin avance de SEQ")
                if now - last_stats_t >= 2.0:
                    logger.debug("[STATS] rx=%d decode=%d chk=%d test=%d",
                                 counters["rx"], counters["decode"], counters["chk"], counters["test"])
                    last_stats_t = now
                continue
            elif err in ("len", "short", "chk"):
                _set_badge("BAD", err)
                continue

            if payload is None:
                continue

            if is_test_payload(payload):
                counters["test"] += 1
                continue

            counters["rx"] += 1

            decoded = _decode_payload(payload)
            if not decoded:
                counters["decode_fail"] += 1
                _set_badge("BAD", "decode_fail")
                continue

            counters["decode"] += 1

            if decoded["seq"] is not None:
                seq = decoded["seq"] & 0xFFFF
                if _last_seq is None:
                    _last_seq = seq
                    _last_seq_advance_ts = now
                    _set_badge("LIVE", "primer SEQ")
                else:
                    diff = _mod16_diff(seq, _last_seq)
                    if diff > 0:
                        _last_seq = seq
                        _last_seq_advance_ts = now
                        _set_badge("LIVE", f"SEQ +{diff}")
                    else:
                        if (now - _last_seq_advance_ts) > _STALE_T:
                            _set_badge("STALE", "SEQ detenido")
            else:
                _set_badge("LIVE", "legacy sin SEQ")

            pt = parse_telemetry_data_frame(decoded)

            if write_api and pt:
                try:
                    pt = pt.tag("piloto", piloto).tag("circuito", circuito)
                    write_api.write(bucket=bucket_id, record=pt)
                except Exception as e:
                    logger.warning("Error escribiendo en Influx: %s", e)

            # Log to Excel every second
            if now - last_excel_log >= 1.0:
                if _excel_logger:
                    _excel_logger.log_data(latest_data_dict)
                last_excel_log = now

            new_data_flag = 1

            if now - last_stats_t >= 2.0:
                logger.debug("[STATS] rx=%d decode=%d",
                             counters["rx"], counters["decode"])
                last_stats_t = now

    finally:
        try:
            ser.close()
        except Exception:
            pass
        
        # Finalize Excel logger
        if _excel_logger:
            _excel_logger.finalize()
        
        logger.info("Recepción USB-Serial finalizada.")
