"""
ISC RTT Serial - Full Implementation
Replaces Excel/Influx with Flat CSV Logging & Marple Data Upload.
Retains all original Serial management, parsing, and AMS logic.
"""

from __future__ import annotations
import os
import time
import struct
import logging
import csv
from datetime import datetime
from typing import Optional, Dict, Any, List
from pathlib import Path

import pandas as pd
import serial
import serial.tools.list_ports
import numpy as np

# Importamos el módulo de subida a Marple
import isc_marple

# ================== CONFIG RF ==================
RF_EXPECTED = {
    "PIPE_ADDR": "0xE7E7E7E7E7",
    "CHANNEL": 76,
    "PAYLOAD": 32,
    "DATA_RATE": "1Mbps",
    "AUTO_ACK": False,
    "CRC": "CRC_16",
    "PA": "PA_MAX",
}

# ================== AMS CONSTANTS ==================
NUM_MODULES = 5
CELLS_PER_MODULE = 19
TEMPS_PER_MODULE = 38
TOTAL_TEMPS = NUM_MODULES * TEMPS_PER_MODULE # 190

# ================== LOGGING CONSTANTS ==================
# Usamos una carpeta de logs locales antes de subir
LOG_DIR = Path("logs")
LOG_DIR.mkdir(exist_ok=True)

# ================== CONFIG DERIVADOS ==================
DEFAULT_BAUD = 115200
DEFAULT_PORT = None

# --- VARIABLES GLOBALES QUE FALTABAN ---
INFLUX_ENABLE_DEFAULT = False # Se usa en la UI para marcar el checkbox de Marple por defecto
DEBUG_ENABLE_DEFAULT = False  # Se usa en la UI para el checkbox de debug

# Variables globales para la UI y estado
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

# Configuración de Logging de Python (Consola/UI)
logger = logging.getLogger("ISC_RTT_USB")
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

# ================== MARCO SERIAL ==================
SOF1 = 0xAA
SOF2 = 0x55
PAYLOAD_LEN = 32
TEST_PATTERN = bytes(range(0xA0, 0xA0 + PAYLOAD_LEN))

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

# ================== CSV LOGGER (REEMPLAZA EXCEL/INFLUX) ==================
class SerialCSVLogger:
    """
    Logger que genera un CSV plano compatible con Marple Data.
    Sustituye al ExcelSessionLogger anterior.
    """
    def __init__(self, bucket_id: str, piloto: str, circuito: str):
        self.bucket_id = bucket_id
        self.piloto = piloto
        self.circuito = circuito
        self.start_time = time.time()
        
        # Nombre del archivo
        self.filename = LOG_DIR / f"{bucket_id}.csv"
        
        self.file = open(self.filename, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.record_count = 0
        
        # CABECERAS: Primera columna "time" para Marple
        self.headers = [
            "time", "time_elapsed_s",  # <--- CRÍTICO: "time"
            
            # 0x600 - Main
            "dc_bus_voltage", "rpm", "torque_total", "cell_min_v", "throttle_raw1", "throttle_raw2",
            
            # 0x610 - Motor
            "motor_temp", "pwrstg_temp", "air_temp", "n_actual", "i_actual",
            
            # 0x620 - Driver Raw
            "s1_raw", "s2_raw", "brake_raw", "precharge_btn", "start_btn",
            
            # 0x630 - Driver Proc
            "torque_req", "torque_est", "throttle_pct", "brake_pct",
            
            # AMS Summary
            "ams_min_cell_mv", "ams_max_cell_mv", "ams_stack_v", "ams_current_a", 
            "ams_max_temp_c", "ams_min_temp_c", "ams_avg_temp_c",
            
            # Dynamics (Placeholders)
            "g_long", "g_lat", "g_total",
            "susp_force_fl", "susp_force_fr", "susp_force_rl", "susp_force_rr",
            "susp_travel_fl", "susp_travel_fr", "susp_travel_rl", "susp_travel_rr",
            "brake_temp_fl", "brake_temp_fr", "brake_temp_rl", "brake_temp_rr"
        ]
        
        self.writer.writerow(self.headers)
        logger.info(f"CSV Logger iniciado: {self.filename}")

    def log_snapshot(self, data_dict: dict):
        """Toma una foto del estado actual y escribe una fila en el CSV"""
        current_ts = datetime.now().isoformat()
        elapsed = time.time() - self.start_time
        
        # Extracción segura de datos (con valores por defecto 0)
        d600 = data_dict.get('0x600', {})
        d610 = data_dict.get('0x610', {})
        d620 = data_dict.get('0x620', {})
        d630 = data_dict.get('0x630', {})
        
        # AMS data
        ams_sum = data_dict.get('ams_summary', {})
        ams_tmp = data_dict.get('ams_temp_summary', {})
        ams_cur = data_dict.get('ams_current', {})
        
        # Dynamics 
        d650 = data_dict.get('0x650', {})
        d660 = data_dict.get('0x660', {})
        d670 = data_dict.get('0x670', {})
        
        s_forces = d660.get('susp_forces', [0]*4)
        s_travel = d660.get('susp_travel', [0]*4)

        row = [
            current_ts, f"{elapsed:.3f}",
            
            # 0x600
            d600.get('dc_bus_voltage', 0), d600.get('rpm', 0), d600.get('torque_total', 0),
            d600.get('cell_min_v', 0), d600.get('throttle_raw1', 0), d600.get('throttle_raw2', 0),
            
            # 0x610
            d610.get('motor_temp', 0), d610.get('pwrstg_temp', 0), d610.get('air_temp', 0),
            d610.get('n_actual', 0), d610.get('i_actual', 0),
            
            # 0x620
            d620.get('s1_raw', 0), d620.get('s2_raw', 0), d620.get('brake_raw', 0),
            d620.get('precharge_button', 0), d620.get('start_button', 0),
            
            # 0x630
            d630.get('torque_req', 0), d630.get('torque_est', 0), d630.get('throttle', 0), d630.get('brake', 0),
            
            # AMS
            ams_sum.get('min_cell_mv', 0), ams_sum.get('max_cell_mv', 0), 
            ams_sum.get('stack_mv', 0) / 1000.0 if ams_sum.get('stack_mv') else 0, # Convert mV to V
            ams_cur.get('current_A', 0),
            ams_tmp.get('max_temp_c', 0), ams_tmp.get('min_temp_c', 0), ams_tmp.get('avg_temp_c', 0),
            
            # Dynamics
            d650.get('g_long', 0), d650.get('g_lat', 0), d650.get('g_total', 0),
            s_forces[0] if len(s_forces)>0 else 0, s_forces[1] if len(s_forces)>1 else 0, s_forces[2] if len(s_forces)>2 else 0, s_forces[3] if len(s_forces)>3 else 0,
            s_travel[0] if len(s_travel)>0 else 0, s_travel[1] if len(s_travel)>1 else 0, s_travel[2] if len(s_travel)>2 else 0, s_travel[3] if len(s_travel)>3 else 0,
            d670.get('brake_temp_fl', 0), d670.get('brake_temp_fr', 0), d670.get('brake_temp_rl', 0), d670.get('brake_temp_rr', 0)
        ]
        
        self.writer.writerow(row)
        self.record_count += 1
        
        if self.record_count % 50 == 0:
            self.file.flush()

    def close(self):
        if self.file:
            self.file.close()
            logger.info(f"CSV cerrado. Registros: {self.record_count}")
            return str(self.filename)
        return None

# ================== UTILIDADES SERIAL ==================
def _dump_hex(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)

def list_serial_ports():
    out = []
    for p in serial.tools.list_ports.comports():
        out.append((p.device, p.description))
    return out

def list_excel_sessions():
    """Ahora lista archivos CSV en lugar de Excel"""
    sessions = []
    if LOG_DIR.exists():
        for file in LOG_DIR.glob("*.csv"):
            sessions.append(file)
    return sorted(sessions, key=lambda x: x.stat().st_mtime, reverse=True)

def load_excel_session(filepath: Path) -> Dict[str, pd.DataFrame]:
    """Carga CSV para el visor de la UI (Adapta a formato dict)"""
    try:
        df = pd.read_csv(filepath)
        return {'Main': df} # Retornamos todo en una 'hoja' llamada Main
    except Exception as e:
        logger.error(f"Error loading CSV session: {e}")
        return {}

def _auto_detect_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = (p.description or "").upper()
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

def _is_consecutive_ramp(payload: bytes) -> bool:
    if len(payload) != PAYLOAD_LEN:
        return False
    return all(((payload[i] - payload[i-1]) & 0xFF) == 1 for i in range(1, PAYLOAD_LEN))

def is_test_payload(payload: bytes) -> bool:
    return payload == TEST_PATTERN or _is_consecutive_ramp(payload)

def _read_frame(ser, counters=None):
    """
    Lectura robusta del frame con SOF y Checksum
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

    # Consolidated Log String for UI
    status_badge = _status.get('badge', '?')
    if seq is not None:
        data_str = f"[{status_badge}] ID={id_hex} SEQ={seq} | V: {v1:.2f}, {v2:.2f}, {v3:.2f}, {v4:.2f}..."
    else:
        data_str = f"[{status_badge}] ID={id_hex} | V: {v1:.2f}, {v2:.2f}, {v3:.2f}..."

    latest_data_dict[id_hex] = {
        "id": id_int, "seq": seq,
        "v1": v1, "v2": v2, "v3": v3, "v4": v4, "v5": v5, "v6": v6, "v7": v7,
    }

    # AMS extended parsing
    if 0x201 <= id_int <= 0x20D:
        parse_ams_extended(id_int, frame_dict)

    # Legacy mappings (Standard Data)
    if id_int == 0x600:
        latest_data_dict[id_hex].update({
            "dc_bus_voltage": v1,
            "dc_bus_power": v2,
            "rpm": v3,
            "torque_total": v4,
            "cell_min_v": v5,
            "throttle_raw1": v6,
            "throttle_raw2": v7,
        })

    elif id_int == 0x610:
        latest_data_dict[id_hex].update({
            "motor_temp": v1,
            "pwrstg_temp": v2,
            "air_temp": v3,
            "n_actual": v4,
            "i_actual": v5,
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
            "throttle": max(0.0, min(100.0, v3)),
            "brake": max(0.0, min(100.0, v4)),
        })

    elif id_int == 0x640:
        latest_data_dict[id_hex].update({
            "current_sensor": v1,
            "cell_min_v": v2,
            "cell_max_temp": v3,
        })
    
    # Nuevos mapeos para Dinámicas (si los recibes por serial, ajusta esto)
    # Por ahora, placeholders para evitar errores en el CSV
    # if id_int == 0x650: ...

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

def create_bucket(piloto: str, circuito: str, use_influx: bool = False) -> str:
    # use_influx argument kept for UI compatibility (it now triggers Marple)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    safe_piloto = piloto.replace(" ", "_")
    safe_circuito = circuito.replace(" ", "_")
    bucket_name = f"ISC_{ts}_{safe_piloto}_{safe_circuito}"
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
                 use_influx: bool = False, # Nota: use_influx ahora activa Marple Upload
                 debug: bool = False):
    
    global new_data_flag, _last_seq, _last_seq_advance_ts, _excel_logger, DEBUG_ENABLE_DEFAULT
    
    DEBUG_ENABLE_DEFAULT = debug
    logger.setLevel(logging.DEBUG if debug else logging.INFO)
    logger.info("Recepción USB-Serial iniciada. Modo Marple Upload: %s", use_influx)

    # Initialize CSV Logger (Replaces Excel)
    _excel_logger = SerialCSVLogger(bucket_id, piloto, circuito)

    if port is None:
        port = _auto_detect_port()
    if port is None:
        raise RuntimeError("No se encontró un puerto serie RF-NANO.")

    ser = _open_serial(port, baud)

    logger.info("[CONFIG] Serial: port=%s, baud=%d", port, baud)
    logger.info("[CONFIG] Logging to: %s", _excel_logger.filename)

    counters = {"rx": 0, "decode": 0, "timeout": 0, "len": 0, "short": 0, "chk": 0, "decode_fail": 0, "test": 0}
    last_stats_t = time.time()
    last_log_t = time.time()

    _set_badge("STALE", "esperando primer frame")

    try:
        while new_data_flag != -1:
            now = time.time()

            payload, err = _read_frame(ser, counters=counters)

            if err == "timeout":
                if _last_seq is not None and (now - _last_seq_advance_ts) > _STALE_T:
                    _set_badge("STALE", "sin avance de SEQ")
                if debug and now - last_stats_t >= 2.0:
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

            parse_telemetry_data_frame(decoded)

            # Log to CSV (Approx 10Hz to avoid massive file growth, adjustable)
            if now - last_log_t >= 0.1: 
                if _excel_logger:
                    _excel_logger.log_snapshot(latest_data_dict)
                last_log_t = now

            new_data_flag = 1

            if debug and now - last_stats_t >= 2.0:
                logger.debug("[STATS] rx=%d decode=%d",
                             counters["rx"], counters["decode"])
                last_stats_t = now

    finally:
        try:
            ser.close()
        except Exception:
            pass
        
        file_path = None
        if _excel_logger:
            file_path = _excel_logger.close()
        
        logger.info("Recepción USB-Serial finalizada.")
        
        # --- SUBIDA A MARPLE ---
        if use_influx and file_path:
            logger.info("Iniciando subida a Marple Data...")
            metadata = {
                "piloto": piloto,
                "circuito": circuito,
                "type": "Real_Telemetry",
                "date": datetime.now().isoformat()
            }
            isc_marple.upload_session_csv(file_path, metadata)