"""
Recepción por USB-Serial desde RF-NANO (forward de NRF24L01, 32B LE)

API (compatible con la UI):
  - new_data_flag, data_str, latest_data_dict
  - create_bucket(piloto, circuito, use_influx=False) -> bucket_id
  - receive_data(bucket_id, piloto, circuito, port=None, baud=115200, use_influx=False, debug=False)
  - get_latest_data(data_id=None)
  - list_serial_ports()

ROBUSTEZ:
  - Framing estricto: SOF1(0xAA) SOF2(0x55) LEN(32) PAYLOAD(32B) XOR(payload)
  - Re-sync si falta SOF y contadores de errores
  - Comprobación de longitud/timeout y checksum XOR
  - Detección de patrón TEST (A0..BF y rampas consecutivas) ⇒ ignora
  - Chequeo monotónico de SEQ (uint16) y badge LIVE/STALE/BAD exportado a la UI:
      latest_data_dict["__STATUS__"] = {"badge": "...", "reason": "...", "ts": epoch_ms}
  - Excel por sesión en ./logs — **una sola hoja** "telemetry" con columnas semánticas y derivadas

NOVEDADES:
  - Mapeo correcto de 0x610 (temps/corrientes de inversor)
  - 0x640: Accumulator summary (v3 = DS18B20 t_max)
  - 0x645 (opcional): Detalle DS18B20 (t1..t4, avg, max, count)
  - Columnas derivadas:
      motor_rpm (prefer 0x610.v4 → 0x600.v3)
      motor_ang_accel_rad_s2 (d/dt RPM)
      veh_accel_mps2 (si defines GEAR_RATIO, FINAL_DRIVE, WHEEL_RADIUS_M)
"""

from __future__ import annotations
import os
import time
import struct
import logging
from datetime import datetime
from typing import Optional, Dict, Any

import serial
import serial.tools.list_ports

# Excel logging
import pandas as pd
import numpy as np

# ================== CONFIG RF (informativo, para log) ==================
RF_EXPECTED = {
    "PIPE_ADDR": "0xE7E7E7E7E7",
    "CHANNEL":   76,          # 0x4C
    "PAYLOAD":   32,          # bytes
    "DATA_RATE": "1Mbps",
    "AUTO_ACK":  False,       # NO-ACK
    "CRC":       "CRC_16",
    "PA":        "PA_MAX",
}

# ================== CONFIG DERIVADOS (vehículo) ==================
# Si quieres a_mps2, rellena estos tres valores (ejemplo: GEAR_RATIO=3.5, FINAL_DRIVE=3.9, WHEEL_RADIUS_M=0.285)
GEAR_RATIO: Optional[float]     = None
FINAL_DRIVE: Optional[float]    = None
WHEEL_RADIUS_M: Optional[float] = None

# ================== CONFIG Influx (OPCIONAL) ==================
INFLUX_CONFIG = {
    "url":   "http://localhost:8086",
    "token": "TOKEN",
    "org":   "TORG",
}
INFLUX_ENABLE_DEFAULT = False   # <-- por defecto DESACTIVADO

_client = None
_influx_ok = False

def _init_influx():
    """Inicializa el cliente de Influx de forma perezosa."""
    global _client, _influx_ok
    if _client is not None:
        return
    try:
        from influxdb_client import InfluxDBClient  # import tardío
        _client = InfluxDBClient(**INFLUX_CONFIG)
        _influx_ok = True
        logging.getLogger("ISC_RTT_USB").info("InfluxDB inicializado: %s", INFLUX_CONFIG["url"])
    except Exception as e:
        _client = None
        _influx_ok = False
        logging.getLogger("ISC_RTT_USB").warning("Influx deshabilitado: %s", e)

def _get_write_api():
    if not _influx_ok or _client is None:
        return None
    try:
        from influxdb_client.client.write_api import SYNCHRONOUS
        return _client.write_api(write_options=SYNCHRONOUS)
    except Exception as e:
        logging.getLogger("ISC_RTT_USB").warning("No se pudo crear write_api: %s", e)
        return None

# ================== MARCO SERIAL ==================
SOF1 = 0xAA
SOF2 = 0x55
PAYLOAD_LEN = 32

DEFAULT_BAUD = 115200
DEFAULT_PORT = None  # autodetect si None

# ================== ESTADO PARA LA UI ==================
data_str = ""
new_data_flag = 0
latest_data_dict: dict = {}

# Estado de enlace / badges para UI
_status = {
    "badge": "STALE",         # LIVE | STALE | BAD
    "reason": "inicio",
    "ts": 0,                  # epoch ms del último cambio
}
_last_seq = None              # último SEQ visto (uint16)
_last_seq_advance_ts = 0.0    # time.time() del último avance de SEQ
_STALE_T = 0.20               # s sin avance de SEQ -> STALE

logger = logging.getLogger("ISC_RTT_USB")
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

# ================== UTILIDADES ==================
def _dump_hex(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)

def list_serial_ports():
    """Devuelve lista de (device, description) para poblar combo de la UI."""
    out = []
    for p in serial.tools.list_ports.comports():
        out.append((p.device, p.description))
    return out

def _auto_detect_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = (p.description or "").upper()
        hwid = (p.hwid or "").upper()
        if "CH340" in desc or "USB-SERIAL" in desc or "CP210" in desc or "CH340" in hwid or "CP210" in hwid:
            return p.device
    return ports[0].device if ports else None

def _open_serial(port, baud):
    ser = serial.Serial(port=port, baudrate=baud, timeout=0.2)
    # Muchos Nanos reinician al abrir DTR → espera un poco
    time.sleep(1.5)
    ser.reset_input_buffer()
    return ser

def _set_badge(badge: str, reason: str):
    """Actualiza badge de estado y lo exporta para la UI."""
    global _status
    _status = {
        "badge": badge,
        "reason": reason,
        "ts": int(time.time() * 1000),
    }
    latest_data_dict["__STATUS__"] = _status

def _mod16_diff(curr: int, prev: int) -> int:
    """Diferencia forward en contador uint16 (0..65535)."""
    return (curr - prev) & 0xFFFF

TEST_PATTERN = bytes(range(0xA0, 0xA0 + PAYLOAD_LEN))  # A0..BF

def _is_consecutive_ramp(payload: bytes) -> bool:
    """True si payload[i] == payload[i-1] + 1 (mod 256) para toda la trama."""
    if len(payload) != PAYLOAD_LEN:
        return False
    return all(((payload[i] - payload[i-1]) & 0xFF) == 1 for i in range(1, PAYLOAD_LEN))

def is_test_payload(payload: bytes) -> bool:
    """Considera TEST si es exactamente A0..BF o si es una rampa consecutiva."""
    return payload == TEST_PATTERN or _is_consecutive_ramp(payload)

def _read_frame(ser, counters=None):
    """
    Frame: SOF1(0xAA) SOF2(0x55) LEN(32) PAYLOAD(32B) XOR(payload)
    Devuelve (payload, err) donde:
      - payload: bytes (32) o None
      - err: None | 'timeout' | 'len' | 'short' | 'chk'
    Re-sync simple: consume hasta ver AA 55.
    """
    # Escanear hasta AA
    b = ser.read(1)
    if not b:
        if counters is not None: counters["timeout"] += 1
        return None, "timeout"
    if b[0] != SOF1:
        return None, None  # seguir escaneando

    # Esperar 55
    b2 = ser.read(1)
    if not b2:
        if counters is not None: counters["timeout"] += 1
        return None, "timeout"
    if b2[0] != SOF2:
        return None, None  # desalineado, volver a escanear

    # LEN
    ln = ser.read(1)
    if not ln:
        if counters is not None: counters["timeout"] += 1
        return None, "timeout"
    if ln[0] != PAYLOAD_LEN:
        if counters is not None: counters["len"] += 1
        # consumir presunto payload + chk para re-sync rápido
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
    """
    Intenta decodificar como TelFrame (2x uint16 + 7 floats).
    Si falla, intenta legacy (8 floats).
    Devuelve dict con: id, seq (o None), v1..v7, raw_floats, fmt
    """
    assert len(payload) == PAYLOAD_LEN

    # Nuevo: TelFrame
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

    # Legacy: 8 floats
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

def parse_telemetry_data_frame(frame_dict: dict):
    """
    Interpreta el frame, actualiza latest_data_dict + data_str,
    y devuelve un Influx Point o None (si Influx no se usa/está caído).
    """
    global data_str, latest_data_dict

    if not frame_dict:
        return None

    id_int = frame_dict["id"]
    id_hex = _id_hex(id_int)
    seq    = frame_dict["seq"]
    v1 = frame_dict["v1"]; v2 = frame_dict["v2"]; v3 = frame_dict["v3"]
    v4 = frame_dict["v4"]; v5 = frame_dict["v5"]; v6 = frame_dict["v6"]; v7 = frame_dict["v7"]

    # Línea para la consola de la UI
    if seq is not None:
        data_str = (
            f"[RX] ID={id_hex} count={seq} state={_status.get('badge','?')}\n"
            f"[RX] FLOATS: {v1:.2f}, {v2:.2f}, {v3:.2f}, {v4:.2f}, {v5:.2f}, {v6:.2f}, {v7:.2f},"
        )
    else:
        data_str = (
            f"[RX] ID={id_hex} state={_status.get('badge','?')}\n"
            f"[RX] FLOATS: {v1:.2f}, {v2:.2f}, {v3:.2f}, {v4:.2f}, {v5:.2f}, {v6:.2f}, {v7:.2f},"
        )

    # Base “raw”
    latest_data_dict[id_hex] = {
        "id": id_int, "seq": seq,
        "v1": v1, "v2": v2, "v3": v3, "v4": v4, "v5": v5, "v6": v6, "v7": v7,
    }

    # ------- Mapeos semánticos -------
    if id_int == 0x600:
        latest_data_dict[id_hex].update({
            "dc_bus_voltage": v1,   # V
            "dc_bus_power":   v2,   # W (si mapeado)
            "rpm":            v3,
            "torque_total":   v4,
            "cell_min_v":     v5,
            "throttle_raw1":  v6,
            "throttle_raw2":  v7,
        })

    elif id_int == 0x610:
        latest_data_dict[id_hex].update({
            "motor_temp":   v1,   # °C
            "pwrstg_temp":  v2,   # °C (IGBT)
            "air_temp":     v3,   # °C
            "n_actual":     v4,   # rpm (o unidades del inversor)
            "i_actual":     v5,   # A
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
            "current_sensor": v1,  # A si lo mandas
            "cell_min_v":     v2,  # V o mV según escale tu TX
            "cell_max_temp":  v3,  # °C (DS18B20 max)
        })

    elif id_int == 0x645:
        latest_data_dict[id_hex].update({
            "ds_t1": v1,
            "ds_t2": v2,
            "ds_t3": v3,
            "ds_t4": v4,
            "ds_avg": v5,
            "ds_max": v6,
            "ds_count": v7,
        })

    elif id_int == 0x680:
        latest_data_dict[id_hex].update({
            "status": v1,
            "errors": v2,
        })

    # Intentar construir un Point (solo si luego se va a usar)
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

# ================== METADATOS / BUCKET (OPCIONAL) ==================
def write_session_metadata(write_api, bucket_id, piloto, circuito):
    """Escribe metadatos de sesión si hay write_api; si no, no hace nada."""
    if write_api is None:
        return
    try:
        from influxdb_client import Point
        p = (
            Point("session_meta")
            .tag("piloto", piloto)
            .tag("circuito", circuito)
            .field("start_ts", int(time.time()))
        )
        write_api.write(bucket=bucket_id, record=p)
    except Exception as e:
        logger.warning("No se pudo escribir metadatos de sesión: %s", e)

def _ensure_bucket(bucket_name: str) -> str:
    """Crea el bucket si es posible; si no, devuelve el nombre sin fallar."""
    if not _influx_ok or _client is None:
        return bucket_name
    try:
        ba = _client.buckets_api()
        existing = [b for b in (ba.find_buckets().buckets or []) if b.name == bucket_name]
        if not existing:
            ba.create_bucket(bucket_name=bucket_name, org=INFLUX_CONFIG["org"])
        return bucket_name
    except Exception as e:
        logger.warning("No se pudo asegurar bucket '%s': %s", bucket_name, e)
        return bucket_name

def create_bucket(piloto: str, circuito: str, use_influx: bool = INFLUX_ENABLE_DEFAULT) -> str:
    """
    Devuelve un nombre de bucket único por sesión. Si use_influx=True,
    intenta comprobar/crear el bucket, pero nunca lanza excepción.
    """
    global INFLUX_ENABLE_DEFAULT
    INFLUX_ENABLE_DEFAULT = bool(use_influx)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    safe_piloto = piloto.replace(" ", "_")
    safe_circuito = circuito.replace(" ", "_")
    bucket_name = f"ISC_{ts}_{safe_piloto}_{safe_circuito}"

    if use_influx:
        _init_influx()
        bucket_name = _ensure_bucket(bucket_name)

    return bucket_name

def get_latest_data(data_id: str = None):
    if data_id:
        return latest_data_dict.get(data_id, {})
    return latest_data_dict.copy()

# ================== EXCEL LOGGING (UNA SOLA HOJA) ==================
_CLAMP_TEMP_MIN = -40.0
_CLAMP_TEMP_MAX = 200.0

def _clamp_temp(x: Optional[float]) -> Optional[float]:
    if x is None or np.isnan(x):
        return x
    if x < _CLAMP_TEMP_MIN or x > _CLAMP_TEMP_MAX:
        return np.nan  # inválido → NaN
    return x

class ExcelSessionLogger:
    """
    Excel de sesión en ./logs con UNA hoja "telemetry".
    - Guarda una fila por frame válido (no-TEST)
    - Añade columnas semánticas y derivadas
    - Calcula motor_ang_accel_rad_s2 en línea (d/dt del mejor RPM)
    """
    def __init__(self, piloto: str, circuito: str):
        os.makedirs("logs", exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_piloto = piloto.replace(" ", "_")
        safe_circuito = circuito.replace(" ", "_")
        self.path = os.path.join("logs", f"ISC_{ts}_{safe_piloto}_{safe_circuito}.xlsx")

        # Estado para derivados (RPM y a_mps2)
        self.last_rpm_ts: Optional[float] = None
        self.last_rpm: Optional[float]    = None

        # Predefinir columnas del Excel (anchas/semánticas)
        self.columns = [
            # básicos
            "timestamp", "epoch_ms", "badge", "badge_reason",
            "id_hex", "seq",
            # 0x600
            "dc_bus_voltage_V", "dc_bus_power_W", "rpm_600", "torque_total", "cell_min_v_600",
            "throttle_raw1", "throttle_raw2",
            # 0x610
            "motor_temp_C", "pwrstg_temp_C", "air_temp_C", "rpm_610", "i_actual_A",
            # 0x620
            "s1_raw", "s2_raw", "brake_raw", "precharge_button", "start_button",
            # 0x630
            "torque_req", "torque_est", "throttle_pct", "brake_pct",
            # 0x640
            "hv_current_A", "cell_min_v_640", "accu_ds_tmax_C",
            # 0x645
            "ds_t1_C", "ds_t2_C", "ds_t3_C", "ds_t4_C", "ds_avg_C", "ds_max_C", "ds_count",
            # 0x680
            "status_word", "error_word",
            # derivados (comunes)
            "motor_rpm", "motor_ang_accel_rad_s2", "veh_accel_mps2",
            # crudos (por depuración)
            "v1","v2","v3","v4","v5","v6","v7",
        ]

        # Crea fichero con cabeceras
        df = pd.DataFrame(columns=self.columns)
        with pd.ExcelWriter(self.path, engine="openpyxl") as writer:
            df.to_excel(writer, index=False, sheet_name="telemetry")

    def _veh_acc_from_alpha(self, ang_acc_rad_s2: Optional[float]) -> Optional[float]:
        if ang_acc_rad_s2 is None or np.isnan(ang_acc_rad_s2):
            return np.nan
        if not (GEAR_RATIO and FINAL_DRIVE and WHEEL_RADIUS_M):
            return np.nan
        # a_vehicle ≈ α_motor * (R / (G*F))
        try:
            return float(ang_acc_rad_s2) * (WHEEL_RADIUS_M / (GEAR_RATIO * FINAL_DRIVE))
        except Exception:
            return np.nan

    def _best_rpm_from_row(self, row_map: Dict[str, Any]) -> Optional[float]:
        """Prefiere rpm_610; si NaN, usa rpm_600."""
        rpm610 = row_map.get("rpm_610")
        rpm600 = row_map.get("rpm_600")
        if rpm610 is not None and not np.isnan(rpm610):
            return float(rpm610)
        if rpm600 is not None and not np.isnan(rpm600):
            return float(rpm600)
        return np.nan

    def _row_from_frame(self, id_hex: str, seq, v1, v2, v3, v4, v5, v6, v7) -> Dict[str, Any]:
        now = time.time()
        ts_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        epoch_ms = int(now * 1000)

        # Base map con NaN en todo
        r = {c: (np.nan) for c in self.columns}
        # Básicos
        r["timestamp"]    = ts_str
        r["epoch_ms"]     = epoch_ms
        r["badge"]        = _status.get("badge")
        r["badge_reason"] = _status.get("reason")
        r["id_hex"]       = id_hex
        r["seq"]          = (int(seq) if seq is not None else -1)

        # Guardar crudos
        r["v1"]=v1; r["v2"]=v2; r["v3"]=v3; r["v4"]=v4; r["v5"]=v5; r["v6"]=v6; r["v7"]=v7

        # Mapeo por ID
        if id_hex == "0x600":
            r["dc_bus_voltage_V"] = float(v1)
            r["dc_bus_power_W"]   = float(v2)
            r["rpm_600"]          = float(v3)
            r["torque_total"]     = float(v4)
            r["cell_min_v_600"]   = float(v5)
            r["throttle_raw1"]    = float(v6)
            r["throttle_raw2"]    = float(v7)

        elif id_hex == "0x610":
            r["motor_temp_C"]  = _clamp_temp(float(v1))
            r["pwrstg_temp_C"] = _clamp_temp(float(v2))
            r["air_temp_C"]    = _clamp_temp(float(v3))
            r["rpm_610"]       = float(v4)
            r["i_actual_A"]    = float(v5)

        elif id_hex == "0x620":
            r["s1_raw"]            = float(v1)
            r["s2_raw"]            = float(v2)
            r["brake_raw"]         = float(v3)
            r["precharge_button"]  = float(v4)
            r["start_button"]      = float(v5)

        elif id_hex == "0x630":
            r["torque_req"]   = float(v1)
            r["torque_est"]   = float(v2)
            r["throttle_pct"] = float(np.clip(v3, 0.0, 100.0))
            r["brake_pct"]    = float(np.clip(v4, 0.0, 100.0))

        elif id_hex == "0x640":
            r["hv_current_A"]   = float(v1)
            r["cell_min_v_640"] = float(v2)
            r["accu_ds_tmax_C"] = _clamp_temp(float(v3))

        elif id_hex == "0x645":
            r["ds_t1_C"]   = _clamp_temp(float(v1))
            r["ds_t2_C"]   = _clamp_temp(float(v2))
            r["ds_t3_C"]   = _clamp_temp(float(v3))
            r["ds_t4_C"]   = _clamp_temp(float(v4))
            r["ds_avg_C"]  = _clamp_temp(float(v5))
            r["ds_max_C"]  = _clamp_temp(float(v6))
            r["ds_count"]  = float(v7)

        elif id_hex == "0x680":
            r["status_word"] = float(v1)
            r["error_word"]  = float(v2)

        # Derivados — elegir mejor RPM y calcular α y a
        rpm_best = self._best_rpm_from_row(r)
        if rpm_best is not None and not np.isnan(rpm_best):
            r["motor_rpm"] = rpm_best
            if self.last_rpm is not None and self.last_rpm_ts is not None:
                dt = max(0.0, now - self.last_rpm_ts)
                if dt > 0.0:
                    rpm_s = (rpm_best - self.last_rpm) / dt
                    ang_acc = (rpm_s * 2.0 * np.pi) / 60.0  # rad/s²
                    r["motor_ang_accel_rad_s2"] = float(ang_acc)
                    r["veh_accel_mps2"] = float(self._veh_acc_from_alpha(ang_acc))
        # actualizar memoria de RPM sólo si este frame aporta rpm_610 o rpm_600
        if id_hex in ("0x610","0x600") and r.get("motor_rpm", np.nan) is not np.nan:
            self.last_rpm = float(r["motor_rpm"]) if not np.isnan(r["motor_rpm"]) else self.last_rpm
            self.last_rpm_ts = now

        return r

    def append(self, id_hex: str, seq, v1, v2, v3, v4, v5, v6, v7):
        try:
            row_map = self._row_from_frame(id_hex, seq, v1, v2, v3, v4, v5, v6, v7)
            # Leemos hoja, añadimos y sobrescribimos (simple y robusto)
            existing = pd.read_excel(self.path, sheet_name="telemetry")
            newdf = pd.concat([existing, pd.DataFrame([row_map], columns=self.columns)], ignore_index=True)
            with pd.ExcelWriter(self.path, engine="openpyxl", mode="w") as writer:
                newdf.to_excel(writer, index=False, sheet_name="telemetry")
        except Exception as e:
            logger.warning("No se pudo escribir Excel '%s': %s", self.path, e)

# ================== RECEPCIÓN PRINCIPAL ==================
def receive_data(bucket_id: str,
                 piloto: str,
                 circuito: str,
                 port: str = DEFAULT_PORT,
                 baud: int = DEFAULT_BAUD,
                 use_influx: bool = INFLUX_ENABLE_DEFAULT,
                 debug: bool = False):
    """
    Bucle principal:
      - Log inicial estilo Arduino ([CONFIG] + entorno)
      - Emite "Radio Checking" cada 500 ms
      - Lee frames, valida, descarta TEST, decodifica, actualiza latest_data_dict + data_str + new_data_flag
      - (Opcional) escribe en Influx
      - Escribe **un único Excel** (hoja 'telemetry') con columnas semánticas y derivadas
      - Muestra [STATS] cada 2 s (nivel DEBUG)
    """
    global new_data_flag, _last_seq, _last_seq_advance_ts

    # Nivel de log
    logger.setLevel(logging.DEBUG if debug else logging.INFO)
    logger.info("Recepción USB-Serial iniciada")

    # Influx opcional
    write_api = None
    if use_influx:
        _init_influx()
        write_api = _get_write_api()
    if write_api:
        write_session_metadata(write_api, bucket_id, piloto, circuito)
    else:
        logger.info("Influx deshabilitado: sólo UI + Excel")

    # Excel
    xlogger = ExcelSessionLogger(piloto, circuito)
    logger.info("[CONFIG] Excel path: %s", xlogger.path)

    # Puerto serie (abrir UNA vez)
    if port is None:
        port = _auto_detect_port()
    if port is None:
        raise RuntimeError("No se encontró un puerto serie RF-NANO.")

    ser = _open_serial(port, baud)

    # ===== Mensajes estilo Arduino de configuración esperada =====
    logger.info("[CONFIG] RF24 (esperada por RX):")
    logger.info("        AddressWidth=5, PipeAddr=%s", RF_EXPECTED["PIPE_ADDR"])
    logger.info("        Channel=%d (0x%02X), Payload=%d", RF_EXPECTED["CHANNEL"], RF_EXPECTED["CHANNEL"], RF_EXPECTED["PAYLOAD"])
    logger.info("        DataRate=%s, AutoAck=%s, CRC=%s, PA=%s",
                RF_EXPECTED["DATA_RATE"], "ON" if RF_EXPECTED["AUTO_ACK"] else "OFF",
                RF_EXPECTED["CRC"], RF_EXPECTED["PA"])
    logger.info("[CONFIG] Serial: port=%s, baud=%d, frame=AA 55 %02X <32B> XOR", port, baud, PAYLOAD_LEN)

    # ===== Estadísticas/contadores =====
    counters = {"rx": 0, "decode": 0, "timeout": 0, "len": 0, "short": 0, "chk": 0, "decode_fail": 0, "test": 0}
    last_check_t = time.time()
    last_stats_t = last_check_t
    first_frame_seen = False

    _set_badge("STALE", "esperando primer frame")

    logger.info("Leyendo de %s @ %d bps", port, baud)

    try:
        while new_data_flag != -1:
            now = time.time()
            # "Radio Checking" cada 500 ms
            if now - last_check_t >= 0.5:
                logger.info("Radio Checking")
                last_check_t = now

            payload, err = _read_frame(ser, counters=counters)

            if err == "timeout":
                # sin datos por timeout; comprobar STALE por inactividad de SEQ
                if _last_seq is not None and (now - _last_seq_advance_ts) > _STALE_T:
                    _set_badge("STALE", "sin avance de SEQ")
                # stats periódicas
                if now - last_stats_t >= 2.0:
                    logger.debug("[STATS] rx=%d decode=%d chk=%d len=%d short=%d timeout=%d decode_fail=%d test=%d",
                                 counters["rx"], counters["decode"], counters["chk"],
                                 counters["len"], counters["short"], counters["timeout"],
                                 counters["decode_fail"], counters["test"])
                    last_stats_t = now
                continue
            elif err == "len":
                logger.debug("[ERR] LEN inválida distinta de 32")
                _set_badge("BAD", "len inválida")
                continue
            elif err == "short":
                logger.debug("[ERR] Payload corto")
                _set_badge("BAD", "payload corto")
                continue
            elif err == "chk":
                logger.debug("[ERR] Checksum XOR no coincide")
                _set_badge("BAD", "checksum XOR")
                continue

            if payload is None:
                # byte basura: seguimos
                continue

            # Detección de patrón de prueba (no afecta estado global)
            if is_test_payload(payload):
                counters["test"] += 1
                logger.debug("[TEST] Patrón TEST (rampa o A0..BF) ignorado para este frame.")
                continue  # no se registra ni se emite a la UI

            # Tenemos un frame válido y no-TEST
            counters["rx"] += 1
            logger.debug("[RX] HEX: %s", _dump_hex(payload))

            decoded = _decode_payload(payload)
            if not decoded:
                counters["decode_fail"] += 1
                _set_badge("BAD", "decode_fail")
                logger.debug("[ERR] decode_fail (ni TelFrame ni legacy)")
                continue

            counters["decode"] += 1

            # Chequeo monotónico de secuencia (solo si TelFrame)
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
                logger.debug("[RX] ID=%s SEQ=%d (%s)", _id_hex(decoded["id"]), seq, _status["badge"])
            else:
                # Legacy sin SEQ
                _set_badge("LIVE", "legacy sin SEQ")
                logger.debug("[RX] ID=%s (legacy)", _id_hex(decoded["id"]))

            # Parse lógico (y Point si la lib está instalada)
            pt = parse_telemetry_data_frame(decoded)

            # Además del data_str (para la UI), deja el detalle de floats en DEBUG
            fline = ", ".join(f"{x:.2f}" for x in (
                decoded["v1"], decoded["v2"], decoded["v3"], decoded["v4"],
                decoded["v5"], decoded["v6"], decoded["v7"]))
            logger.debug("[RX] FLOATS: %s", fline)

            # Excel (solo frames válidos y no-TEST) — UNA HOJA
            id_hex = _id_hex(decoded["id"])
            try:
                xlogger.append(
                    id_hex,
                    decoded.get("seq"),
                    decoded["v1"], decoded["v2"], decoded["v3"], decoded["v4"],
                    decoded["v5"], decoded["v6"], decoded["v7"]
                )
            except Exception as e:
                logger.warning("Excel append falló: %s", e)

            # Influx opcional
            if write_api and pt:
                try:
                    pt = pt.tag("piloto", piloto).tag("circuito", circuito)
                    write_api.write(bucket=bucket_id, record=pt)
                except Exception as e:
                    logger.warning("Error escribiendo en Influx: %s", e)

            # Notifica a la UI
            new_data_flag = 1

            if not first_frame_seen:
                first_frame_seen = True
                logger.info("[SYNC] Primer frame recibido correctamente.")

            # stats periódicas cada 2s
            if now - last_stats_t >= 2.0:
                logger.debug("[STATS] rx=%d decode=%d chk=%d len=%d short=%d timeout=%d decode_fail=%d test=%d",
                             counters["rx"], counters["decode"], counters["chk"],
                             counters["len"], counters["short"], counters["timeout"],
                             counters["decode_fail"], counters["test"])
                last_stats_t = now

    finally:
        try:
            ser.close()
        except Exception:
            pass
        logger.info("Recepción USB-Serial finalizada.")
