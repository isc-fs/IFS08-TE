"""
ISC_RTT_demo.py — Formula Student EV Physics Simulator (v3)
Developed by Andrés Sánchez de Ágreda © 2025/2026

Accumulator: 95s6p Sony VTC6
  - Cell voltage: 3.0 V (depleted) → 4.2 V (full)
  - Cell capacity: 3.0 Ah × 6p = 18.0 Ah pack
  - Pack voltage:  285 V (depleted) → 399 V (full)
  - Max current:   80 A peak from inverter (FS rules)
  - Cell temp limit: 60 °C (safety cutoff)

v3 changes over v2:
  - Battery model corrected for 95s6p VTC6 (18 Ah, 285–399 V)
  - Peak current capped at 80 A (inverter-side FS constraint)
  - Battery temperature hard-limited to 60 °C with thermal rollback
  - GPS track simulation added to demo CSV (simulated Montmeló FS loop)
  - DemoCSVLogger headers updated to match SerialCSVLogger.HEADERS exactly,
    including GPS_MERGE_COLS columns (gps_lat_deg … gps_fix)
  - Cell-level SoC-OCV curve updated to VTC6 characteristic
  - DCDC load, internal resistance and capacity all corrected
"""

from __future__ import annotations
import csv
import math
import random
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import List, Optional

import ISC_RTT_serial as rtt

try:
    import isc_marple
    _MARPLE_OK = True
except ImportError:
    _MARPLE_OK = False


# ══════════════════════════════════════════════════════════════════════════════
#  BATTERY / DRIVETRAIN CONSTANTS  —  95s6p Sony VTC6
# ══════════════════════════════════════════════════════════════════════════════
NUM_MODULES      = 5
CELLS_PER_MODULE = 19          # cells in series per module
CELLS_SERIES     = NUM_MODULES * CELLS_PER_MODULE   # 95 cells total

# VTC6 per-cell voltage range
CELL_V_FULL      = 4.18        # V — fully charged OCV
CELL_V_DEPLETED  = 3.10        # V — usable floor (30 % SoC)
CELL_V_NOM       = 3.60        # V — nominal

# Pack voltage
PACK_V_FULL      = CELLS_SERIES * CELL_V_FULL       # ≈ 397.1 V
PACK_V_DEPLETED  = CELLS_SERIES * CELL_V_DEPLETED   # ≈ 294.5 V

# VTC6: 3.0 Ah/cell × 6p = 18.0 Ah pack
PACK_CAP_AH      = 18.0        # Ah

# Internal resistance: VTC6 ~10 mΩ/cell, 95 series → 95 × 0.010 / 6 = 0.158 Ω
PACK_R_INT       = 0.16        # Ω total pack

DCDC_POWER_W     = 350.0       # W — 24 V auxiliary load (LV system)

# Motor & drivetrain
MOTOR_MAX_RPM    = 6000
MOTOR_MAX_TORQUE = 230.0       # Nm at motor shaft
MOTOR_MAX_POWER  = 80_000      # W — peak shaft power
MOTOR_EFF        = 0.93
INV_EFF          = 0.97
GEAR_RATIO       = 3.5         # motor : wheel
WHEEL_R          = 0.250       # m

# FS rules: max 80 A from the accumulator (including any DC bus current)
INV_MAX_CURRENT  = 80.0        # A  — hard cap on pack discharge current

# Chassis & aero
CAR_MASS         = 280.0       # kg (car + driver)
DRAG_CD          = 0.90
FRONTAL_A        = 1.60        # m²
AIR_RHO          = 1.225       # kg/m³
REGEN_FRAC       = 0.0         # no regenerative braking

# Brake model
BRAKE_FORCE_MAX  = 4200.0      # N  — total peak brake force
BRAKE_TH_GAIN    = 0.000095    # °C / (N·m/s) — disc heating sensitivity

# ADC mapping  (12-bit, 0–4095)
ADC_FULL         = 4095
APPS_IDLE        = 900
APPS_MAX         = 3850
APPS2_OFFSET     = -18         # sensor B offset (APPS plausibility)
BRAKE_IDLE       = 120
BRAKE_MAX        = 3200

# Thermal — ambient and thermal time constants (s)
T_AMB            = 26.0
BAT_TEMP_LIMIT   = 60.0        # °C — hardware safety cutoff
TAU_MOTOR        = 180.0
TAU_PWRSTG       = 110.0
TAU_BOARD        = 220.0
TAU_DCDC         =  90.0
TAU_BAT          = 700.0       # large: 18 Ah pack has high thermal mass
TAU_BRAKE        =  35.0

# Per-module aging factors (capacity & resistance spread)
MOD_AGING = [1.000, 0.998, 0.995, 0.997, 0.999]

# ══════════════════════════════════════════════════════════════════════════════
#  VTC6 SoC → OCV CURVE  (linear segments fit to datasheet)
# ══════════════════════════════════════════════════════════════════════════════
# (SoC 0.0 → 1.0 maps to depleted → full)
_OCV_SOC  = [0.00, 0.10, 0.20, 0.30, 0.50, 0.70, 0.90, 1.00]
_OCV_CELL = [3.10, 3.45, 3.60, 3.68, 3.73, 3.84, 4.05, 4.18]  # V per cell

def _cell_ocv(soc: float) -> float:
    """Piecewise-linear VTC6 OCV curve. Returns cell OCV in Volts."""
    soc = max(0.0, min(1.0, soc))
    for i in range(len(_OCV_SOC) - 1):
        if soc <= _OCV_SOC[i + 1]:
            t = (soc - _OCV_SOC[i]) / (_OCV_SOC[i + 1] - _OCV_SOC[i])
            return _OCV_CELL[i] + t * (_OCV_CELL[i + 1] - _OCV_CELL[i])
    return _OCV_CELL[-1]

def _pack_ocv(soc: float) -> float:
    """Return pack OCV (V) from SoC."""
    return _cell_ocv(soc) * CELLS_SERIES


# ══════════════════════════════════════════════════════════════════════════════
#  GPS TRACK SIMULATION  (Montmeló-style FS endurance loop)
# Approximate coordinates for a realistic FS loop near Barcelona
# The track is a fictional ~800 m closed loop; GPS is advanced using
# simple bearing-based dead-reckoning at the car's current speed.
# ══════════════════════════════════════════════════════════════════════════════
# Track waypoints: (lat, lon, segment_length_m, bearing_deg)
# Based on a fictional loop near 41.5700°N, 2.2600°E (Montmeló area)
_TRACK_WPT = [
    (41.57000, 2.26000, 80,   0),    # Start / finish straight (N)
    (41.57072, 2.26000, 30,  45),    # Brake zone
    (41.57099, 2.26021, 40,  90),    # Medium right-hander
    (41.57099, 2.26057, 55,  90),    # Short straight
    (41.57099, 2.26106, 20, 135),    # Brake zone
    (41.57082, 2.26124, 35, 180),    # Tight hairpin
    (41.57051, 2.26124, 50, 180),    # Exit straight
    (41.57006, 2.26124, 45, 225),    # Fast sweeper
    (41.56974, 2.26103, 70, 270),    # Back straight
    (41.56974, 2.26040, 22, 315),    # Brake zone
    (41.56995, 2.26022, 32, 315),    # Medium left
    (41.57023, 2.26002, 60, 315),    # Straight
    (41.57050, 2.25983, 25,   0),    # Brake zone
    (41.57072, 2.25983, 38,   0),    # Tight chicane
    (41.57106, 2.25983, 65, 350),    # Return straight
    (41.57165, 2.25996, 28,  20),    # Last corner
    (41.57190, 2.26000, 48,   0),    # Final straight → start
]
_TRACK_SEG_LENS = [w[2] for w in _TRACK_WPT]
_TRACK_LEN_GPS  = sum(_TRACK_SEG_LENS)

_DEG_PER_M_LAT = 1.0 / 111_320.0              # degrees per metre (latitude)

def _gps_from_dist(dist: float) -> tuple:
    """
    Return (lat, lon, bearing_deg, sog_knots) given cumulative track distance.
    Dead-reckoning using piecewise-linear waypoints.
    """
    d   = dist % _TRACK_LEN_GPS
    acc = 0.0
    for i, (lat0, lon0, seg_len, bearing) in enumerate(_TRACK_WPT):
        if d < acc + seg_len:
            frac = (d - acc) / seg_len
            # Next waypoint (wrap around)
            lat1, lon1, _, _ = _TRACK_WPT[(i + 1) % len(_TRACK_WPT)]
            lat  = lat0 + frac * (lat1 - lat0)
            lon  = lon0 + frac * (lon1 - lon0)
            return lat, lon, float(bearing)
        acc += seg_len
    lat0, lon0, _, bearing = _TRACK_WPT[0]
    return lat0, lon0, float(bearing)


# ══════════════════════════════════════════════════════════════════════════════
#  FS ENDURANCE TRACK SEGMENTS  (physics driver)
# ══════════════════════════════════════════════════════════════════════════════
TRACK_SEGS = [
    ( 80,  95,  0 ),   # Main straight
    ( 30,  55,  0 ),   # Brake zone
    ( 40,  52,  12),   # Medium corner
    ( 55,  82,  0 ),   # Short straight
    ( 20,  40,  0 ),   # Brake zone
    ( 35,  38,  8 ),   # Tight hairpin
    ( 50,  72,  0 ),   # Exit straight
    ( 45,  62,  18),   # Fast sweeper
    ( 70,  88,  0 ),   # Back straight
    ( 22,  42,  0 ),   # Brake zone
    ( 32,  44,  10),   # Medium corner
    ( 60,  78,  0 ),   # Straight
    ( 25,  36,  0 ),   # Brake zone
    ( 38,  40,  6 ),   # Tight chicane
    ( 65,  85,  0 ),   # Return straight
    ( 28,  58,  16),   # Last corner
    ( 48,  88,  0 ),   # Finish straight
]
TRACK_LEN = sum(s[0] for s in TRACK_SEGS)

try:
    from ISC_RTT_serial import LOG_DIR
except ImportError:
    LOG_DIR = Path("logs")
    LOG_DIR.mkdir(exist_ok=True)


# ══════════════════════════════════════════════════════════════════════════════
#  CSV LOGGER  (v3 — mirrors SerialCSVLogger.HEADERS + GPS_MERGE_COLS)
# ══════════════════════════════════════════════════════════════════════════════
class DemoCSVLogger:
    """
    Column layout exactly matches SerialCSVLogger.HEADERS (ISC_RTT_serial.py)
    with GPS_MERGE_COLS and IMU fields appended so that post-race and demo sessions share
    one unified schema usable by Marple Data.
    """
    HEADERS = [
        # ── Timing ────────────────────────────────────────────────────────────
        "time", "time_elapsed_s",
        # ── Snapshot meta ──────────────────────────────────────────────────────
        "seq", "tick_ms",
        # ── Driver inputs ──────────────────────────────────────────────────────
        "start_button",
        "apps1_raw", "apps2_raw", "brake_raw",
        # ── Control ────────────────────────────────────────────────────────────
        "torque_pct", "ev_2_3", "t11_8_9", "ctrl_state",
        # ── AMS / BMS ──────────────────────────────────────────────────────────
        "ok_precharge", "ams_fsm_state",
        "v_cell_min_mV", "soc",
        "vmin_mod0", "vmin_mod1", "vmin_mod2", "vmin_mod3", "vmin_mod4",
        "vmax_mod0", "vmax_mod1", "vmax_mod2", "vmax_mod3", "vmax_mod4",
        "corriente_accu", "corriente_dcdc", "temp_dcdc",
        "tmax_mod0", "tmax_mod1", "tmax_mod2", "tmax_mod3", "tmax_mod4",
        # ── Inverter ───────────────────────────────────────────────────────────
        "inv_state", "inv_vconfig_active", "inv_error",
        "inv_dc_bus_V",
        "inv_temp_motor1", "inv_temp_pwrstg", "inv_temp_board",
        "inv_rpm", "inv_speed_actual", "inv_current_actual",
        # ── GPS (simulated — same columns added by merge_gps_into_session) ────
        "gps_lat_deg", "gps_lon_deg", "gps_sog_knots",
        "gps_cog_deg", "gps_sats", "gps_fix",
        # ── IMU (simulated) ───────────────────────────────────────────────────
        "imu_ax_g", "imu_ay_g", "imu_az_g",
        "imu_gx_dps", "imu_gy_dps", "imu_gz_dps",
        "imu_roll_deg", "imu_pitch_deg",
        # ── Pit-Wall Notes ───────────────────────────────────────────────────
        "notes",
    ]

    def __init__(self, piloto: str, circuito: str):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.piloto     = piloto
        self.circuito   = circuito
        self.start_time = time.time()
        self.filename   = LOG_DIR / f"ISC_DEMO_{ts}_{piloto}_{circuito}.csv"
        self.file       = open(self.filename, "w", newline="")
        self.writer     = csv.writer(self.file)
        self.count      = 0
        self.writer.writerow(self.HEADERS)
        print(f"[DEMO] CSV: {self.filename}")

        # Companion AMS SD log file (to test post-race merge flow)
        self.ams_filename = LOG_DIR / f"ISC_DEMO_{ts}_{piloto}_{circuito}_AMS_SD.csv"
        self.ams_file     = open(self.ams_filename, "w", newline="")
        self.ams_writer   = csv.writer(self.ams_file)
        
        # Build AMS headers matching bench-microsd branch
        self.ams_headers = [
            "tick_ms", "fsm", "mode", "ams_ok", "fault", "detail", "tsms", "dash_chg", "mod_mask",
            "pack_mV", "I_raw_mA", "I_filt_mA", "Idcdc_mA", "dcbus_V", "vmin_mV", "vmax_mV",
            "tmin_C", "tmax_C", "tavg_C"
        ]
        for m in range(5):
            for c in range(19):
                self.ams_headers.append(f"c{m}_{c}")
        for m in range(5):
            for t in range(40):
                self.ams_headers.append(f"t{m}_{t}")
                
        self.ams_writer.writerow(self.ams_headers)
        print(f"[DEMO] AMS SD CSV: {self.ams_filename}")

    def log(self, snap: dict, elapsed: float, gps: dict) -> None:
        ts   = datetime.now().isoformat()
        vmin = snap.get("vmin_modulo",     [0] * 5)
        vmax = snap.get("vmax_modulo",     [0] * 5)
        tmax = snap.get("temp_max_modulo", [0] * 5)
        row = [
            ts,                              f"{elapsed:.3f}",
            snap.get("seq",            0),   snap.get("tick_ms",         0),
            snap.get("start_button",   0),
            snap.get("apps1_raw",      0),   snap.get("apps2_raw",       0),
            snap.get("brake_raw",      0),
            snap.get("torque_pct",     0),   snap.get("ev_2_3",          0),
            snap.get("t11_8_9",        0),   snap.get("state",           0),
            snap.get("ok_precharge",   0),   snap.get("ams_fsm_state",   0),
            snap.get("v_cell_min_mV",  0),   snap.get("soc",             0),
            *(vmin[i] if i < len(vmin) else 0 for i in range(5)),
            *(vmax[i] if i < len(vmax) else 0 for i in range(5)),
            snap.get("corriente_accu", 0),   snap.get("corriente_dcdc",  0),
            snap.get("temp_dcdc",      0),
            *(tmax[i] if i < len(tmax) else 0 for i in range(5)),
            snap.get("inv_state",            0), snap.get("last_vconfig_tick", 0),
            snap.get("inv_error",            0), snap.get("inv_dc_bus_V",      0),
            snap.get("inv_temp_motor1",      0), snap.get("inv_temp_pwrstg",   0),
            snap.get("inv_temp_board",       0), snap.get("inv_rpm",           0),
            snap.get("inv_speed_actual",     0), snap.get("inv_current_actual", 0),
            # GPS columns
            f"{gps.get('lat',  0.0):.7f}",
            f"{gps.get('lon',  0.0):.7f}",
            f"{gps.get('sog',  0.0):.2f}",
            f"{gps.get('cog',  0.0):.1f}",
            gps.get("sats",  8),
            gps.get("fix",   1),
            # IMU columns
            snap.get('imu_ax_g',       0.0),
            snap.get('imu_ay_g',       0.0),
            snap.get('imu_az_g',       0.0),
            snap.get('imu_gx_dps',     0.0),
            snap.get('imu_gy_dps',     0.0),
            snap.get('imu_gz_dps',     0.0),
            snap.get('imu_roll_deg',   0.0),
            snap.get('imu_pitch_deg',  0.0),
            "",  # Empty note for telemetry snapshots
        ]
        self.writer.writerow(row)
        self.count += 1

    def write_note(self, text: str) -> None:
        """Write a custom comment/note row into the CSV file."""
        try:
            ts = datetime.now().isoformat()
            elapsed = time.time() - self.start_time
            row = [ts, f"{elapsed:.3f}"] + [""] * (len(self.HEADERS) - 3) + [text]
            self.writer.writerow(row)
            self.file.flush()
        except Exception as e:
            print(f"[DEMO] Error writing note to CSV: {e}")

        # Log to AMS SD file
        tick = snap.get("tick_ms", 0)
        fsm = snap.get("ams_fsm_state", 0)
        mode = 1 # Car mode
        ams_ok = 1
        fault = 0
        detail = 0
        tsms = 1
        dash_chg = 0
        mod_mask = 31 # binary 11111 (all online)
        pack_mv = int(snap.get("inv_dc_bus_V", 0) * 1000)
        current_ma = int(snap.get("corriente_accu", 0) * 100) # dA to mA
        dcdc_ma = int(snap.get("corriente_dcdc", 0) * 100)
        dcbus_v = snap.get("inv_dc_bus_V", 0)
        
        vmin_mv = min(vmin) if vmin else 0
        vmax_mv = max(vmax) if vmax else 0
        tmin_c = min(tmax) if tmax else 0
        tmax_c = max(tmax) if tmax else 0
        tavg_c = int(sum(tmax) / len(tmax)) if tmax else 0
        
        ams_row = [
            tick, fsm, mode, ams_ok, fault, detail, tsms, dash_chg, mod_mask,
            pack_mv, current_ma, current_ma, dcdc_ma, dcbus_v, vmin_mv, vmax_mv,
            tmin_c, tmax_c, tavg_c
        ]
        
        # Generate cell voltages c{m}_{c} with slight cell-to-cell spread (±2mV)
        for m in range(5):
            base_v = vmin[m] if m < len(vmin) else 3800
            for c in range(19):
                spread = (c % 5) - 2 # -2 to +2 mV
                ams_row.append(base_v + spread)
        
        # Generate cell temperatures t{m}_{t} (40 thermistors per module)
        for m in range(5):
            base_t = tmax[m] if m < len(tmax) else 25
            for t in range(40):
                spread = (t % 3) - 1 # -1 to +1 °C
                ams_row.append(base_t + spread)
                
        self.ams_writer.writerow(ams_row)

        if self.count % 50 == 0:
            self.file.flush()
            self.ams_file.flush()

    def close(self) -> Optional[str]:
        if self.file:
            self.file.close()
            print(f"[DEMO] CSV closed - {self.count} rows -> {self.filename}")
        if self.ams_file:
            self.ams_file.close()
            print(f"[DEMO] AMS SD CSV closed -> {self.ams_filename}")
            return str(self.filename)
        return None


# ══════════════════════════════════════════════════════════════════════════════
#  PHYSICS ENGINE
# ══════════════════════════════════════════════════════════════════════════════
class DemoDataGenerator:
    DT = 0.050   # simulation timestep (s)  — 20 Hz physics, 10 Hz CSV

    def __init__(self):
        self.running    = False
        self.thread: Optional[threading.Thread] = None
        self._lock      = threading.Lock()
        self.logger: Optional[DemoCSVLogger] = None
        self.use_marple = False

        # ── Kinematic state ────────────────────────────────────────────────
        self.t      = 0.0     # simulation time (s)
        self.seq    = 0       # snapshot sequence counter
        self.v      = 0.0     # vehicle speed (m/s)
        self.dist   = 0.0     # cumulative distance (m)
        self.accel  = 0.0     # longitudinal acceleration (m/s²)
        self.g_long = 0.0
        self.g_lat  = 0.0

        # ── Driver inputs (smooth, 0–100 %) ───────────────────────────────
        self.thr = 0.0
        self.brk = 0.0

        # ── Electrical state ───────────────────────────────────────────────
        self.soc          = 1.0         # State of Charge  0.0–1.0
        self.v_oc         = _pack_ocv(1.0)
        self.v_load       = self.v_oc
        self.pack_current = 0.0         # A  (+ = discharging, never negative)
        self.motor_I      = 0.0         # A  (inverter output current)

        # ── Per-module cell voltages (mV) ──────────────────────────────────
        # Stored as the WEAKEST / STRONGEST individual cell voltage in the module
        # (mirrors the real firmware: vmin_modX = min cell mV in module X)
        self.vmin_mod: List[float] = [
            _cell_ocv(1.0) * f * 1000.0
            for f in MOD_AGING
        ]
        self.vmax_mod: List[float] = list(self.vmin_mod)

        # ── Thermal state (°C) ─────────────────────────────────────────────
        self.T_motor  = T_AMB
        self.T_pwrstg = T_AMB
        self.T_board  = T_AMB
        self.T_dcdc   = T_AMB
        self.T_bat    = [T_AMB] * NUM_MODULES   # per module
        self.T_brake  = [T_AMB] * 4             # FL / FR / RL / RR

        # ── GPS state ─────────────────────────────────────────────────────
        self.gps_lat  = _TRACK_WPT[0][0]
        self.gps_lon  = _TRACK_WPT[0][1]
        self.gps_cog  = 0.0
        self.gps_sats = 9
        # Simulate occasional satellite count variation
        self._gps_sats_t = 0.0

        # ── EV state machine ───────────────────────────────────────────────
        # 0=OFF  1=PRECHARGE  2=READY  3=RUNNING  4=ERROR
        self.ctrl_state = 0
        self.ams_state  = 0
        self.inv_state  = 0
        self.inv_error  = 0

        # ── IMU state ─────────────────────────────────────────────────────
        self.imu_ax_g = 0.0
        self.imu_ay_g = 0.0
        self.imu_az_g = 1.0
        self.imu_gx_dps = 0.0
        self.imu_gy_dps = 0.0
        self.imu_gz_dps = 0.0
        self.imu_roll_deg = 0.0
        self.imu_pitch_deg = 0.0

    # ── Public API ─────────────────────────────────────────────────────────
    def start(self, use_marple: bool = False,
              piloto: str = "Demo", circuito: str = "Track") -> None:
        if self.running:
            return
        self._reset()
        self.use_marple = use_marple
        self.logger     = DemoCSVLogger(piloto, circuito)
        self.running    = True
        self.thread     = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        print(
            f"[DEMO] Physics engine started  "
            f"SoC=100 %  V_pack={_pack_ocv(1.0):.1f} V  "
            f"Capacity={PACK_CAP_AH:.0f} Ah  Track={TRACK_LEN:.0f} m/lap"
        )

    def stop(self) -> None:
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        if self.logger:
            fp = self.logger.close()
            # Automatically merge the simulated AMS temperature data into the session CSV!
            ams_fp = self.logger.ams_filename
            if ams_fp.exists():
                print("[DEMO] Automatically merging simulated AMS cell temperatures...")
                rtt.merge_ams_temps_into_session(Path(fp), ams_fp)
                try:
                    ams_fp.unlink()
                    print("[DEMO] Companion AMS SD CSV deleted.")
                except Exception as e:
                    print(f"[DEMO] Error deleting companion file: {e}")

            if self.use_marple and fp and _MARPLE_OK:
                print("[DEMO] Uploading to Marple...")
                isc_marple.upload_session_csv(fp, {
                    "piloto":   self.logger.piloto,
                    "circuito": self.logger.circuito,
                    "type":     "DemoSim_v3",
                    "date":     datetime.now().isoformat(),
                })
            self.logger = None
        rtt.latest_data_dict["__STATUS__"] = {
            "badge": "IDLE", "reason": "demo stopped", "ts": 0
        }

    # ── Internal reset ──────────────────────────────────────────────────────
    def _reset(self) -> None:
        self.t = self.seq = 0
        self.v = self.dist = self.accel = self.g_long = self.g_lat = 0.0
        self.thr = self.brk = 0.0
        self.soc = 1.0
        self.v_oc = self.v_load = _pack_ocv(1.0)
        self.pack_current = 0.0
        self.motor_I      = 0.0
        self.T_motor = self.T_pwrstg = self.T_board = self.T_dcdc = T_AMB
        self.T_bat   = [T_AMB] * NUM_MODULES
        self.T_brake = [T_AMB] * 4
        self.ctrl_state = self.ams_state = self.inv_state = self.inv_error = 0
        self.gps_lat, self.gps_lon, _, _ = _TRACK_WPT[0]
        self.gps_cog = 0.0
        self.vmin_mod = [
            _cell_ocv(1.0) * f * 1000.0
            for f in MOD_AGING
        ]
        self.vmax_mod = list(self.vmin_mod)
        self.imu_ax_g = 0.0
        self.imu_ay_g = 0.0
        self.imu_az_g = 1.0
        self.imu_gx_dps = 0.0
        self.imu_gy_dps = 0.0
        self.imu_gz_dps = 0.0
        self.imu_roll_deg = 0.0
        self.imu_pitch_deg = 0.0

    # ── Main physics loop ───────────────────────────────────────────────────
    def _loop(self) -> None:
        # ── Startup sequence (EV precharge state machine) ──────────────────
        self.ctrl_state = 1; self.ams_state = 1
        time.sleep(0.5)
        self.ctrl_state = 2; self.ams_state = 3; self.inv_state = 2
        time.sleep(0.3)
        self.ctrl_state = 3; self.inv_state = 3

        last_csv_t = time.time()

        while self.running:
            t0 = time.time()

            # 1. DRIVER ──────────────────────────────────────────────────────
            thr_t, brk_t = self._driver()
            # First-order input smoothing (realistic pedal lag)
            α_t = 1.0 - math.exp(-self.DT / 0.15)   # 150 ms throttle lag
            α_b = 1.0 - math.exp(-self.DT / 0.07)   # 70 ms brake lag
            self.thr += α_t * (thr_t - self.thr)
            self.brk += α_b * (brk_t - self.brk)
            self.thr  = max(0.0, min(100.0, self.thr))
            self.brk  = max(0.0, min(100.0, self.brk))

            # APPS plausibility: brake > 20 % → cut throttle (EV safety rule)
            eff_thr = self.thr if self.brk < 20.0 else max(0.0, self.thr - self.brk * 1.5)

            # 2. MOTOR & DRIVETRAIN ──────────────────────────────────────────
            motor_rpm = (self.v / (2 * math.pi * WHEEL_R)) * 60.0 * GEAR_RATIO
            motor_rpm = max(0.0, min(motor_rpm, MOTOR_MAX_RPM))

            # Torque-RPM: power-limited above base RPM
            torque_cmd = (eff_thr / 100.0) * MOTOR_MAX_TORQUE
            if motor_rpm > 50:
                p_limit   = MOTOR_MAX_POWER / (motor_rpm * math.pi / 30.0)
                torque_cmd = min(torque_cmd, p_limit)

            traction_F = torque_cmd * GEAR_RATIO / WHEEL_R
            mech_P     = traction_F * self.v if self.v > 0 else 0.0
            elec_P     = mech_P / (MOTOR_EFF * INV_EFF)

            # 3. BRAKING (mechanical only — no regen) ──────────────────────
            brk_F = 0.0
            if self.brk > 0.5 and self.v > 0.5:
                brk_F = (self.brk / 100.0) * BRAKE_FORCE_MAX

            # 4. DYNAMICS ────────────────────────────────────────────────────
            drag_F     = 0.5 * AIR_RHO * DRAG_CD * FRONTAL_A * self.v ** 2
            net_F      = traction_F - brk_F - drag_F
            self.accel = net_F / CAR_MASS
            self.v     = max(0.0, self.v + self.accel * self.DT)
            self.dist += self.v * self.DT
            self.g_long = self.accel / 9.81

            # Lateral G from current corner
            _len, _kph, radius, _prog, _rem = self._current_seg()
            if radius > 0 and self.v > 0.5:
                lat_a      = (self.v ** 2) / radius
                sign       = 1 if int(self.dist / 60) % 2 == 0 else -1
                self.g_lat = sign * min(lat_a / 9.81, 2.8)
            else:
                self.g_lat *= 0.80   # fade out

            # 5. ELECTRICAL MODEL ────────────────────────────────────────────
            # OCV from VTC6 curve
            self.v_oc  = _pack_ocv(self.soc)
            dcdc_I     = DCDC_POWER_W / max(self.v_oc, 1.0)

            # Current is always positive (discharge only — no regen)
            raw_I             = elec_P / max(self.v_oc, 1.0) + dcdc_I
            self.pack_current = max(0.0, min(INV_MAX_CURRENT, raw_I))

            self.v_load = self.v_oc - self.pack_current * PACK_R_INT
            self.v_load = max(PACK_V_DEPLETED - 5.0, self.v_load)

            # Coulomb counting (SoC)
            self.soc -= (self.pack_current * self.DT) / (PACK_CAP_AH * 3600.0)
            self.soc  = max(0.01, min(1.0, self.soc))

            # Inverter output current (phase side)
            self.motor_I = elec_P / max(self.v_load, 1.0) if self.v_load > 0 else 0.0
            self.motor_I = min(self.motor_I, INV_MAX_CURRENT)

            # 6. THERMAL MODEL ───────────────────────────────────────────────
            # Motor: 7 % of mechanical power as heat
            Q_mech      = mech_P * 0.07
            T_ss_motor  = T_AMB + Q_mech / 65.0
            self.T_motor = self._tau(self.T_motor, T_ss_motor, TAU_MOTOR)

            # Inverter power stage: 3 % of electrical power
            Q_inv       = elec_P * 0.03
            T_ss_pwrstg = T_AMB + Q_inv / 40.0
            self.T_pwrstg = self._tau(self.T_pwrstg, T_ss_pwrstg, TAU_PWRSTG)

            # Controller board: constant low-power dissipation
            self.T_board = self._tau(self.T_board, T_AMB + 12.0, TAU_BOARD)

            # DC-DC converter
            Q_dcdc      = DCDC_POWER_W * 0.06
            self.T_dcdc = self._tau(self.T_dcdc, T_AMB + Q_dcdc / 8.0, TAU_DCDC)

            # Battery modules — I²R heating, velocity-dependent airflow cooling
            # Hard limit: if any module exceeds BAT_TEMP_LIMIT, throttle back
            bat_hot = max(self.T_bat)
            for i in range(NUM_MODULES):
                I_mod    = self.pack_current               # same I in series
                Q_bat    = (I_mod ** 2) * (PACK_R_INT / NUM_MODULES) * MOD_AGING[i]
                cool_C   = 0.50 + self.v * 0.015           # W/°C convection
                # Thermal gradient from front to rear module
                T_ss_bat = T_AMB + Q_bat / cool_C + i * 0.5
                # Clamp steady-state target to safety limit
                T_ss_bat = min(T_ss_bat, BAT_TEMP_LIMIT - 2.0)
                self.T_bat[i] = self._tau(self.T_bat[i], T_ss_bat, TAU_BAT)
                # Hard clip — never simulate above limit
                self.T_bat[i] = min(self.T_bat[i], BAT_TEMP_LIMIT)

            # If any module hits the thermal limit → flag AMS error (state 4)
            if bat_hot >= BAT_TEMP_LIMIT - 0.5:
                self.ams_state = 4

            # Brake discs: all braking energy → heat (no regen)
            per_disc_F = brk_F / 4.0
            for i in range(4):
                Q_disc   = per_disc_F * self.v * BRAKE_TH_GAIN   # full energy to disc
                cool_br  = 0.06 + self.v * 0.014
                dT_cool  = (self.T_brake[i] - T_AMB) * cool_br * self.DT
                self.T_brake[i] = max(T_AMB, self.T_brake[i] + Q_disc - dT_cool)

            # 7. PER-MODULE CELL VOLTAGES (mV per individual cell) ────────────
            cell_voc = _cell_ocv(self.soc)   # V per cell at current SoC
            for i in range(NUM_MODULES):
                spread = 4.0 * (1.0 - MOD_AGING[i])    # mV imbalance from aging
                noise  = random.gauss(0.0, 0.8)         # ADC + thermal noise (mV)
                # Single cell loaded voltage in mV
                v_cell = cell_voc * MOD_AGING[i] * 1000.0
                # Subtract per-cell IR drop
                v_cell -= (self.pack_current * (PACK_R_INT / CELLS_SERIES)) * 1000.0
                self.vmin_mod[i] = max(3000.0, v_cell - spread + noise)
                self.vmax_mod[i] = min(4200.0, v_cell + spread * 0.4 + abs(noise) * 0.3)

            # 8. GPS STATE ───────────────────────────────────────────────────
            lat, lon, cog = _gps_from_dist(self.dist)
            # Add tiny GPS noise (±0.5 m)
            lat += random.gauss(0.0, 4.5e-6)
            lon += random.gauss(0.0, 6.0e-6)
            self.gps_lat  = lat
            self.gps_lon  = lon
            self.gps_cog  = cog
            # SOG in knots (1 m/s = 1.944 kn)
            sog_knots     = self.v * 1.9438
            # Simulate satellite count varying slowly between 8–12
            if self.t - self._gps_sats_t > random.uniform(30, 90):
                self.gps_sats   = random.randint(8, 12)
                self._gps_sats_t = self.t
            gps_dict = {
                "lat":  self.gps_lat,
                "lon":  self.gps_lon,
                "sog":  sog_knots,
                "cog":  self.gps_cog,
                "sats": self.gps_sats,
                "fix":  1,
            }

            # 8b. IMU SIMULATION ──────────────────────────────────────────────
            noise_g = lambda: random.gauss(0, 0.015)
            self.imu_ax_g = self.g_long + noise_g()
            self.imu_ay_g = self.g_lat + noise_g()
            self.imu_az_g = 1.0 + noise_g()
            
            target_roll = -self.g_lat * 4.0
            target_pitch = self.g_long * 2.5
            self.imu_roll_deg += 0.25 * (target_roll - self.imu_roll_deg)
            self.imu_pitch_deg += 0.25 * (target_pitch - self.imu_pitch_deg)
            
            self.imu_gx_dps = (target_roll - self.imu_roll_deg) / self.DT + random.gauss(0, 0.3)
            self.imu_gy_dps = (target_pitch - self.imu_pitch_deg) / self.DT + random.gauss(0, 0.3)
            
            if radius > 0 and self.v > 0.5:
                sign = 1 if int(self.dist / 60) % 2 == 0 else -1
                yaw_rate_rads = self.v / radius
                self.imu_gz_dps = sign * (yaw_rate_rads * 180.0 / math.pi) + random.gauss(0, 0.3)
            else:
                self.imu_gz_dps *= 0.80

            # 9. BUILD & PUBLISH SNAPSHOT ────────────────────────────────────
            snap = self._build_snapshot(motor_rpm)
            self._publish(snap, gps_dict)

            # CSV at ~10 Hz
            if time.time() - last_csv_t >= 0.10:
                with self._lock:
                    if self.logger:
                        self.logger.log(snap, self.t, gps_dict)
                last_csv_t = time.time()

            self.t   += self.DT
            self.seq  = (self.seq + 1) & 0xFFFF
            time.sleep(max(0.0, self.DT - (time.time() - t0)))

    # ── Driver model (look-ahead braking) ───────────────────────────────────
    def _driver(self) -> tuple:
        _len, target_kph, _r, _prog, dist_rem = self._current_seg()
        next_len, next_kph, _nr = self._next_seg()

        target_v      = target_kph / 3.6
        next_target_v = next_kph   / 3.6

        # Stopping distance required for next segment
        decel_a   = 9.0    # m/s²
        Δv_sq     = self.v ** 2 - next_target_v ** 2
        stop_dist = Δv_sq / (2.0 * decel_a) if Δv_sq > 0.0 else 0.0

        need_brake = stop_dist >= dist_rem and self.v > next_target_v + 1.5

        if need_brake:
            overspeed = (self.v - next_target_v) / max(self.v, 0.1)
            thr = 0.0
            brk = min(100.0, 35.0 + overspeed * 130.0)
        elif self.v < target_v - 1.0:
            err = min(1.0, (target_v - self.v) / max(target_v, 1.0))
            thr = min(100.0, 22.0 + err * 115.0)
            brk = 0.0
        elif self.v > target_v + 1.5:
            thr = 0.0
            brk = min(45.0, (self.v - target_v) * 5.0)
        else:
            thr = 18.0 + random.gauss(0.0, 2.0)   # cruise noise
            brk = 0.0

        return max(0.0, thr), max(0.0, brk)

    # ── Track segment helpers ────────────────────────────────────────────────
    def _current_seg(self) -> tuple:
        """Return (length, target_kph, radius, progress, dist_remaining)."""
        d = self.dist % TRACK_LEN
        accum = 0.0
        for (length, kph, radius) in TRACK_SEGS:
            if d < accum + length:
                prog = (d - accum) / length
                return length, kph, radius, prog, length * (1.0 - prog)
            accum += length
        l, k, r = TRACK_SEGS[0]
        return l, k, r, 0.0, l

    def _next_seg(self) -> tuple:
        """Return (length, target_kph, radius) of the next segment."""
        d = self.dist % TRACK_LEN
        accum = 0.0
        for i, (length, kph, radius) in enumerate(TRACK_SEGS):
            if d < accum + length:
                return TRACK_SEGS[(i + 1) % len(TRACK_SEGS)]
            accum += length
        return TRACK_SEGS[1]

    # ── Thermal helper (first-order exponential approach) ────────────────────
    def _tau(self, T_curr: float, T_ss: float, tau: float) -> float:
        α = 1.0 - math.exp(-self.DT / tau)
        return T_curr + α * (T_ss - T_curr)

    # ── Build snapshot dict (mirrors _decode_snapshot() output format) ───────
    def _build_snapshot(self, motor_rpm: float) -> dict:
        # APPS ADC  (12-bit, two sensors)
        apps1 = int(APPS_IDLE + (self.thr / 100.0) * (APPS_MAX - APPS_IDLE))
        apps2 = int(APPS_IDLE + (self.thr / 100.0) * (APPS_MAX - APPS_IDLE)
                    + APPS2_OFFSET + random.gauss(0.0, 2.0))
        brake_adc = int(BRAKE_IDLE + (self.brk / 100.0) * (BRAKE_MAX - BRAKE_IDLE))
        apps1     = max(0, min(ADC_FULL, apps1))
        apps2     = max(0, min(ADC_FULL, apps2))
        brake_adc = max(0, min(ADC_FULL, brake_adc))

        soc_pct   = int(self.soc * 100)

        # corriente in dA (raw int16): pack current × 10
        corriente_a = int(self.pack_current * 10)
        corriente_d = int((DCDC_POWER_W / max(self.v_load, 1.0)) * 10)

        # DC bus voltage (integer V, clamp to pack range)
        dc_bus_v = max(int(PACK_V_DEPLETED), min(int(PACK_V_FULL) + 5,
                                                   int(self.v_load)))

        return {
            "tick_ms":            int(self.t * 1000) & 0xFFFFFFFF,
            "seq":                self.seq,
            "start_button":       1,
            "apps1_raw":          apps1,
            "apps2_raw":          apps2,
            "brake_raw":          brake_adc,
            "torque_pct":         int(self.thr),
            "ev_2_3":             1,
            "t11_8_9":            1,
            "state":              self.ctrl_state,
            "ok_precharge":       1,
            "ams_fsm_state":      self.ams_state,
            "v_cell_min_mV":      int(min(self.vmin_mod)),   # weakest cell in pack
            "soc":                soc_pct,
            "vmin_modulo":        [int(v) for v in self.vmin_mod],
            "vmax_modulo":        [int(v) for v in self.vmax_mod],
            "corriente_accu":     corriente_a,
            "corriente_dcdc":     corriente_d,
            "temp_dcdc":          int(self.T_dcdc),
            "temp_max_modulo":    [int(t) for t in self.T_bat],
            "inv_state":          self.inv_state,
            "last_vconfig_tick":  1,
            "inv_error":          self.inv_error,
            "inv_dc_bus_V":       dc_bus_v,
            "inv_temp_motor1":    int(self.T_motor),
            "inv_temp_pwrstg":    int(self.T_pwrstg),
            "inv_temp_board":     int(self.T_board),
            "inv_rpm":            int(motor_rpm),
            "inv_speed_actual":   int(self.v * 3.6),   # km/h
            "inv_current_actual": int(self.motor_I),
            # Simulated IMU values in snapshot
            "imu_ax_g":           self.imu_ax_g,
            "imu_ay_g":           self.imu_ay_g,
            "imu_az_g":           self.imu_az_g,
            "imu_gx_dps":         self.imu_gx_dps,
            "imu_gy_dps":         self.imu_gy_dps,
            "imu_gz_dps":         self.imu_gz_dps,
            "imu_roll_deg":       self.imu_roll_deg,
            "imu_pitch_deg":      self.imu_pitch_deg,
        }

    # ── Publish snapshot into rtt module (same path as real radio data) ──────
    def _publish(self, snap: dict, gps: dict) -> None:
        rtt.latest_data_dict["snapshot"] = snap

        # GPS state (read by any UI component that checks gps_state — future use)
        rtt.latest_data_dict["gps_demo"] = gps

        # Badge / status
        rtt.latest_data_dict["__STATUS__"] = {
            "badge":  "LIVE",
            "reason": "demo",
            "ts":     int(time.time() * 1000),
        }

        # Backward-compat aggregate keys
        tmax = snap["temp_max_modulo"]
        vmax = snap["vmax_modulo"]
        rtt.latest_data_dict["ams_summary"] = {
            "min_cell_mv": snap["v_cell_min_mV"],
            "max_cell_mv": max(vmax) if vmax else 0,
            "stack_mv":    0,
        }
        rtt.latest_data_dict["ams_current"] = {
            "current_A": snap["corriente_accu"] / 10.0,
        }
        tmax_valid = [t for t in tmax if t > 0]
        rtt.latest_data_dict["ams_temp_summary"] = {
            "max_temp_c": max(tmax_valid) if tmax_valid else 0,
            "min_temp_c": min(tmax_valid) if tmax_valid else 0,
            "avg_temp_c": sum(tmax_valid) / len(tmax_valid) if tmax_valid else 0,
        }

        # Update rtt.ams_modules for any code that reads them directly
        for i, mod in enumerate(rtt.ams_modules):
            mod.min_cell_mv    = snap["vmin_modulo"][i]
            mod.max_cell_mv    = snap["vmax_modulo"][i]
            mod.max_temp_c     = float(snap["temp_max_modulo"][i])
            mod.last_update_ts = time.time()

        # Trigger log line in ISCmetrics
        rtt.new_data_flag = 1
        rtt.data_str = (
            f"[DEMO] SEQ={snap['seq']:5d}  "
            f"V={snap['inv_dc_bus_V']:3d} V  "
            f"SoC={snap['soc']:3d}%  "
            f"rpm={snap['inv_rpm']:5d}  "
            f"T_bat_max={max(snap['temp_max_modulo']):.0f}°C  "
            f"I={snap['corriente_accu']/10.0:.1f} A  "
            f"GPS={gps['lat']:.5f},{gps['lon']:.5f}"
        )

    # ── Compatibility: expose brake temps & G-force for legacy access ────────
    def get_brake_temps(self) -> List[float]:
        return list(self.T_brake)

    def get_g_forces(self) -> dict:
        g_tot = math.sqrt(self.g_long ** 2 + self.g_lat ** 2)
        return {"g_long": self.g_long, "g_lat": self.g_lat, "g_total": g_tot}


# ══════════════════════════════════════════════════════════════════════════════
#  MODULE-LEVEL API  (matches v1/v2 interface used by ISCmetrics)
# ══════════════════════════════════════════════════════════════════════════════
_gen: Optional[DemoDataGenerator] = None


def start_demo(use_marple: bool = False,
               piloto: str = "Demo",
               circuito: str = "Track") -> None:
    global _gen
    if _gen is None:
        _gen = DemoDataGenerator()
    _gen.start(use_marple=use_marple, piloto=piloto, circuito=circuito)


def stop_demo() -> None:
    global _gen
    if _gen:
        _gen.stop()


def get_latest_data() -> dict:
    """Returns rtt.latest_data_dict — the same dict ISCmetrics reads from."""
    return rtt.get_latest_data()


def get_ams_module_data(module_id: int):
    """Return the rtt.AMSModule object for the given module (0–4)."""
    return rtt.get_ams_module_data(module_id)


def is_demo_running() -> bool:
    return _gen.running if _gen else False


def get_brake_temps() -> List[float]:
    return _gen.get_brake_temps() if _gen else [0.0] * 4


def get_g_forces() -> dict:
    return _gen.get_g_forces() if _gen else {"g_long": 0, "g_lat": 0, "g_total": 0}