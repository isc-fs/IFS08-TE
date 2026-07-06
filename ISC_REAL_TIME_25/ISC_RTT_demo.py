"""
ISC_RTT_demo.py — Formula Student EV Physics Simulator (v2)
Developed by Andrés Sánchez de Ágreda © 2025/2026

Improvements over v1:
  - Output in snapshot format, injected directly into rtt.latest_data_dict
    so ISCmetrics reads demo data through the exact same path as real radio.
  - Look-ahead braking driver model (computes stopping distance to next corner).
  - SoC-based OCV: 400 V → 360 V over session, Coulomb counting.
  - Per-module cell voltage spread with per-module aging coefficients.
  - Regen braking: 30 % of brake energy recovered, fed back to battery.
  - APPS plausibility check: throttle cut when brake > 20 % (EV safety rule).
  - Dual APPS ADC sensors with realistic offset.
  - Full thermal network (first-order): motor, inverter, board, DC-DC,
    5 battery modules, 4 brake discs (velocity-dependent air cooling).
  - CSV logger uses v2 SerialCSVLogger headers (Marple-compatible).
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
#  BATTERY / DRIVETRAIN CONSTANTS  (ISC FS EV 2025/2026)
# ══════════════════════════════════════════════════════════════════════════════
NUM_MODULES      = 5
CELLS_PER_MODULE = 19
CELLS_SERIES     = NUM_MODULES * CELLS_PER_MODULE          # 95 cells in series

# Usable voltage range mapped over demo session (SoC 1.0 → 0.0)
PACK_V_FULL      = 400.0   # V  — SoC = 1.0
PACK_V_DEPLETED  = 360.0   # V  — SoC = 0.0 (demo floor)
PACK_R_INT       = 0.10    # Ω  — total pack internal resistance

PACK_CAP_AH      = 7.5     # Ah — nominal capacity
DCDC_POWER_W     = 300.0   # W  — 12/24 V auxiliary load

# Motor & drivetrain
MOTOR_MAX_RPM    = 6000
MOTOR_MAX_TORQUE = 230.0   # Nm at motor shaft
MOTOR_MAX_POWER  = 80_000  # W  — peak
MOTOR_EFF        = 0.92
INV_EFF          = 0.97
GEAR_RATIO       = 3.5     # motor : wheel
WHEEL_R          = 0.250   # m

# Chassis & aero
CAR_MASS         = 280.0   # kg (car + driver)
DRAG_CD          = 0.90
FRONTAL_A        = 1.60    # m²
AIR_RHO          = 1.225   # kg/m³
REGEN_FRAC       = 0.30    # fraction of braking recovered by motor
MAX_REGEN_KW     = 20.0    # kW — inverter regen limit

# Brake model
BRAKE_FORCE_MAX  = 4200.0  # N  — total peak brake force
BRAKE_TH_GAIN    = 0.00010 # °C / (N·m/s) — disc heating sensitivity

# ADC mapping  (12-bit, 0–4095)
ADC_FULL         = 4095
APPS_IDLE        = 900
APPS_MAX         = 3850
APPS2_OFFSET     = -18     # sensor B has a slight offset (APPS plausibility)
BRAKE_IDLE       = 120
BRAKE_MAX        = 3200

# Thermal — ambient and thermal time constants (seconds)
T_AMB            = 25.0
TAU_MOTOR        = 180.0
TAU_PWRSTG       = 110.0
TAU_BOARD        = 220.0
TAU_DCDC         =  90.0
TAU_BAT          = 600.0
TAU_BRAKE        =  35.0

# Per-module aging factors (capacity & resistance spread)
MOD_AGING = [1.000, 0.998, 0.995, 0.997, 0.999]

# FS Endurance/Autocross track: (length_m, target_kph, corner_radius_m)
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

LOG_DIR = Path("logs")
LOG_DIR.mkdir(exist_ok=True)


# ══════════════════════════════════════════════════════════════════════════════
#  CSV LOGGER  (v2 headers — Marple-compatible)
# ══════════════════════════════════════════════════════════════════════════════
class DemoCSVLogger:
    HEADERS = [
        "time", "time_elapsed_s",
        "seq", "tick_ms",
        "start_button",
        "apps1_raw", "apps2_raw", "brake_raw",
        "torque_pct", "ev_2_3", "t11_8_9", "ctrl_state",
        "ok_precharge", "ams_fsm_state",
        "v_cell_min_mV", "soc",
        "vmin_mod0", "vmin_mod1", "vmin_mod2", "vmin_mod3", "vmin_mod4",
        "vmax_mod0", "vmax_mod1", "vmax_mod2", "vmax_mod3", "vmax_mod4",
        "corriente_accu", "corriente_dcdc", "temp_dcdc",
        "tmax_mod0", "tmax_mod1", "tmax_mod2", "tmax_mod3", "tmax_mod4",
        "inv_state", "inv_vconfig_active", "inv_error",
        "inv_dc_bus_V",
        "inv_temp_motor1", "inv_temp_pwrstg", "inv_temp_board",
        "inv_rpm", "inv_speed_actual", "inv_current_actual",
    ]

    def __init__(self, piloto: str, circuito: str):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.piloto   = piloto
        self.circuito = circuito
        self.filename = LOG_DIR / f"ISC_DEMO_{ts}_{piloto}_{circuito}.csv"
        self.file     = open(self.filename, "w", newline="")
        self.writer   = csv.writer(self.file)
        self.count    = 0
        self.writer.writerow(self.HEADERS)
        print(f"[DEMO] CSV: {self.filename}")

    def log(self, snap: dict, elapsed: float) -> None:
        ts   = datetime.now().isoformat()
        vmin = snap.get("vmin_modulo", [0] * 5)
        vmax = snap.get("vmax_modulo", [0] * 5)
        tmax = snap.get("temp_max_modulo", [0] * 5)
        row = [
            ts, f"{elapsed:.3f}",
            snap.get("seq", 0),          snap.get("tick_ms", 0),
            snap.get("start_button", 0),
            snap.get("apps1_raw", 0),    snap.get("apps2_raw", 0),
            snap.get("brake_raw", 0),
            snap.get("torque_pct", 0),   snap.get("ev_2_3", 0),
            snap.get("t11_8_9", 0),      snap.get("state", 0),
            snap.get("ok_precharge", 0), snap.get("ams_fsm_state", 0),
            snap.get("v_cell_min_mV", 0), snap.get("soc", 0),
            *(vmin[i] if i < len(vmin) else 0 for i in range(5)),
            *(vmax[i] if i < len(vmax) else 0 for i in range(5)),
            snap.get("corriente_accu", 0), snap.get("corriente_dcdc", 0),
            snap.get("temp_dcdc", 0),
            *(tmax[i] if i < len(tmax) else 0 for i in range(5)),
            snap.get("inv_state", 0),     snap.get("last_vconfig_tick", 0),
            snap.get("inv_error", 0),
            snap.get("inv_dc_bus_V", 0),
            snap.get("inv_temp_motor1", 0), snap.get("inv_temp_pwrstg", 0),
            snap.get("inv_temp_board", 0),
            snap.get("inv_rpm", 0),         snap.get("inv_speed_actual", 0),
            snap.get("inv_current_actual", 0),
        ]
        self.writer.writerow(row)
        self.count += 1
        if self.count % 50 == 0:
            self.file.flush()

    def close(self) -> Optional[str]:
        if self.file:
            self.file.close()
            print(f"[DEMO] CSV closed — {self.count} rows → {self.filename}")
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
        self.soc          = 1.0     # State of Charge  0.0–1.0
        self.v_oc         = PACK_V_FULL
        self.v_load       = PACK_V_FULL
        self.pack_current = 0.0     # A  (+ = discharging)
        self.regen_cur    = 0.0     # A  (+ = charging back)
        self.motor_I      = 0.0     # A  (motor phase current)

        # ── Per-module cell voltages (mV) ──────────────────────────────────
        # Each module has 19 series cells; aging shifts average voltage
        self.vmin_mod: List[float] = [
            CELLS_PER_MODULE * (PACK_V_FULL / CELLS_SERIES) * f * 1000
            for f in MOD_AGING
        ]
        self.vmax_mod: List[float] = list(self.vmin_mod)

        # ── Thermal state (°C) ─────────────────────────────────────────────
        self.T_motor = T_AMB
        self.T_pwrstg= T_AMB
        self.T_board = T_AMB
        self.T_dcdc  = T_AMB
        self.T_bat   = [T_AMB] * NUM_MODULES   # one per module
        self.T_brake = [T_AMB] * 4             # FL / FR / RL / RR

        # ── EV state machine ───────────────────────────────────────────────
        # 0=OFF  1=PRECHARGE  2=READY  3=RUNNING  4=ERROR
        self.ctrl_state = 0
        self.ams_state  = 0
        self.inv_state  = 0
        self.inv_error  = 0

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
            f"SoC=100 %  V_pack={PACK_V_FULL:.0f} V  "
            f"Track={TRACK_LEN:.0f} m/lap"
        )

    def stop(self) -> None:
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        if self.logger:
            fp = self.logger.close()
            if self.use_marple and fp and _MARPLE_OK:
                print("[DEMO] Uploading to Marple…")
                isc_marple.upload_session_csv(fp, {
                    "piloto":   self.logger.piloto,
                    "circuito": self.logger.circuito,
                    "type":     "DemoSim_v2",
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
        self.v_oc = self.v_load = PACK_V_FULL
        self.pack_current = self.regen_cur = self.motor_I = 0.0
        self.T_motor = self.T_pwrstg = self.T_board = self.T_dcdc = T_AMB
        self.T_bat   = [T_AMB] * NUM_MODULES
        self.T_brake = [T_AMB] * 4
        self.ctrl_state = self.ams_state = self.inv_state = self.inv_error = 0
        self.vmin_mod = [
            CELLS_PER_MODULE * (PACK_V_FULL / CELLS_SERIES) * f * 1000
            for f in MOD_AGING
        ]
        self.vmax_mod = list(self.vmin_mod)

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
                p_limit = MOTOR_MAX_POWER / (motor_rpm * math.pi / 30.0)
                torque_cmd = min(torque_cmd, p_limit)

            traction_F = torque_cmd * GEAR_RATIO / WHEEL_R
            mech_P     = traction_F * self.v if self.v > 0 else 0.0
            elec_P     = mech_P / (MOTOR_EFF * INV_EFF)

            # 3. REGEN BRAKING ───────────────────────────────────────────────
            brk_F = 0.0; regen_P = 0.0
            if self.brk > 0.5 and self.v > 0.5:
                brk_F   = (self.brk / 100.0) * BRAKE_FORCE_MAX
                regen_P = min(self.v * brk_F * REGEN_FRAC, MAX_REGEN_KW * 1000.0)

            # 4. DYNAMICS ────────────────────────────────────────────────────
            drag_F      = 0.5 * AIR_RHO * DRAG_CD * FRONTAL_A * self.v ** 2
            net_F       = traction_F - brk_F - drag_F
            self.accel  = net_F / CAR_MASS
            self.v      = max(0.0, self.v + self.accel * self.DT)
            self.dist  += self.v * self.DT
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
            # OCV: linear map SoC → [PACK_V_DEPLETED, PACK_V_FULL]
            self.v_oc = PACK_V_DEPLETED + self.soc * (PACK_V_FULL - PACK_V_DEPLETED)

            dcdc_I = DCDC_POWER_W / max(self.v_oc, 1.0)

            if regen_P > 0:
                self.regen_cur    = regen_P / max(self.v_oc, 1.0)
                net_I             = dcdc_I - self.regen_cur
            else:
                self.regen_cur    = 0.0
                net_I             = elec_P / max(self.v_oc, 1.0) + dcdc_I

            self.pack_current = net_I
            self.v_load       = self.v_oc - net_I * PACK_R_INT
            self.v_load       = max(PACK_V_DEPLETED - 5, self.v_load)

            # Coulomb counting
            self.soc -= (net_I * self.DT) / (PACK_CAP_AH * 3600.0)
            self.soc  = max(0.01, min(1.0, self.soc))

            # Motor phase current (approximation)
            self.motor_I = elec_P / max(self.v_load, 1.0) if self.v_load > 0 else 0.0

            # 6. THERMAL MODEL ───────────────────────────────────────────────
            # Motor: mechanical losses → heating
            Q_mech = mech_P * 0.08        # 8 % of mech power as heat
            T_ss_motor  = T_AMB + Q_mech / 70.0   # °C steady-state
            self.T_motor = self._tau(self.T_motor, T_ss_motor, TAU_MOTOR)

            # Inverter PWRSTG: switching & conduction losses
            Q_inv = elec_P * 0.03
            T_ss_pwrstg = T_AMB + Q_inv / 45.0
            self.T_pwrstg= self._tau(self.T_pwrstg, T_ss_pwrstg, TAU_PWRSTG)

            # Controller board: constant low-power dissipation
            self.T_board = self._tau(self.T_board, T_AMB + 12.0, TAU_BOARD)

            # DC-DC converter
            Q_dcdc = DCDC_POWER_W * 0.06
            self.T_dcdc = self._tau(self.T_dcdc, T_AMB + Q_dcdc / 8.0, TAU_DCDC)

            # Battery modules: I²R + airflow cooling (each module slightly different)
            for i in range(NUM_MODULES):
                I_mod     = net_I                          # series circuit → same I
                Q_bat     = (I_mod ** 2) * (PACK_R_INT / NUM_MODULES) * MOD_AGING[i]
                # Cooling increases with airspeed
                cool_C    = 0.45 + self.v * 0.012          # W/°C convection
                T_ss_bat  = T_AMB + Q_bat / cool_C + i * 0.4  # thermal gradient
                self.T_bat[i] = self._tau(self.T_bat[i], T_ss_bat, TAU_BAT)

            # Brake discs: friction heating vs. airspeed cooling
            per_disc_F = brk_F / 4.0
            for i in range(4):
                Q_disc   = per_disc_F * self.v * (1.0 - REGEN_FRAC) * BRAKE_TH_GAIN
                cool_br  = 0.06 + self.v * 0.014   # °C/s per °C above ambient
                dT       = (self.T_brake[i] - T_AMB) * cool_br * self.DT
                self.T_brake[i] = max(T_AMB, self.T_brake[i] + Q_disc - dT)

            # 7. PER-MODULE VOLTAGES ─────────────────────────────────────────
            cell_voc = self.v_oc / CELLS_SERIES    # average cell OCV (V)
            for i in range(NUM_MODULES):
                spread  = 4.0 * (1.0 - MOD_AGING[i])   # mV imbalance from aging
                noise   = random.gauss(0.0, 0.8)        # thermal + ADC noise (mV)
                v_nom   = cell_voc * CELLS_PER_MODULE * MOD_AGING[i] * 1000  # mV
                # Loaded voltage: subtract resistive drop
                v_nom  -= (net_I * (PACK_R_INT / NUM_MODULES)) * 1000
                self.vmin_mod[i] = max(3300.0, v_nom - spread + noise)
                self.vmax_mod[i] = v_nom + spread * 0.4 + abs(noise) * 0.3

            # 8. BUILD & PUBLISH SNAPSHOT ────────────────────────────────────
            snap = self._build_snapshot(motor_rpm)
            self._publish(snap)

            # CSV at ~10 Hz
            if time.time() - last_csv_t >= 0.10:
                with self._lock:
                    if self.logger:
                        self.logger.log(snap, self.t)
                last_csv_t = time.time()

            self.t   += self.DT
            self.seq  = (self.seq + 1) & 0xFFFF
            time.sleep(max(0.0, self.DT - (time.time() - t0)))

    # ── Driver model (look-ahead braking) ───────────────────────────────────
    def _driver(self) -> tuple[float, float]:
        _len, target_kph, _r, _prog, dist_rem = self._current_seg()
        next_len, next_kph, _nr = self._next_seg()

        target_v      = target_kph / 3.6
        next_target_v = next_kph  / 3.6

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

    # ── Build snapshot dict (mirrors _decode_snapshot() format) ─────────────
    def _build_snapshot(self, motor_rpm: float) -> dict:
        # APPS ADC
        apps1 = int(APPS_IDLE + (self.thr / 100.0) * (APPS_MAX - APPS_IDLE))
        apps2 = int(APPS_IDLE + (self.thr / 100.0) * (APPS_MAX - APPS_IDLE)
                    + APPS2_OFFSET + random.gauss(0, 2))
        brake_adc = int(BRAKE_IDLE + (self.brk / 100.0) * (BRAKE_MAX - BRAKE_IDLE))
        apps1     = max(0, min(ADC_FULL, apps1))
        apps2     = max(0, min(ADC_FULL, apps2))
        brake_adc = max(0, min(ADC_FULL, brake_adc))

        soc_pct     = int(self.soc * 100)
        vcell_min   = min(self.vmin_mod)
        corriente_a = int(self.pack_current * 10)      # dA  — raw int16
        corriente_d = int((DCDC_POWER_W / max(self.v_load, 1.0)) * 10)

        return {
            "tick_ms":           int(self.t * 1000) & 0xFFFFFFFF,
            "seq":               self.seq,
            "start_button":      1,
            "apps1_raw":         apps1,
            "apps2_raw":         apps2,
            "brake_raw":         brake_adc,
            "torque_pct":        int(self.thr),
            "ev_2_3":            1,
            "t11_8_9":           1,
            "state":             self.ctrl_state,
            "ok_precharge":      1,
            "ams_fsm_state":     self.ams_state,
            "v_cell_min_mV":     int(vcell_min),
            "soc":               soc_pct,
            "vmin_modulo":       [int(v) for v in self.vmin_mod],
            "vmax_modulo":       [int(v) for v in self.vmax_mod],
            "corriente_accu":    corriente_a,
            "corriente_dcdc":    corriente_d,
            "temp_dcdc":         int(self.T_dcdc),
            "temp_max_modulo":   [int(t) for t in self.T_bat],
            "inv_state":         self.inv_state,
            "last_vconfig_tick": 1,
            "inv_error":         self.inv_error,
            "inv_dc_bus_V":      max(300, int(self.v_load)),
            "inv_temp_motor1":   int(self.T_motor),
            "inv_temp_pwrstg":   int(self.T_pwrstg),
            "inv_temp_board":    int(self.T_board),
            "inv_rpm":           int(motor_rpm),
            "inv_speed_actual":  int(motor_rpm),
            "inv_current_actual": int(self.motor_I),
        }

    # ── Publish snapshot into rtt module (same path as real radio data) ──────
    def _publish(self, snap: dict) -> None:
        rtt.latest_data_dict["snapshot"] = snap

        # Badge / status (same as _set_badge in rtt)
        rtt.latest_data_dict["__STATUS__"] = {
            "badge":  "LIVE",
            "reason": "demo",
            "ts":     int(time.time() * 1000),
        }

        # Backward-compat aggregate keys (used by any legacy UI code)
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
        rtt.latest_data_dict["ams_temp_summary"] = {
            "max_temp_c": max(tmax) if tmax else 0,
            "min_temp_c": min(tmax) if tmax else 0,
            "avg_temp_c": sum(tmax) / len(tmax) if tmax else 0,
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
            f"I={snap['corriente_accu']/10.0:.1f} A"
        )

    # ── Compatibility: expose brake temps & G-force for legacy access ────────
    def get_brake_temps(self) -> List[float]:
        return list(self.T_brake)

    def get_g_forces(self) -> dict:
        g_tot = math.sqrt(self.g_long ** 2 + self.g_lat ** 2)
        return {"g_long": self.g_long, "g_lat": self.g_lat, "g_total": g_tot}


# ══════════════════════════════════════════════════════════════════════════════
#  MODULE-LEVEL API  (matches v1 interface used by ISCmetrics)
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