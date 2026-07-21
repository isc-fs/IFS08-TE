"""
ISCmetrics v2 — ISC Formula Student Telemetry System
Developed by Andrés Sánchez de Ágreda © 2025/2026
F1 / Grafana dark-mode visualization — Fragmented Snapshot Protocol
"""

from __future__ import annotations
import sys
import math
import threading
import time
import logging
import subprocess
import hashlib
import webbrowser
try:
    import requests as _requests
    _REQUESTS_OK = True
except ImportError:
    _REQUESTS_OK = False

_CAN_OK = False
try:
    import can
    _CAN_OK = True
except ImportError:
    pass

import struct
import zlib
import csv
import io
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Optional, Dict, List, Deque

import numpy as np
import matplotlib
matplotlib.use("Qt5Agg")

from PyQt5.QtCore import (
    QTimer, Qt, QMimeData, QObject, pyqtSignal, QPoint, QThread
)
from PyQt5.QtGui import (
    QFont, QPalette, QColor, QPixmap, QIcon,
    QPainter, QPen, QBrush, QLinearGradient, QDrag
)
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QGridLayout,
    QWidget, QLabel, QPushButton, QLineEdit, QComboBox, QTextEdit,
    QMessageBox, QTabWidget, QFrame, QGroupBox, QCheckBox,
    QListWidget, QListWidgetItem, QDialog, QSizePolicy, QInputDialog,
    QProgressBar, QTableWidget, QTableWidgetItem, QAbstractItemView, QHeaderView
)
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

import ISC_RTT_serial as rtt

# ── Optional demo module ──────────────────────────────────────────────────────
try:
    import ISC_RTT_demo as demo
    DEMO_AVAILABLE = True
except ImportError:
    DEMO_AVAILABLE = False

# ══════════════════════════════════════════════════════════════════════════════
#  VERSION  — patched automatically by GitHub Actions on each release tag
# ══════════════════════════════════════════════════════════════════════════════
APP_VERSION    = "2.2.0"
_RELEASES_URL  = "https://api.github.com/repos/MrAndy5/ISCmetrics/releases/latest"
_RELEASES_PAGE = "https://github.com/MrAndy5/ISCmetrics/releases/latest"

logger = logging.getLogger("ISC_RTT_USB")

# ── NxTech Inverter FSM State Machine ────────────────────────────────────────
# Source: NxTech Portal — Controller state machine section
# App_State_App / App_State_Req enumeration (both Tx and Rx use same values)
INVERTER_STATES_MAP = {
    1:  "INIT",               # Initialization state
    2:  "POST",               # Power-On Self Test
    3:  "STANDBY",            # Awaiting HV
    4:  "READY",              # HV detected, ready for active control
    5:  "CURRENT",            # Current control (active)
    6:  "TORQUE",             # Torque control (active)
    7:  "SPEED",              # Speed control (active)
    8:  "Placeholder",
    9:  "Placeholder",
    10: "FAULT_SOFT",         # Non-latched fault — reset via STANDBY request
    11: "FAULT_HARD",         # Latched fault — needs power cycle or SHUTDOWN request
    12: "DISCHARGE",          # Discharging DC-Link
    13: "SHUTDOWN",           # Shutting down (power saving)
    14: "OFF",                # LV is off
}
# Active control states (torque is being applied)
_INV_ACTIVE_STATES = {5, 6, 7}

# ── NxTech DEM Diagnostic Codes (L3 — DEM_Code) ─────────────────────────────
# Source: NxTech Portal — Diagnostics section, L3 table
# inv_error = DEM_Code from EMC_TX_STATE_2 (0x461).
# The DEM cycles through all registered errors/warnings; 0 = everything OK.
INVERTER_ERRORS_MAP = {
    0:  "No Fault",
    1:  "Lost msg: setpoint not received",
    2:  "DCBus Undervoltage",
    3:  "PwrStg Overtemperature",
    4:  "PwrStg Temp Degradation",
    5:  "EMCtrl Fault (see PwrStg_BitState / EMCtrl_FOC_BitState)",
    6:  "Task Overrun",
    7:  "CAN1 BusOff",
    8:  "EMachine Overtemperature",
    9:  "Phase Current Out-of-Range",
    10: "Power Stage Temp Out-of-Range",
    11: "DC Bus Out-of-Range",
    12: "DP Overtemp",
    13: "DRV Overtemp",
    14: "Aux Supply Undervoltage",
    15: "Aux Supply Overvoltage",
    16: "Overspeed",
    17: "Speed Degrade",
    18: "EMachine Temp Degrade",
    19: "Bad Current Offset",
    20: "AbsEnc Error 1",
    21: "Ext Temp 1 Out-of-Range",     # Sensor 1 disconnected / inadequate wiring
    22: "Ext Temp 2 Out-of-Range",     # Sensor 2 (KTY81-210 motor winding) out of range
    23: "PMIC Not Ready",
    24: "E2E CRC Fault (setpoint)",
    25: "E2E CNT Fault (setpoint)",
    26: "Invalid Calibration",
    27: "E2E CRC Fault (state)",
    28: "E2E CNT Fault (state)",
    29: "Crosscheck Fault",
    30: "Crosscheck Torque",
    31: "Wrong Member",
    32: "Hardware Supervisor Fault",   # KL30/31 supply inadequate
    33: "KL30 Undervoltage",
    34: "KL30 Overvoltage",
    35: "Lost Message (setpoint)",
    36: "Torque Not Coherent",
    37: "LV External Sensor Supply Fault",  # 5V supply shorted or KL30/31 inadequate
}

# ── DEM Safe State reference ──────────────────────────────────────────────────
INVERTER_ERRORS_SAFESTATE = {
    1: "Freewheeling", 2: "Freewheeling", 3: "Freewheeling", 4: "Degrade",
    5: "Freewheeling", 6: "Freewheeling", 7: "None", 8: "Freewheeling",
    9: "Freewheeling", 10: "Freewheeling", 11: "Freewheeling", 12: "Freewheeling",
    13: "Freewheeling", 14: "Freewheeling", 15: "None", 16: "Freewheeling",
    17: "Degrade", 18: "Degrade", 19: "Freewheeling", 20: "Freewheeling",
    21: "None", 22: "None", 23: "Freewheeling", 24: "Freewheeling",
    25: "Freewheeling", 26: "Freewheeling", 27: "Freewheeling", 28: "Freewheeling",
    29: "Freewheeling", 30: "Freewheeling", 31: "Freewheeling", 32: "Freewheeling",
    33: "Freewheeling", 34: "Freewheeling", 35: "Freewheeling", 36: "Freewheeling",
    37: "Freewheeling",
}

def decode_inverter_state(state_code: int) -> str:
    name = INVERTER_STATES_MAP.get(state_code, f"State {state_code}")
    # Official descriptions per NxTech documentation
    descs = {
        1: "Init",
        2: "Power-On Self Test",
        3: "Awaiting HV",
        4: "HV Ready",
        5: "Current Active",
        6: "Torque Active",
        7: "Speed Active",
        10: "Non-latched Fault",
        11: "Latched Fault / Shutdown",
        12: "Discharging DC-Link",
        13: "Power Saving Sleep",
        14: "LV Off",
    }
    desc = descs.get(state_code)
    return f"{name} ({desc})" if desc else name

def decode_inverter_errors(error_code: int, state_code: int = 0) -> list:
    """Decode DEM_Code from EMC_TX_STATE_2. Returns empty list when no fault."""
    if error_code == 0:
        return []
    desc = INVERTER_ERRORS_MAP.get(error_code, f"DEM Code {error_code} (Unknown)")
    safe = INVERTER_ERRORS_SAFESTATE.get(error_code, "")
    suffix = f" [{safe}]" if safe else ""
    fault_type = "SOFT" if state_code == 10 else "HARD" if state_code == 11 else "DEM"
    return [f"{fault_type}: {desc}{suffix}"]


# ══════════════════════════════════════════════════════════════════════════════
#  COLOUR SCHEME  (ISC Green / Grafana dark)
# ══════════════════════════════════════════════════════════════════════════════
ISC_GREEN   = '#008000'
F1_DARK_BG  = '#111111'
F1_MID_BG   = '#1a1a1a'
F1_PANEL_BG = '#222222'
F1_TEXT     = '#e0e0e0'
F1_ACCENT   = ISC_GREEN          # backward-compat alias
F1_WARNING  = '#f0b429'
F1_ERROR    = '#ef4444'
F1_BLUE     = '#3b82f6'
F1_PURPLE   = '#8b5cf6'

# ── Marple upload password (SHA-256 of 'ISC_telemetry_2026') ─────────────────
_MARPLE_PASSWORD_HASH = hashlib.sha256(b"ISC_telemetry_2026").hexdigest()

# Alert thresholds
ALERT_TEMP_C   = 40.0   # °C   — any module max temp above this
ALERT_VOLT_V   = 380    # V    — DC bus below this
ALERT_CELL_MV  = 3400   # mV   — per-module min cell voltage below this

HISTORY_LEN  = 120    # rolling plot sample depth
ADC_MAX      = 4095   # 12-bit ADC full scale (pedal normalisation)
APPS1_MIN    = 2500   # raw ADC value at 0% depression
APPS1_MAX    = 3400   # raw ADC value at 100% depression
APPS2_MIN    = 2330   # raw ADC value at 0% depression
APPS2_MAX    = 3040   # raw ADC value at 100% depression
RPM_MAX      = 6000

# ── Sony VTC6 95s6p OCV–SoC lookup ───────────────────────────────────────────
# 95 cells in series × 6 cells in parallel = 570 total cells
# Pack capacity: 6 × 3.0 Ah = 18 Ah   |   Max voltage: 95 × 4.2 V ≈ 399 V
# OCV table: (SoC_fraction, cell_OCV_V)  — derived from Sony VTC6 discharge curve
_VTC6_OCV_TABLE = [
    (1.000, 4.200), (0.950, 4.150), (0.900, 4.100), (0.800, 4.020),
    (0.700, 3.940), (0.600, 3.870), (0.500, 3.800), (0.400, 3.740),
    (0.300, 3.680), (0.200, 3.600), (0.100, 3.500), (0.050, 3.400),
    (0.000, 3.000),
]

def soc_from_cell_mv(cell_mv: float, current_a: float = 0.0) -> float:
    """
    Estimate SoC (0–100 %) from cell voltage using VTC6 OCV table with IR compensation.
    95s6p Sony VTC6 pack internal resistance ≈ 0.35 mV/A per 6p cell group.
    Compensates voltage sag during heavy current draw (e.g. 376A acceleration spikes)
    so SoC readout remains stable during driving.
    """
    # Add back IR voltage drop during discharge (I > 0)
    ir_drop_mv = max(0.0, float(current_a)) * 0.35
    ocv_est_mv = cell_mv + ir_drop_mv
    
    cell_v = ocv_est_mv / 1000.0
    for i in range(len(_VTC6_OCV_TABLE) - 1):
        soc_hi, v_hi = _VTC6_OCV_TABLE[i]
        soc_lo, v_lo = _VTC6_OCV_TABLE[i + 1]
        if v_lo <= cell_v <= v_hi:
            frac = (cell_v - v_lo) / (v_hi - v_lo) if v_hi != v_lo else 0.0
            return round((soc_lo + frac * (soc_hi - soc_lo)) * 100.0, 1)
    if cell_v >= _VTC6_OCV_TABLE[0][1]:  return 100.0
    if cell_v <= _VTC6_OCV_TABLE[-1][1]: return 0.0
    return 0.0

# ── All ECU signals available in the Customise tab ───────────────────────────
# Format: 'snapshot_key': ('Display Label', 'unit')
SNAPSHOT_CHANNELS: Dict[str, tuple] = {
    # ── Frame header ───────────────────────────────────────────────────────
    'tick_ms':              ('RTOS Tick',              'ms'),
    'seq':                  ('Snapshot Seq',           ''),
    # ── Driver inputs ──────────────────────────────────────────────────────
    'start_button':         ('Start Button',           '0/1'),
    'apps1_raw':            ('APPS Sensor A',          'ADC'),
    'apps2_raw':            ('APPS Sensor B',          'ADC'),
    'brake_raw':            ('Brake Pressure',         'ADC'),
    # ── Control FSM ───────────────────────────────────────────────────────
    'torque_pct':           ('Torque Request',         '%'),
    'ev_2_3':               ('EV 2/3 Flags',           ''),
    't11_8_9':              ('T.11 Interlock',         ''),
    'state':                ('Control FSM State',      ''),
    # ── Battery / AMS — pack level ─────────────────────────────────────────
    'ok_precharge':         ('Precharge OK',           '0/1'),
    'ams_fsm_state':        ('AMS FSM State',          ''),
    'v_cell_min_mV':        ('Min Cell Voltage',       'mV'),
    'soc':                  ('State of Charge',        '%'),
    # ── Battery / AMS — per-module min cell voltage ────────────────────────
    'vmin_modulo_0':        ('Vmin Module 0',          'mV'),
    'vmin_modulo_1':        ('Vmin Module 1',          'mV'),
    'vmin_modulo_2':        ('Vmin Module 2',          'mV'),
    'vmin_modulo_3':        ('Vmin Module 3',          'mV'),
    'vmin_modulo_4':        ('Vmin Module 4',          'mV'),
    # ── Battery / AMS — per-module max cell voltage ────────────────────────
    'vmax_modulo_0':        ('Vmax Module 0',          'mV'),
    'vmax_modulo_1':        ('Vmax Module 1',          'mV'),
    'vmax_modulo_2':        ('Vmax Module 2',          'mV'),
    'vmax_modulo_3':        ('Vmax Module 3',          'mV'),
    'vmax_modulo_4':        ('Vmax Module 4',          'mV'),
    # ── Battery / AMS — current & DC-DC ───────────────────────────────────
    'corriente_accu':       ('Pack Current',           'A'),
    'corriente_dcdc':       ('DC-DC Current',          'A'),
    'temp_dcdc':            ('DC-DC Temperature',      'ºC'),
    # ── Battery / AMS — per-module max temperature ─────────────────────────
    'tmax_modulo_0':        ('Tmax Module 0',          'ºC'),
    'tmax_modulo_1':        ('Tmax Module 1',          'ºC'),
    'tmax_modulo_2':        ('Tmax Module 2',          'ºC'),
    'tmax_modulo_3':        ('Tmax Module 3',          'ºC'),
    'tmax_modulo_4':        ('Tmax Module 4',          'ºC'),
    # ── Inverter status ───────────────────────────────────────────────────
    'inv_state':            ('Inverter State',         ''),
    'inv_vconfig_active':   ('Vconfig Active',         '0/1'),
    'inv_error':            ('Inverter Error (DEM)',   ''),
    'dem_code':             ('DEM Code',               ''),
    'emctrl_foc_bitstate':  ('EMCtrl FOC BitState',    'bitfield'),
    'est_time_remaining':   ('Est. Time Remaining',    'min'),
    # ── Inverter electrical ───────────────────────────────────────────────
    'inv_dc_bus_V':         ('DC Bus Voltage',         'V'),
    'inv_temp_motor1':      ('Ext Temp Sensor 1',      'ºC'),   # NTC on Sensor 1 input (disconnected = 255)
    'inv_temp_motor2':      ('Motor 2 Winding Temp',   'ºC'),   # NTC on Sensor 2 input (KTY81-210 working sensor)
    'inv_temp_pwrstg':      ('Power Stage Temp',       'ºC'),   # Alias for Motor 2 / Sensor 2
    'inv_temp_board':       ('Inverter Board Temp',    'ºC'),
    # ── Motor speed & current ─────────────────────────────────────────────
    'inv_rpm':              ('Motor Speed',            'RPM'),
    'inv_speed_actual':     ('Vehicle Speed',          'km/h'),
    'inv_current_actual':   ('Motor Current',          'A'),
}

plt.style.use('dark_background')
plt.rcParams.update({
    'axes.facecolor':   F1_DARK_BG,
    'figure.facecolor': F1_DARK_BG,
    'text.color':       F1_TEXT,
    'axes.labelcolor':  F1_TEXT,
    'xtick.color':      F1_TEXT,
    'ytick.color':      F1_TEXT,
    'axes.edgecolor':   '#333333',
    'grid.color':       '#2a2a2a',
    'grid.alpha':       1.0,
})

current_settings: dict = {
    "port":       rtt.DEFAULT_PORT,
    "baud":       rtt.DEFAULT_BAUD,
    "use_influx": False,
    "debug":      rtt.DEBUG_ENABLE_DEFAULT,
    "demo_mode":  False,
    "enable_tts": True,
    "alert_temp_c":  ALERT_TEMP_C,
    "alert_volt_v":  ALERT_VOLT_V,
    "alert_cell_mv": ALERT_CELL_MV,
}


# ══════════════════════════════════════════════════════════════════════════════
#  SIGNAL EMITTER
# ══════════════════════════════════════════════════════════════════════════════
class Signaler(QObject):
    log_message = pyqtSignal(str)
    update_detected = pyqtSignal(str, str)    # version_tag, download_url
    download_progress = pyqtSignal(int)
    download_finished = pyqtSignal(str)     # filepath or error message starting with "ERR:"

signaler = Signaler()

class _QtLogHandler(logging.Handler):
    def __init__(self, sig: Signaler):
        super().__init__()
        self._sig = sig
    def emit(self, record):
        self._sig.log_message.emit(self.format(record))

logging.getLogger("ISC_RTT_USB").addHandler(_QtLogHandler(signaler))


# ══════════════════════════════════════════════════════════════════════════════
#  ALERT BANNER
# ══════════════════════════════════════════════════════════════════════════════
class AlertBanner(QFrame):
    """Flashing coloured strip shown when alert conditions are active."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self._alerts: List[tuple] = []
        self._flash  = False
        self._base   = F1_ERROR
        self.setFixedHeight(28)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.hide()

        lay = QHBoxLayout(self)
        lay.setContentsMargins(10, 2, 10, 2)
        self._lbl = QLabel("")
        self._lbl.setAlignment(Qt.AlignCenter)
        lay.addWidget(self._lbl)

        self._ftimer = QTimer(self)
        self._ftimer.timeout.connect(self._toggle_flash)
        self._ftimer.start(450)

    def set_alerts(self, alerts: List[tuple]) -> None:
        self._alerts = alerts
        if not alerts:
            self.hide()
            return
        self.show()
        self._lbl.setText("   |   ".join(a[0] for a in alerts))
        self._lbl.setStyleSheet(f"color: {F1_DARK_BG}; font-size: 11px; font-weight: bold;")
        self._base = F1_ERROR if any(a[1] == 'critical' for a in alerts) else F1_WARNING

    def _toggle_flash(self):
        if not self._alerts:
            return
        self._flash = not self._flash
        dim = '#5a0000' if self._base == F1_ERROR else '#5a3e00'
        col = self._base if self._flash else dim
        self.setStyleSheet(f"QFrame {{ background: {col}; border-radius: 2px; }}")


# ══════════════════════════════════════════════════════════════════════════════
#  ROLLING PLOT CANVAS
# ══════════════════════════════════════════════════════════════════════════════
class MplCanvas(QWidget):
    """Grafana-style rolling time-series plot embedded in a QWidget."""
    def __init__(self, title: str = "", color: str = ISC_GREEN, parent=None):
        super().__init__(parent)
        self._color   = color
        self._history: Deque[float] = deque([0.0] * HISTORY_LEN, maxlen=HISTORY_LEN)

        fig = Figure(figsize=(4, 2), tight_layout=True)
        fig.patch.set_facecolor(F1_PANEL_BG)
        self._ax = fig.add_subplot(111)
        self._line, = self._ax.plot([], [], color=color, linewidth=1.4, antialiased=True)
        self._ax.set_facecolor(F1_DARK_BG)
        self._ax.set_title(title, color=color, fontsize=8, fontweight='bold', pad=2)
        self._ax.tick_params(labelsize=7.5, colors='#ffffff' if F1_TEXT == '#e0e0e0' else '#111111')
        self._ax.grid(True, color='#444444' if F1_TEXT == '#e0e0e0' else '#cccccc', alpha=0.35)
        for s in self._ax.spines.values():
            s.set_color('#555555' if F1_TEXT == '#e0e0e0' else '#aaaaaa')

        canvas = FigureCanvas(fig)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(canvas)
        self._canvas = canvas

    def update_plot(self, value: float) -> None:
        self._history.append(float(value))
        y = list(self._history)
        x = list(range(len(y)))
        self._line.set_data(x, y)
        for coll in self._ax.collections:
            coll.remove()
        self._ax.fill_between(x, y, alpha=0.10, color=self._color)
        lo, hi = min(y), max(y)
        mg = max((hi - lo) * 0.1, 1.0)
        self._ax.set_xlim(0, HISTORY_LEN)
        self._ax.set_ylim(lo - mg, hi + mg)
        self._canvas.draw_idle()


# ══════════════════════════════════════════════════════════════════════════════
#  METRIC CARD
# ══════════════════════════════════════════════════════════════════════════════
class MetricCard(QFrame):
    """Grafana-style metric tile: title, large value, unit, alert highlight."""
    def __init__(self, title: str, unit: str = "", color: str = ISC_GREEN, parent=None):
        super().__init__(parent)
        self._color   = color
        self._alerting = False
        self._set_border(color)

        v = QVBoxLayout(self)
        v.setContentsMargins(8, 5, 8, 5)
        v.setSpacing(1)

        self._title = QLabel(title)
        self._title.setAlignment(Qt.AlignCenter)
        self._title.setStyleSheet(f"color:{color}; font-size:9px; font-weight:bold; background:transparent; border:none;")

        self._value = QLabel("—")
        self._value.setAlignment(Qt.AlignCenter)
        self._value.setStyleSheet(f"color:{F1_TEXT}; font-size:19px; font-weight:bold; background:transparent; border:none;")

        self._unit = QLabel(unit)
        self._unit.setAlignment(Qt.AlignCenter)
        self._unit.setStyleSheet("color:#555; font-size:8px; background:transparent; border:none;")

        v.addWidget(self._title)
        v.addWidget(self._value)
        v.addWidget(self._unit)

    def _set_border(self, color: str) -> None:
        self.setStyleSheet(f"QFrame {{ background:{F1_PANEL_BG}; border:1px solid {color}; border-radius:3px; }}")

    def set_value(self, text: str) -> None:
        self._value.setText(text)

    def set_alert(self, active: bool) -> None:
        if active == self._alerting:
            return
        self._alerting = active
        col = F1_ERROR if active else self._color
        self.setStyleSheet(f"QFrame {{ background:{F1_PANEL_BG}; border:2px solid {col}; border-radius:3px; }}")
        self._title.setStyleSheet(f"color:{col}; font-size:9px; font-weight:bold; background:transparent; border:none;")


# ══════════════════════════════════════════════════════════════════════════════
#  RPM GAUGE  (circular, QPainter)
# ══════════════════════════════════════════════════════════════════════════════
class RPMGauge(QWidget):
    """Circular RPM gauge with green→yellow→red arc."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self._rpm = 0.0
        self.setMinimumSize(180, 180)

    def set_rpm(self, rpm: float) -> None:
        self._rpm = max(0.0, min(float(rpm), RPM_MAX))
        self.update()

    def paintEvent(self, _ev):
        w, h  = self.width(), self.height()
        side  = min(w, h) - 12
        cx, cy = w // 2, h // 2
        r = side // 2

        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        # Background disc
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor(F1_PANEL_BG)))
        p.drawEllipse(cx - r, cy - r, side, side)

        # Track
        track_w = max(8, r // 8)
        pad = track_w + 6
        rect_x, rect_y = cx - r + pad, cy - r + pad
        diam = (r - pad) * 2
        p.setPen(QPen(QColor('#2a2a2a'), track_w, Qt.SolidLine, Qt.RoundCap))
        p.setBrush(Qt.NoBrush)
        p.drawArc(rect_x, rect_y, diam, diam, 225 * 16, -270 * 16)

        # Coloured arc
        frac  = self._rpm / RPM_MAX
        sweep = int(frac * 270 * 16)
        col   = ISC_GREEN if frac < 0.60 else (F1_WARNING if frac < 0.85 else F1_ERROR)
        p.setPen(QPen(QColor(col), track_w, Qt.SolidLine, Qt.RoundCap))
        p.drawArc(rect_x, rect_y, diam, diam, 225 * 16, -sweep)

        # RPM text
        p.setPen(QColor(F1_TEXT))
        p.setFont(QFont("Arial", max(10, r // 4), QFont.Bold))
        p.drawText(cx - r, cy - r // 3, side, side // 2, Qt.AlignCenter, f"{int(self._rpm):,}")

        p.setPen(QColor(col))
        p.setFont(QFont("Arial", max(7, r // 9)))
        p.drawText(cx - r, cy + r // 6, side, r // 2, Qt.AlignCenter, "RPM")
        p.end()


# ══════════════════════════════════════════════════════════════════════════════
#  PEDAL WIDGET  (vertical bar)
# ══════════════════════════════════════════════════════════════════════════════
class PedalWidget(QWidget):
    """Vertical bar pedal indicator with gradient fill."""
    def __init__(self, label: str, color: str = ISC_GREEN, parent=None):
        super().__init__(parent)
        self._label  = label
        self._color  = color
        self._norm   = 0.0   # 0.0 – 1.0
        self._raw    = 0
        self.setMinimumSize(90, 180)

    def set_value(self, normalized: float, raw: int = 0) -> None:
        self._norm = max(0.0, min(1.0, float(normalized)))
        self._raw  = raw
        self.update()

    def paintEvent(self, _ev):
        w, h = self.width(), self.height()
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        bw = int(w * 0.38)
        bh = int(h * 0.62)
        bx = (w - bw) // 2
        by = int(h * 0.10)

        # Track
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor('#e0e0e0' if F1_TEXT == '#1a1a1a' else '#1e1e1e')))
        p.drawRoundedRect(bx, by, bw, bh, 4, 4)

        # Fill (bottom-up)
        fh = int(bh * self._norm)
        if fh > 0:
            grad = QLinearGradient(bx, by + bh, bx, by)
            grad.setColorAt(0.0, QColor(self._color))
            grad.setColorAt(1.0, QColor(self._color).lighter(140))
            p.setBrush(QBrush(grad))
            p.drawRoundedRect(bx, by + bh - fh, bw, fh, 4, 4)

        # Border
        p.setPen(QPen(QColor(self._color), 1))
        p.setBrush(Qt.NoBrush)
        p.drawRoundedRect(bx, by, bw, bh, 4, 4)

        # Label (top)
        p.setPen(QColor(self._color))
        p.setFont(QFont("Arial", 8, QFont.Bold))
        p.drawText(0, 0, w, by, Qt.AlignCenter | Qt.AlignVCenter, self._label)

        # Percentage
        p.setPen(QColor(F1_TEXT))
        p.setFont(QFont("Arial", 13, QFont.Bold))
        p.drawText(0, by + bh + 4, w, 22, Qt.AlignCenter, f"{int(self._norm * 100)}%")

        # Raw
        p.setPen(QColor('#555'))
        p.setFont(QFont("Arial", 7))
        p.drawText(0, by + bh + 26, w, 16, Qt.AlignCenter, f"raw {self._raw}")
        p.end()


# ══════════════════════════════════════════════════════════════════════════════
#  G-FORCE CIRCLE
# ══════════════════════════════════════════════════════════════════════════════
class GCircleWidget(QWidget):
    """Circular G-force display with tracking dot."""
    MAX_G = 3.0

    def __init__(self, parent=None):
        super().__init__(parent)
        self._gx = 0.0   # lateral
        self._gy = 0.0   # longitudinal
        self._trail = deque(maxlen=15)
        self.setMinimumSize(160, 160)

    def set_g_force(self, g_long: float, g_lat: float) -> None:
        self._gx = g_lat
        self._gy = g_long
        self._trail.append((g_lat, g_long))
        self.update()

    def paintEvent(self, _ev):
        w, h = self.width(), self.height()
        r    = min(w, h) // 2 - 12
        cx, cy = w // 2, h // 2
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        # Outer ring
        p.setPen(QPen(QColor('#333'), 1))
        p.setBrush(QBrush(QColor(F1_PANEL_BG)))
        p.drawEllipse(cx - r, cy - r, r * 2, r * 2)

        # Inner rings (1 G, 2 G)
        for frac in (1/3, 2/3):
            rr = int(r * frac)
            p.setPen(QPen(QColor('#2a2a2a'), 1, Qt.DashLine))
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(cx - rr, cy - rr, rr * 2, rr * 2)

        # Cross-hairs
        p.setPen(QPen(QColor('#2a2a2a'), 1))
        p.drawLine(cx - r, cy, cx + r, cy)
        p.drawLine(cx, cy - r, cx, cy + r)

        # G-Force vector trails
        num_points = len(self._trail)
        for idx, (gx, gy) in enumerate(self._trail):
            nx = max(-1.0, min(1.0, gx / self.MAX_G))
            ny = max(-1.0, min(1.0, gy / self.MAX_G))
            tx = int(cx + nx * r)
            ty = int(cy - ny * r)
            
            # Fading opacity and size for trailing path
            alpha = int(((idx + 1) / (num_points + 1)) * 140)
            dot_size = max(2, int(8 * ((idx + 1) / (num_points + 1))))
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(QColor(239, 68, 68, alpha)))  # Red trail
            p.drawEllipse(tx - dot_size // 2, ty - dot_size // 2, dot_size, dot_size)

        # Main active G-dot
        nx = max(-1.0, min(1.0, self._gx / self.MAX_G))
        ny = max(-1.0, min(1.0, self._gy / self.MAX_G))
        dot_x = int(cx + nx * r)
        dot_y = int(cy - ny * r)
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor(ISC_GREEN)))
        p.drawEllipse(dot_x - 6, dot_y - 6, 12, 12)

        # Axis labels
        p.setPen(QColor('#444'))
        p.setFont(QFont("Arial", 7))
        p.drawText(cx - r, cy + r + 3, r * 2, 14, Qt.AlignCenter, "← LAT →")
        p.end()


# ══════════════════════════════════════════════════════════════════════════════
#  MODULE BAR WIDGET
# ══════════════════════════════════════════════════════════════════════════════
class ModuleBarWidget(QWidget):
    """Horizontal progress bar for per-module voltage or temperature."""
    def __init__(self, module_id: int, unit: str = "mV",
                 lo: float = 2800, hi: float = 4250,
                 warn_lo: Optional[float] = None,
                 warn_hi: Optional[float] = None,
                 parent=None):
        super().__init__(parent)
        self._id       = module_id
        self._unit     = unit
        self._lo_range = lo
        self._hi_range = hi
        self._warn_lo  = warn_lo
        self._warn_hi  = warn_hi
        self._val_lo   = lo
        self._val_hi   = lo
        self.setFixedHeight(32)

    def set_values(self, lo: float, hi: float) -> None:
        self._val_lo = float(lo)
        self._val_hi = float(hi)
        self.update()

    def _frac(self, v: float) -> float:
        span = self._hi_range - self._lo_range
        return (v - self._lo_range) / span if span else 0.0

    def paintEvent(self, _ev):
        w, h = self.width(), self.height()
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        lw   = 58
        vw   = 90
        bx   = lw + 6
        bw   = w - bx - vw - 4
        bh   = 12
        by   = (h - bh) // 2

        # Module label
        p.setPen(QColor(ISC_GREEN))
        p.setFont(QFont("Arial", 8, QFont.Bold))
        p.drawText(2, 0, lw, h, Qt.AlignVCenter | Qt.AlignLeft, f"MOD {self._id}")

        # Track
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor('#e0e0e0' if F1_TEXT == '#1a1a1a' else '#1e1e1e')))
        p.drawRoundedRect(bx, by, bw, bh, 3, 3)

        # Filled range
        alert = ((self._warn_hi is not None and self._val_hi > self._warn_hi) or
                 (self._warn_lo is not None and self._val_lo < self._warn_lo))
        col = QColor(F1_ERROR if alert else ISC_GREEN)

        x0 = bx + int(max(0, min(1, self._frac(self._val_lo))) * bw)
        x1 = bx + int(max(0, min(1, self._frac(self._val_hi))) * bw)
        fill_w = max(4, x1 - x0)
        p.setBrush(QBrush(col))
        p.drawRoundedRect(x0, by, fill_w, bh, 3, 3)

        # Threshold lines
        for thresh, tcol in [(self._warn_hi, F1_WARNING), (self._warn_lo, F1_ERROR)]:
            if thresh is not None:
                tx = bx + int(max(0, min(1, self._frac(thresh))) * bw)
                p.setPen(QPen(QColor(tcol), 2))
                p.drawLine(tx, by - 3, tx, by + bh + 3)

        # Value text
        p.setPen(QColor(F1_TEXT))
        p.setFont(QFont("Courier New", 8))
        p.drawText(bx + bw + 6, 0, vw, h, Qt.AlignVCenter | Qt.AlignLeft,
                   f"{self._val_lo:.0f}–{self._val_hi:.0f} {self._unit}")
        p.end()



# ══════════════════════════════════════════════════════════════════════════════
#  SIGNAL BARS WIDGET  — WiFi-style link quality indicator
# ══════════════════════════════════════════════════════════════════════════════
class SignalBarsWidget(QWidget):
    """
    Draws 4 rising bars like a WiFi / mobile signal indicator.
      lqi >= 85 %  → 4 bars, green
      lqi >= 70 %  → 3 bars, green-yellow
      lqi >= 50 %  → 2 bars, orange
      lqi >= 25 %  → 1 bar,  red
      lqi <  25 %  → 0 bars (all bars dim), red
    """
    _THRESHOLDS = [85, 70, 50, 25]   # 4,3,2,1 active bars
    _COLOURS = {
        4: "#00c853",  # green
        3: "#8bc34a",  # yellow-green
        2: "#f0b429",  # orange
        1: "#ef4444",  # red
        0: "#ef4444",  # red (all dim)
    }

    def __init__(self, parent=None):
        super().__init__(parent)
        self._lqi = 100.0
        self.setFixedSize(36, 22)
        self.setToolTip("Link Quality Indicator (packet success rate, last 50 snapshots)")

    def set_lqi(self, lqi: float):
        if lqi != self._lqi:
            self._lqi = lqi
            self.update()

    def _active_bars(self) -> int:
        for i, thr in enumerate(self._THRESHOLDS):
            if self._lqi >= thr:
                return 4 - i
        return 0

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        n_active = self._active_bars()
        colour   = QColor(self._COLOURS[n_active])
        dim      = QColor("#2a2a2a")

        num_bars  = 4
        margin    = 2
        w         = self.width()
        h         = self.height()
        bar_w     = max(4, (w - margin * (num_bars + 1)) // num_bars)
        max_bar_h = h - margin * 2

        for i in range(num_bars):
            bar_h   = int(max_bar_h * (i + 1) / num_bars)
            bx      = margin + i * (bar_w + margin)
            by      = h - margin - bar_h
            active  = i < n_active
            p.setBrush(colour if active else dim)
            p.setPen(Qt.NoPen)
            p.drawRoundedRect(bx, by, bar_w, bar_h, 2, 2)

        p.end()



class ChannelListWidget(QListWidget):
    """Drag-enabled list of snapshot channel names."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setDragEnabled(True)
        self.setDefaultDropAction(Qt.CopyAction)
        self.setStyleSheet(f"""
            QListWidget {{
                background: {F1_PANEL_BG}; color: {F1_TEXT};
                border: 1px solid #333; font-size: 10px;
            }}
            QListWidget::item:selected {{ background: {ISC_GREEN}; color: {F1_DARK_BG}; }}
            QListWidget::item:hover    {{ background: #2a2a2a; }}
        """)
        for key, (label, unit) in SNAPSHOT_CHANNELS.items():
            display = f"{label}  [{unit}]" if unit else label
            item = QListWidgetItem(display)
            item.setData(Qt.UserRole, key)
            self.addItem(item)

    def startDrag(self, _actions):
        item = self.currentItem()
        if not item:
            return
        drag = QDrag(self)
        mime = QMimeData()
        mime.setText(item.data(Qt.UserRole))
        drag.setMimeData(mime)
        drag.exec_(Qt.CopyAction)


class DroppablePlotPanel(QFrame):
    """A panel that accepts a channel drop and shows a rolling plot."""
    def __init__(self, idx: int, parent=None):
        super().__init__(parent)
        self._idx     = idx
        self._channel: Optional[str] = None
        self._history: Deque[float]  = deque([0.0] * HISTORY_LEN, maxlen=HISTORY_LEN)
        self.setAcceptDrops(True)
        self.setMinimumHeight(155)
        self._border_idle()

        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.setSpacing(2)

        # Header
        hdr = QHBoxLayout()
        self._title_lbl = QLabel(f"Drop channel here  (panel {idx})")
        self._title_lbl.setStyleSheet("color:#444; font-size:9px; background:transparent; border:none;")
        hdr.addWidget(self._title_lbl)
        hdr.addStretch()
        btn_clr = QPushButton("✕")
        btn_clr.setFixedSize(17, 17)
        btn_clr.setStyleSheet(
            f"QPushButton {{ background:#2a2a2a; color:#666; border:none; font-size:9px; border-radius:2px; }}"
            f"QPushButton:hover {{ background:{F1_ERROR}; color:white; }}"
        )
        btn_clr.clicked.connect(self.clear_channel)
        hdr.addWidget(btn_clr)
        lay.addLayout(hdr)

        # Matplotlib canvas
        fig = Figure(figsize=(3, 1.4), tight_layout=True)
        fig.patch.set_facecolor(F1_PANEL_BG)
        self._ax = fig.add_subplot(111)
        self._line, = self._ax.plot([], [], color=ISC_GREEN, linewidth=1.1)
        self._ax.set_facecolor(F1_DARK_BG)
        self._ax.tick_params(labelsize=7.5, colors='#ffffff' if F1_TEXT == '#e0e0e0' else '#111111')
        self._ax.grid(True, color='#444444' if F1_TEXT == '#e0e0e0' else '#cccccc', alpha=0.35)
        for s in self._ax.spines.values():
            s.set_color('#555555' if F1_TEXT == '#e0e0e0' else '#aaaaaa')
        self._canvas = FigureCanvas(fig)
        lay.addWidget(self._canvas)

        # Current value
        self._val_lbl = QLabel("—")
        self._val_lbl.setAlignment(Qt.AlignCenter)
        self._val_lbl.setStyleSheet(f"color:{ISC_GREEN}; font-size:15px; font-weight:bold; background:transparent; border:none;")
        lay.addWidget(self._val_lbl)

    # ── drag-drop events ──────────────────────────────────────────────────────
    def dragEnterEvent(self, ev):
        if ev.mimeData().hasText():
            self.setStyleSheet(f"QFrame {{ background:{F1_PANEL_BG}; border:2px solid {ISC_GREEN}; border-radius:4px; }}")
            ev.acceptProposedAction()

    def dragLeaveEvent(self, _ev):
        self._border_idle()

    def dropEvent(self, ev):
        self.assign_channel(ev.mimeData().text())
        self.setStyleSheet(f"QFrame {{ background:{F1_PANEL_BG}; border:1px solid {ISC_GREEN}; border-radius:4px; }}")
        ev.acceptProposedAction()

    def _border_idle(self):
        self.setStyleSheet(f"QFrame {{ background:{F1_PANEL_BG}; border:1px dashed #333; border-radius:4px; }}")

    # ── assignment ────────────────────────────────────────────────────────────
    def assign_channel(self, key: str) -> None:
        self._channel = key
        label, unit = SNAPSHOT_CHANNELS.get(key, (key, ''))
        self._unit = unit
        title_txt = f"{label}  [{unit}]" if unit else label
        self._title_lbl.setText(title_txt)
        self._title_lbl.setStyleSheet(
            f"color:{ISC_GREEN}; font-size:9px; font-weight:bold; background:transparent; border:none;")
        # Update y-axis label with unit
        self._ax.set_ylabel(unit, fontsize=6, color='#555')
        self._history = deque([0.0] * HISTORY_LEN, maxlen=HISTORY_LEN)

    def clear_channel(self) -> None:
        self._channel = None
        self._unit = ''
        self._title_lbl.setText(f"Drop channel here  (panel {self._idx})")
        self._title_lbl.setStyleSheet("color:#444; font-size:9px; background:transparent; border:none;")
        self._history  = deque([0.0] * HISTORY_LEN, maxlen=HISTORY_LEN)
        self._val_lbl.setText("—")
        self._ax.cla()
        self._ax.set_facecolor(F1_DARK_BG)
        self._ax.tick_params(labelsize=7.5, colors='#ffffff' if F1_TEXT == '#e0e0e0' else '#111111')
        self._ax.grid(True, color='#444444' if F1_TEXT == '#e0e0e0' else '#cccccc', alpha=0.35)
        for s in self._ax.spines.values():
            s.set_color('#555555' if F1_TEXT == '#e0e0e0' else '#aaaaaa')
        self._line, = self._ax.plot([], [], color=ISC_GREEN, linewidth=1.1)
        self._canvas.draw_idle()
        self._border_idle()

    def update_value(self, snapshot: dict) -> None:
        if not self._channel:
            return
        # Flat key lookup first; then handle per-module array channels
        key = self._channel
        if key in snapshot:
            raw = snapshot[key]
            val = float(raw[0] if isinstance(raw, list) else raw)
        elif key.startswith('vmin_modulo_'):
            idx = int(key[-1])
            arr = snapshot.get('vmin_modulo', [])
            val = float(arr[idx]) if idx < len(arr) else 0.0
        elif key.startswith('vmax_modulo_'):
            idx = int(key[-1])
            arr = snapshot.get('vmax_modulo', [])
            val = float(arr[idx]) if idx < len(arr) else 0.0
        elif key.startswith('tmax_modulo_'):
            idx = int(key[-1])
            arr = snapshot.get('temp_max_modulo', [])
            val = float(arr[idx]) if idx < len(arr) else 0.0
        else:
            val = 0.0
        self._history.append(val)
        y = list(self._history)
        x = list(range(len(y)))
        self._line.set_data(x, y)
        lo, hi = min(y), max(y)
        mg = max((hi - lo) * 0.1, 1.0)
        self._ax.set_xlim(0, HISTORY_LEN)
        self._ax.set_ylim(lo - mg, hi + mg)
        self._canvas.draw_idle()
        unit = getattr(self, '_unit', '')
        self._val_lbl.setText(f"{val:.1f} {unit}" if unit else f"{val:.1f}")


# ══════════════════════════════════════════════════════════════════════════════
#  SETTINGS DIALOG
# ══════════════════════════════════════════════════════════════════════════════
class SettingsDialog(QDialog):
    def __init__(self, parent: 'MainWindow' = None):
        super().__init__(parent)
        self.setWindowTitle("ISCmetrics — Ajustes")
        self.setWindowFlags(self.windowFlags() & ~Qt.WindowContextHelpButtonHint)
        self.setGeometry(200, 200, 440, 360)
        self._p = parent
        if parent:
            self.setPalette(parent.palette())
        
        # High-contrast premium style sheet with embedded vector checkmark
        self.setStyleSheet(f"""
            QDialog {{
                background: {F1_DARK_BG};
                color: {F1_TEXT};
            }}
            QCheckBox {{
                color: {F1_TEXT};
                font-size: 11px;
                spacing: 8px;
            }}
            QCheckBox#chk_demo {{
                color: {ISC_GREEN};
                font-weight: bold;
            }}
            QCheckBox::indicator {{
                width: 14px;
                height: 14px;
                border: 1.5px solid #555555;
                background-color: {F1_MID_BG};
                border-radius: 3px;
            }}
            QCheckBox::indicator:hover {{
                border: 1.5px solid {ISC_GREEN};
            }}
            QCheckBox::indicator:checked {{
                border: 1.5px solid #00c853;
                background-color: {F1_MID_BG};
                image: url(data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHdpZHRoPSIxMiIgaGVpZ2h0PSIxMiIgdmlld0JveD0iMCAwIDI0IDI0IiBmaWxsPSJub25lIiBzdHJva2U9IiMwMGM4NTMiIHN0cm9rZS13aWR0aD0iNCIgc3Ryb2tlLWxpbmVjYXA9InJvdW5kIiBzdHJva2UtbGluZWpvaW49InJvdW5kIj48cG9seWxpbmUgcG9pbnRzPSIyMCA2IDkgMTcgNCAxMiI+PC9wb2x5bGluZT48L3N2Zz4=);
            }}
        """)
        self._build()

    def _build(self):
        g = QGridLayout(self)
        g.setSpacing(10)
        g.setContentsMargins(16, 16, 16, 16)
        ls  = f"color:{F1_TEXT}; font-size:11px; font-weight:bold;"
        ins = self._p.get_input_style()

        g.addWidget(self._lbl("COM Port:", ls), 0, 0)
        self.combo_port = QComboBox(); self.combo_port.setStyleSheet(ins)
        self._refresh_ports(); g.addWidget(self.combo_port, 0, 1)

        g.addWidget(self._lbl("Baud Rate:", ls), 1, 0)
        self.input_baud = QLineEdit(str(self._p.settings.get("baud", rtt.DEFAULT_BAUD))); self.input_baud.setStyleSheet(ins)
        g.addWidget(self.input_baud, 1, 1)

        self.chk_marple = QCheckBox("Upload to Marple Data (cloud)  🔒")
        self.chk_marple.setChecked(self._p.settings.get("use_influx", False))
        self.chk_marple.stateChanged.connect(self._on_marple_toggled)
        g.addWidget(self.chk_marple, 2, 0, 1, 2)

        self.chk_debug = QCheckBox("Enable debug output")
        self.chk_debug.setChecked(self._p.settings.get("debug", False))
        g.addWidget(self.chk_debug, 3, 0, 1, 2)

        self.chk_demo = QCheckBox("Demo mode  (simulated data)")
        self.chk_demo.setObjectName("chk_demo")
        self.chk_demo.setChecked(self._p.settings.get("demo_mode", False))
        g.addWidget(self.chk_demo, 4, 0, 1, 2)

        self.chk_tts = QCheckBox("Enable voice alerts (TTS)")
        self.chk_tts.setChecked(self._p.settings.get("enable_tts", True))
        g.addWidget(self.chk_tts, 5, 0, 1, 2)

        # Alert thresholds
        g.addWidget(self._lbl("Alert Max Temp (°C):", ls), 6, 0)
        self.input_alert_temp = QLineEdit(str(self._p.settings.get("alert_temp_c", 40.0)))
        self.input_alert_temp.setStyleSheet(ins)
        g.addWidget(self.input_alert_temp, 6, 1)

        g.addWidget(self._lbl("Alert Min DC Bus (V):", ls), 7, 0)
        self.input_alert_volt = QLineEdit(str(self._p.settings.get("alert_volt_v", 380.0)))
        self.input_alert_volt.setStyleSheet(ins)
        g.addWidget(self.input_alert_volt, 7, 1)

        g.addWidget(self._lbl("Alert Min Cell (mV):", ls), 8, 0)
        self.input_alert_cell = QLineEdit(str(self._p.settings.get("alert_cell_mv", 3400.0)))
        self.input_alert_cell.setStyleSheet(ins)
        g.addWidget(self.input_alert_cell, 8, 1)

        btn = QPushButton("Apply & Close")
        btn.setStyleSheet(self._p.get_button_style('accent'))
        btn.clicked.connect(self.accept)
        g.addWidget(btn, 9, 0, 1, 2)

    # ── Marple password gate ──────────────────────────────────────────────────
    def _on_marple_toggled(self, state: int):
        """Ask for the Marple API password whenever the checkbox is ticked on."""
        if state == 0:
            return  # unchecking — always allowed
        # Prompt for password (echo mode hidden)
        pwd, ok = QInputDialog.getText(
            self,
            "Marple Upload — Authentication Required",
            "Enter the Marple API password:",
            QLineEdit.Password,
        )
        if not ok:
            # User cancelled → silently uncheck
            self.chk_marple.blockSignals(True)
            self.chk_marple.setChecked(False)
            self.chk_marple.blockSignals(False)
            return
        entered_hash = hashlib.sha256(pwd.encode()).hexdigest()
        if entered_hash != _MARPLE_PASSWORD_HASH:
            QMessageBox.warning(
                self,
                "Access Denied",
                "Incorrect password.\nMarple cloud upload has not been enabled.",
            )
            self.chk_marple.blockSignals(True)
            self.chk_marple.setChecked(False)
            self.chk_marple.blockSignals(False)

    @staticmethod
    def _lbl(t, s):
        l = QLabel(t); l.setStyleSheet(s); return l

    def _refresh_ports(self):
        self.combo_port.clear()
        cur = self._p.settings.get("port")
        for i, (port, desc) in enumerate(rtt.list_serial_ports()):
            self.combo_port.addItem(f"{port}  ({desc})", port)
            if port == cur:
                self.combo_port.setCurrentIndex(i)

    def get_settings(self) -> dict:
        try:    baud = int(self.input_baud.text())
        except: baud = self._p.settings["baud"]
        try:    temp_c = float(self.input_alert_temp.text())
        except: temp_c = self._p.settings.get("alert_temp_c", 40.0)
        try:    volt_v = float(self.input_alert_volt.text())
        except: volt_v = self._p.settings.get("alert_volt_v", 380.0)
        try:    cell_mv = float(self.input_alert_cell.text())
        except: cell_mv = self._p.settings.get("alert_cell_mv", 3400.0)
        return {
            "port":       self.combo_port.currentData(),
            "baud":       baud,
            "use_influx": self.chk_marple.isChecked(),
            "debug":      self.chk_debug.isChecked(),
            "demo_mode":  self.chk_demo.isChecked(),
            "enable_tts": self.chk_tts.isChecked(),
            "alert_temp_c":  temp_c,
            "alert_volt_v":  volt_v,
            "alert_cell_mv": cell_mv,
        }


# ══════════════════════════════════════════════════════════════════════════════
#  SESSION VIEWER
# ══════════════════════════════════════════════════════════════════════════════
class SessionViewerWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ISCmetrics — Session Viewer")
        self.setGeometry(160, 160, 1100, 650)
        self.setStyleSheet(f"background:{F1_DARK_BG}; color:{F1_TEXT};")
        self._build()

    def _build(self):
        h = QHBoxLayout(self)
        # Sidebar
        side = QVBoxLayout()
        hdr = QLabel("Saved Sessions")
        hdr.setStyleSheet(f"color:{ISC_GREEN}; font-weight:bold; font-size:12px;")
        side.addWidget(hdr)
        self._list = QListWidget()
        self._list.setStyleSheet(f"background:{F1_PANEL_BG}; color:{F1_TEXT}; border:1px solid #333;")
        self._list.itemDoubleClicked.connect(self._load)
        side.addWidget(self._list)
        btn = QPushButton("Refresh")
        btn.setStyleSheet(f"background:{F1_MID_BG}; color:{ISC_GREEN}; border:1px solid {ISC_GREEN}; padding:4px 8px;")
        btn.clicked.connect(self._refresh)
        side.addWidget(btn)
        sw = QWidget(); sw.setLayout(side); sw.setFixedWidth(240)
        h.addWidget(sw)
        # Content
        right = QVBoxLayout()
        self._info = QLabel("Double-click a session to load.")
        self._info.setStyleSheet("color:#555; padding:8px;")
        right.addWidget(self._info)
        self._txt = QTextEdit()
        self._txt.setReadOnly(True)
        self._txt.setStyleSheet(
            f"background:{F1_PANEL_BG}; color:{F1_TEXT}; font-family:'Courier New'; font-size:9px;")
        right.addWidget(self._txt)
        rw = QWidget(); rw.setLayout(right)
        h.addWidget(rw)
        self._refresh()

    def _refresh(self):
        self._list.clear()
        for f in rtt.list_excel_sessions():
            item = QListWidgetItem(f.name)
            item.setData(Qt.UserRole, str(f))
            self._list.addItem(item)

    def _load(self, item: QListWidgetItem):
        data = rtt.load_excel_session(Path(item.data(Qt.UserRole)))
        if 'Main' in data:
            df = data['Main']
            self._info.setText(f"{Path(item.data(Qt.UserRole)).name} — {len(df)} rows × {len(df.columns)} cols")
            self._txt.setPlainText(df.to_string(max_rows=60))
        else:
            self._info.setText("Failed to load.")


# ══════════════════════════════════════════════════════════════════════════════
#  POST-RACE WINDOW
# ══════════════════════════════════════════════════════════════════════════════
class PostRaceWindow(QWidget):
    """
    Post-race data injection window.

    Allows the engineer to merge data recorded on the car's micro-SD card
    (GPS coordinates from NMEA log, AMS per-cell temperatures) into an
    existing telemetry session CSV, ready for analysis in Marple / Excel.

    Two panels:
      • GPS Coordinates  — select NMEA log (.nmea / .txt / .log)
      • AMS Temperatures — select AMS SD-card file (format TBD)
    """

    # UTC-offset labels shown in the combo box
    _UTC_OFFSETS = [
        ("UTC+0  (Portugal / UK)",     0),
        ("UTC+1  (Central Europe / CET)", 1),
        ("UTC+2  (Central Europe / CEST — Spain summer)", 2),
        ("UTC+3  (Eastern Europe)", 3),
    ]

    def __init__(self):
        super().__init__()
        self.setWindowTitle("ISCmetrics — Post-Race Analysis")
        self.setGeometry(120, 120, 1050, 680)
        self.setMinimumSize(900, 580)
        self.setStyleSheet(f"background:{F1_DARK_BG}; color:{F1_TEXT};")
        self._session_path: Optional[Path] = None
        self._gps_file_path: Optional[Path] = None
        self._ams_file_path: Optional[Path] = None
        self._build()
        self._refresh_sessions()

    # ── UI construction ───────────────────────────────────────────────────────
    def _build(self):
        root = QVBoxLayout(self)
        root.setSpacing(10)
        root.setContentsMargins(14, 14, 14, 14)

        # ── Title ─────────────────────────────────────────────────────────────
        title = QLabel("POST-RACE DATA INJECTION")
        title.setStyleSheet(
            f"color:{ISC_GREEN}; font-size:16px; font-weight:bold; "
            f"border-bottom:2px solid {ISC_GREEN}; padding-bottom:6px;")
        root.addWidget(title)

        sub = QLabel(
            "Merge data recorded on the car's micro-SD card into an existing session CSV.")
        sub.setStyleSheet("color:#555; font-size:10px;")
        root.addWidget(sub)

        # ── Session selector row ───────────────────────────────────────────────
        sel_row = QHBoxLayout()
        lbl_s = QLabel("Session CSV:")
        lbl_s.setStyleSheet(f"color:{ISC_GREEN}; font-weight:bold; font-size:11px;")
        sel_row.addWidget(lbl_s)

        self._session_combo = QComboBox()
        self._session_combo.setMinimumWidth(460)
        self._session_combo.setStyleSheet(
            f"background:{F1_MID_BG}; color:{F1_TEXT}; border:1px solid {ISC_GREEN}; "
            f"font-size:11px; padding:3px; border-radius:2px;")
        self._session_combo.currentIndexChanged.connect(self._on_session_changed)
        sel_row.addWidget(self._session_combo, stretch=1)

        btn_ref = QPushButton("⟳ Refresh")
        btn_ref.setStyleSheet(
            f"background:{F1_MID_BG}; color:{ISC_GREEN}; border:1px solid {ISC_GREEN}; "
            f"padding:4px 10px; font-size:10px; border-radius:2px;")
        btn_ref.clicked.connect(self._refresh_sessions)
        sel_row.addWidget(btn_ref)
        root.addLayout(sel_row)

        # Session info
        self._session_info = QLabel("No session selected.")
        self._session_info.setStyleSheet("color:#444; font-size:9px; font-family:'Courier New';")
        root.addWidget(self._session_info)

        # ── Two injection panels ───────────────────────────────────────────────
        panels = QHBoxLayout()
        panels.setSpacing(12)
        panels.addWidget(self._build_gps_panel(), stretch=1)
        panels.addWidget(self._build_ams_panel(), stretch=1)
        root.addLayout(panels, stretch=1)

        # ── Log area ──────────────────────────────────────────────────────────
        log_box = QGroupBox("IMPORT LOG")
        log_box.setStyleSheet(
            f"QGroupBox {{ color:{ISC_GREEN}; border:1px solid #222; "
            f"margin-top:10px; font-size:9px; font-weight:bold; }}")
        log_lay = QVBoxLayout(log_box)
        log_lay.setContentsMargins(4, 6, 4, 4)
        self._log = QTextEdit()
        self._log.setReadOnly(True)
        self._log.setMaximumHeight(120)
        self._log.setStyleSheet(
            f"background:{F1_DARK_BG}; color:{F1_TEXT}; "
            f"font-family:'Courier New'; font-size:9px; border:none;")
        log_lay.addWidget(self._log)
        root.addWidget(log_box)

    def _build_gps_panel(self) -> QGroupBox:
        """GPS coordinates injection panel."""
        box = QGroupBox("GPS COORDINATES")
        box.setStyleSheet(
            f"QGroupBox {{ color:{F1_BLUE}; border:1px solid {F1_BLUE}; "
            f"margin-top:14px; font-size:10px; font-weight:bold; }}"
            f"QGroupBox::title {{ subcontrol-origin:margin; "
            f"subcontrol-position:top left; padding:0 6px; "
            f"color:{F1_BLUE}; background:{F1_DARK_BG}; }}")
        v = QVBoxLayout(box)
        v.setSpacing(8)
        v.setContentsMargins(10, 14, 10, 10)

        # Description
        desc = QLabel(
            "Select the NMEA 0183 log file recorded by the on-board GPS module\n"
            "(MTK3339 micro-SD logger). Accepted formats: .nmea, .txt, .log, .csv")
        desc.setStyleSheet(f"color:{F1_TEXT}; font-size:9px;")
        desc.setWordWrap(True)
        v.addWidget(desc)

        # File selector
        file_row = QHBoxLayout()
        self._gps_file_lbl = QLabel("No file selected.")
        self._gps_file_lbl.setStyleSheet("color:#555; font-size:9px; font-family:'Courier New';")
        file_row.addWidget(self._gps_file_lbl, stretch=1)

        btn_browse = QPushButton("Browse…")
        btn_browse.setStyleSheet(
            f"background:{F1_MID_BG}; color:{F1_BLUE}; border:1px solid {F1_BLUE}; "
            f"padding:4px 10px; font-size:10px; border-radius:2px;")
        btn_browse.clicked.connect(self._browse_gps)
        file_row.addWidget(btn_browse)
        v.addLayout(file_row)

        # UTC offset
        off_row = QHBoxLayout()
        off_lbl = QLabel("GPS time zone:")
        off_lbl.setStyleSheet(f"color:{F1_TEXT}; font-size:10px; font-weight:bold;")
        off_row.addWidget(off_lbl)
        self._utc_offset_combo = QComboBox()
        self._utc_offset_combo.setStyleSheet(
            f"background:{F1_MID_BG}; color:{F1_TEXT}; border:1px solid #333; "
            f"font-size:9px; padding:3px; border-radius:2px;")
        for label, _ in self._UTC_OFFSETS:
            self._utc_offset_combo.addItem(label)
        self._utc_offset_combo.setCurrentIndex(2)   # default UTC+2 (Spain CEST)
        off_row.addWidget(self._utc_offset_combo, stretch=1)
        v.addLayout(off_row)

        v.addStretch()

        # Import button + status
        self._gps_status = QLabel("Ready.")
        self._gps_status.setStyleSheet("color:#555; font-size:9px; font-family:'Courier New';")
        self._gps_status.setWordWrap(True)
        v.addWidget(self._gps_status)

        btn_import = QPushButton("⬇  Import GPS Data")
        btn_import.setStyleSheet(
            f"QPushButton {{ background:{F1_BLUE}; color:{F1_DARK_BG}; border:none; "
            f"border-radius:3px; padding:7px 14px; font-size:11px; font-weight:bold; }}"
            f"QPushButton:hover {{ background:#60a5fa; }}"
            f"QPushButton:disabled {{ background:#1e3a5f; color:#444; }}")
        btn_import.clicked.connect(self._import_gps)
        v.addWidget(btn_import)

        return box

    def _build_ams_panel(self) -> QGroupBox:
        """AMS temperature injection panel."""
        box = QGroupBox("AMS TEMPERATURES")
        box.setStyleSheet(
            f"QGroupBox {{ color:{F1_WARNING}; border:1px solid {F1_WARNING}; "
            f"margin-top:14px; font-size:10px; font-weight:bold; }}"
            f"QGroupBox::title {{ subcontrol-origin:margin; "
            f"subcontrol-position:top left; padding:0 6px; "
            f"color:{F1_WARNING}; background:{F1_DARK_BG}; }}")
        v = QVBoxLayout(box)
        v.setSpacing(8)
        v.setContentsMargins(10, 14, 10, 10)

        # Description
        desc = QLabel(
            "Select the AMS temperature log file from the micro-SD card.\n"
            "When merged, adds 95 columns (ams_t_mod{m}_cell{c}) to the session CSV.")
        desc.setStyleSheet(f"color:{F1_TEXT}; font-size:9px;")
        desc.setWordWrap(True)
        v.addWidget(desc)

        # File selector
        file_row = QHBoxLayout()
        self._ams_file_lbl = QLabel("No file selected.")
        self._ams_file_lbl.setStyleSheet("color:#555; font-size:9px; font-family:'Courier New';")
        file_row.addWidget(self._ams_file_lbl, stretch=1)

        btn_browse = QPushButton("Browse…")
        btn_browse.setStyleSheet(
            f"background:{F1_MID_BG}; color:{F1_WARNING}; border:1px solid {F1_WARNING}; "
            f"padding:4px 10px; font-size:10px; border-radius:2px;")
        btn_browse.clicked.connect(self._browse_ams)
        file_row.addWidget(btn_browse)
        v.addLayout(file_row)

        v.addStretch()

        # Status
        self._ams_status = QLabel("Ready.")
        self._ams_status.setStyleSheet("color:#555; font-size:9px; font-family:'Courier New';")
        self._ams_status.setWordWrap(True)
        v.addWidget(self._ams_status)

        btn_import = QPushButton("⬇  Import AMS Temperatures")
        btn_import.setEnabled(True)
        btn_import.setStyleSheet(
            f"QPushButton {{ background:{F1_WARNING}; color:{F1_DARK_BG}; border:none; "
            f"border-radius:3px; padding:7px 14px; font-size:11px; font-weight:bold; }}"
            f"QPushButton:hover {{ background:#fbbf24; }}"
            f"QPushButton:disabled {{ background:#5e3a00; color:#555; }}")
        btn_import.clicked.connect(self._import_ams)
        v.addWidget(btn_import)

        return box

    # ── Session list helpers ──────────────────────────────────────────────────
    def _refresh_sessions(self):
        self._session_combo.clear()
        sessions = rtt.list_excel_sessions()
        if not sessions:
            self._session_combo.addItem("(no sessions found)", None)
            self._session_path = None
            self._session_info.setText("No session files found in logs/.")
            return
        for s in sessions:
            self._session_combo.addItem(s.name, str(s))
        self._on_session_changed(0)

    def _on_session_changed(self, idx: int):
        path_str = self._session_combo.currentData()
        if not path_str:
            self._session_path = None
            self._session_info.setText("")
            return
        p = Path(path_str)
        self._session_path = p
        try:
            import os
            size_kb = p.stat().st_size / 1024
            # Count rows quickly
            with open(p, 'r', errors='ignore') as f:
                rows = sum(1 for _ in f) - 1  # minus header
            self._session_info.setText(
                f"{p}   |   {rows} rows   |   {size_kb:.1f} KB")
        except Exception:
            self._session_info.setText(str(p))

    # ── File browse handlers ──────────────────────────────────────────────────
    def _browse_gps(self):
        from PyQt5.QtWidgets import QFileDialog
        path, _ = QFileDialog.getOpenFileName(
            self, "Select NMEA GPS log file", "",
            "NMEA / Text files (*.nmea *.txt *.log *.csv);;All files (*.*)")
        if path:
            self._gps_file_path = Path(path)
            self._gps_file_lbl.setText(self._gps_file_path.name)
            self._gps_file_lbl.setStyleSheet(
                f"color:{F1_BLUE}; font-size:9px; font-family:'Courier New';")
            self._gps_status.setText("File selected — click Import to merge.")
            self._gps_status.setStyleSheet(
                f"color:{ISC_GREEN}; font-size:9px; font-family:'Courier New';")

    def _browse_ams(self):
        from PyQt5.QtWidgets import QFileDialog
        path, _ = QFileDialog.getOpenFileName(
            self, "Select AMS temperature log file", "",
            "CSV / Text files (*.csv *.txt *.log);;All files (*.*)")
        if path:
            self._ams_file_path = Path(path)
            self._ams_file_lbl.setText(self._ams_file_path.name)
            self._ams_file_lbl.setStyleSheet(
                f"color:{F1_WARNING}; font-size:9px; font-family:'Courier New';")
            self._ams_status.setText("File selected — click Import to merge.")
            self._ams_status.setStyleSheet(
                f"color:{ISC_GREEN}; font-size:9px; font-family:'Courier New';")

    # ── Import handlers ───────────────────────────────────────────────────────
    def _import_gps(self):
        if not self._session_path:
            QMessageBox.warning(self, "No session", "Please select a session CSV first.")
            return
        if not self._gps_file_path:
            QMessageBox.warning(self, "No GPS file", "Please browse to a GPS NMEA log file first.")
            return

        utc_off = self._UTC_OFFSETS[self._utc_offset_combo.currentIndex()][1]
        self._log_append(
            f"[GPS] Merging {self._gps_file_path.name} "
            f"→ {self._session_path.name}  (UTC+{utc_off})")

        self._gps_status.setText("Merging… please wait.")
        self._gps_status.setStyleSheet(
            f"color:{F1_WARNING}; font-size:9px; font-family:'Courier New';")
        QApplication.processEvents()

        ok, msg, new_path = rtt.merge_gps_into_session(
            self._session_path, self._gps_file_path, utc_offset_hours=utc_off)

        if ok:
            self._session_path = new_path
            self._gps_status.setText(f"✓  {msg}")
            self._gps_status.setStyleSheet(
                f"color:{ISC_GREEN}; font-size:9px; font-family:'Courier New';")
            self._log_append(f"[GPS] ✓ {msg}")
            # Refresh session info (size/rows may have changed)
            self._on_session_changed(self._session_combo.currentIndex())
        else:
            self._gps_status.setText(f"✗  {msg}")
            self._gps_status.setStyleSheet(
                f"color:{F1_ERROR}; font-size:9px; font-family:'Courier New';")
            self._log_append(f"[GPS] ✗ {msg}")

    def _import_ams(self):
        if not self._session_path:
            QMessageBox.warning(self, "No session", "Please select a session CSV first.")
            return
        if not self._ams_file_path:
            QMessageBox.warning(self, "No AMS file", "Please browse to an AMS log file first.")
            return

        self._log_append(f"[AMS] Merging {self._ams_file_path.name} → {self._session_path.name}")
        self._ams_status.setText("Merging… please wait.")
        self._ams_status.setStyleSheet(
            f"color:{F1_WARNING}; font-size:9px; font-family:'Courier New';")
        QApplication.processEvents()

        ok, msg, new_path = rtt.merge_ams_temps_into_session(self._session_path, self._ams_file_path)

        if ok:
            self._session_path = new_path
            self._ams_status.setText(f"✓  {msg}")
            self._ams_status.setStyleSheet(
                f"color:{ISC_GREEN}; font-size:9px; font-family:'Courier New';")
            self._log_append(f"[AMS] ✓ {msg}")
            self._on_session_changed(self._session_combo.currentIndex())
        else:
            self._ams_status.setText(f"✗  {msg}")
            self._ams_status.setStyleSheet(
                f"color:{F1_ERROR}; font-size:9px; font-family:'Courier New';")
            self._log_append(f"[AMS] ✗ {msg}")

    def _log_append(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S")
        self._log.append(f"[{ts}] {msg}")

    def closeEvent(self, event):
        # Automatically integrate selected files when closing the post-race window
        if self._session_path:
            merged_any = False
            gps_msg = ""
            ams_msg = ""
            
            if self._gps_file_path:
                utc_off = self._UTC_OFFSETS[self._utc_offset_combo.currentIndex()][1]
                ok, msg, new_path = rtt.merge_gps_into_session(
                    self._session_path, self._gps_file_path, utc_offset_hours=utc_off)
                if ok:
                    self._session_path = new_path
                    merged_any = True
                    gps_msg = f"GPS: {msg}\n"
                    self._log_append(f"[AUTO-MERGE] GPS integrated: {msg}")
                else:
                    self._log_append(f"[AUTO-MERGE] GPS failed: {msg}")
            
            if self._ams_file_path:
                ok, msg, new_path = rtt.merge_ams_temps_into_session(
                    self._session_path, self._ams_file_path)
                if ok:
                    self._session_path = new_path
                    merged_any = True
                    ams_msg = f"AMS: {msg}\n"
                    self._log_append(f"[AUTO-MERGE] AMS integrated: {msg}")
                else:
                    self._log_append(f"[AUTO-MERGE] AMS failed: {msg}")
                    
            if merged_any:
                QMessageBox.information(
                    self, "Post-Race Integration Complete",
                    f"Selected files have been integrated into: {self._session_path.name}\n\n"
                    f"{gps_msg}{ams_msg}")
        event.accept()


# ══════════════════════════════════════════════════════════════════════════════
#  MAIN WINDOW
# ══════════════════════════════════════════════════════════════════════════════
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(f"ISCmetrics v{APP_VERSION} — Formula Student Telemetry")
        self.setGeometry(40, 40, 1600, 960)
        self.theme_mode   = "dark"

        self.settings     = current_settings.copy()
        self._load_settings_from_file()
        self.demo_mode    = self.settings["demo_mode"]
        self.is_receiving = False
        self.rx_thread: Optional[threading.Thread] = None
        self._post_race_win: Optional["PostRaceWindow"] = None
        self._settings_dlg: Optional[SettingsDialog]    = None
        self._log: Optional[QTextEdit] = None

        icon = Path(__file__).resolve().parent / "isc_logo.ico"
        if icon.exists():
            self.setWindowIcon(QIcon(str(icon)))

        self._build_ui()
        self._apply_theme()
        self._update_widget_thresholds()

        self._timer = QTimer()
        self._timer.timeout.connect(self._update)
        self._timer.start(400)

        signaler.log_message.connect(self._log_append)
        signaler.update_detected.connect(self._show_update_banner)
        self._log_append(f"ISCmetrics v{APP_VERSION} ready.")

        # Kick off a background update check (non-blocking)
        threading.Thread(target=self._check_for_update, daemon=True).start()

    def _check_for_update(self):
        """Background thread: query GitHub for the latest release tag."""
        if not _REQUESTS_OK:
            logger.warning("[UPDATE] requests module is not available. Cannot check for updates.")
            return
        try:
            logger.info("[UPDATE] Checking for updates at %s...", _RELEASES_URL)
            resp = _requests.get(_RELEASES_URL, timeout=5,
                                 headers={"Accept": "application/vnd.github+json"})
            if resp.status_code != 200:
                logger.warning("[UPDATE] Failed check: HTTP status %d", resp.status_code)
                return
            tag = resp.json().get("tag_name", "").lstrip("v")
            if not tag:
                logger.warning("[UPDATE] Failed check: No tag_name in response")
                return
            
            def _ver_tuple(s):
                try:    return tuple(int(x) for x in s.split("."))
                except: return (0, 0, 0)
                
            logger.info("[UPDATE] Latest version: %s (Current version: %s)", tag, APP_VERSION)
            if _ver_tuple(tag) > _ver_tuple(APP_VERSION):
                # Find download URL for Windows setup EXE
                download_url = None
                for asset in resp.json().get("assets", []):
                    name = asset.get("name", "")
                    if name.endswith(".exe") and "Setup" in name:
                        download_url = asset.get("browser_download_url")
                        break
                # Fallback if no specific setup EXE is found
                if not download_url:
                    download_url = resp.json().get("html_url", _RELEASES_PAGE)
                
                # Emit signal to thread-safely show the banner on the main GUI thread
                signaler.update_detected.emit(tag, download_url)
        except Exception as e:
            logger.warning("[UPDATE] Check failed with exception: %s", e)

    def _show_update_banner(self, new_version: str, download_url: str):
        """Show a non-blocking update banner at the top of the window."""
        if hasattr(self, "_update_banner") and self._update_banner is not None:
            return  # already shown
        banner = QFrame(self)
        banner.setStyleSheet(
            f"background:#1a3a1a; border-bottom:2px solid {ISC_GREEN};"
        )
        bh = QHBoxLayout(banner)
        bh.setContentsMargins(12, 6, 12, 6)
        lbl = QLabel(f"🔄  ISCmetrics v{new_version} is available — you have v{APP_VERSION}")
        lbl.setStyleSheet(f"color:{ISC_GREEN}; font-size:11px; font-weight:bold;")
        btn_dl = QPushButton("Download Update")
        btn_dl.setStyleSheet(self.get_button_style("accent"))
        btn_dl.setFixedWidth(140)
        btn_dl.clicked.connect(lambda: self._start_automatic_update(new_version, download_url))
        btn_close = QPushButton("✕")
        btn_close.setStyleSheet(self.get_button_style())
        btn_close.setFixedWidth(28)
        btn_close.clicked.connect(lambda: self._dismiss_update_banner())
        bh.addWidget(lbl)
        bh.addStretch()
        bh.addWidget(btn_dl)
        bh.addWidget(btn_close)
        # Insert banner at the top of the central widget's layout
        cw = self.centralWidget()
        if cw and cw.layout():
            cw.layout().insertWidget(0, banner)
        banner.show()
        self._update_banner = banner
        self._log_append(f"[UPDATE] ISCmetrics v{new_version} available. Klik en 'Download Update' para instalar.")

    def _start_automatic_update(self, new_version: str, download_url: str):
        """Start the background download of the setup installer and show a progress dialog."""
        from PyQt5.QtWidgets import QProgressDialog
        
        self._updater_dlg = QProgressDialog(f"Descargando actualización v{new_version}...", "Cancelar", 0, 100, self)
        self._updater_dlg.setWindowTitle("Actualización de ISCmetrics")
        self._updater_dlg.setWindowModality(Qt.WindowModal)
        self._updater_dlg.setMinimumDuration(0)
        self._updater_dlg.setValue(0)
        
        self._updater_cancelled = False
        self._updater_dlg.canceled.connect(self._cancel_update)
        
        # Connect worker thread signals to main thread slots
        signaler.download_progress.connect(self._on_update_download_progress)
        signaler.download_finished.connect(self._on_update_download_finished)
        
        def _download_worker():
            import tempfile
            import os
            try:
                resp = _requests.get(download_url, stream=True, timeout=15)
                if resp.status_code != 200:
                    signaler.download_finished.emit(f"ERR: HTTP status {resp.status_code}")
                    return
                
                total_size = int(resp.headers.get('content-length', 0))
                if total_size <= 0:
                    signaler.download_finished.emit("ERR: Invalid content length")
                    return
                
                temp_dir = tempfile.gettempdir()
                dest_path = os.path.join(temp_dir, f"ISCmetrics_Setup_v{new_version}.exe")
                
                downloaded = 0
                with open(dest_path, 'wb') as f:
                    for chunk in resp.iter_content(chunk_size=131072):
                        if self._updater_cancelled:
                            return
                        if chunk:
                            f.write(chunk)
                            downloaded += len(chunk)
                            pct = int((downloaded / total_size) * 100)
                            signaler.download_progress.emit(pct)
                
                if not self._updater_cancelled:
                    signaler.download_finished.emit(dest_path)
            except Exception as e:
                signaler.download_finished.emit(f"ERR: {e}")
                
        threading.Thread(target=_download_worker, daemon=True).start()

    def _cancel_update(self):
        self._updater_cancelled = True
        self._log_append("[UPDATE] Descarga de la actualización cancelada.")

    def _on_update_download_progress(self, val: int):
        if hasattr(self, "_updater_dlg") and self._updater_dlg:
            self._updater_dlg.setValue(val)

    def _on_update_download_finished(self, result: str):
        # Disconnect signals to prevent any cross-triggering
        try:
            signaler.download_progress.disconnect(self._on_update_download_progress)
            signaler.download_finished.disconnect(self._on_update_download_finished)
        except:
            pass
            
        if hasattr(self, "_updater_dlg") and self._updater_dlg:
            self._updater_dlg.close()
            self._updater_dlg = None
            
        if result.startswith("ERR:"):
            err_msg = result[4:]
            QMessageBox.critical(self, "Error de descarga", 
                                 f"No se pudo descargar la actualización automáticamente:\n{err_msg}\n\nPor favor, inténtelo de nuevo o instálela desde la web.")
            webbrowser.open(_RELEASES_PAGE)
        else:
            self._log_append(f"[UPDATE] Descarga de actualización completada. Iniciando instalador: {result}")
            try:
                import os
                # Execute the installer asynchronously
                os.startfile(result)
                # Close the application immediately so the installer can replace the files
                self.close()
            except Exception as install_err:
                QMessageBox.critical(self, "Error de instalación", 
                                     f"No se pudo iniciar el instalador descargado:\n{install_err}\n\nUbicación del archivo: {result}")

    def _dismiss_update_banner(self):
        if hasattr(self, "_update_banner") and self._update_banner:
            self._update_banner.hide()
            self._update_banner.setParent(None)
            self._update_banner = None

    # ── style helpers ─────────────────────────────────────────────────────────
    def get_input_style(self) -> str:
        return (f"background:{F1_MID_BG}; color:{F1_TEXT}; "
                f"border:1px solid {ISC_GREEN}; font-size:13px; padding:4px; border-radius:2px;")

    def get_button_style(self, v: str = 'default') -> str:
        if v == 'accent':
            bg, fg, br, hbg = ISC_GREEN, F1_DARK_BG, 'none', '#009a00'
        else:
            bg, fg, br, hbg = F1_MID_BG, ISC_GREEN, f'1px solid {ISC_GREEN}', ISC_GREEN
        return f"""
            QPushButton {{ background:{bg}; color:{fg}; border:{br};
                           border-radius:3px; padding:5px 11px;
                           font-size:11px; font-weight:bold; }}
            QPushButton:hover {{ background:{hbg}; color:{F1_DARK_BG}; }}
            QPushButton:disabled {{ background:#222; color:#444; border:1px solid #333; }}
        """

    @staticmethod
    def _lbl(text: str, style: str) -> QLabel:
        l = QLabel(text); l.setStyleSheet(style); return l

    @staticmethod
    def _vsep() -> QFrame:
        f = QFrame(); f.setFrameShape(QFrame.VLine)
        f.setStyleSheet("color:#2a2a2a; max-width:1px;"); return f

    def _apply_theme(self):
        pal = QPalette()
        pal.setColor(QPalette.Window,          QColor(F1_DARK_BG))
        pal.setColor(QPalette.WindowText,      QColor(F1_TEXT))
        pal.setColor(QPalette.Base,            QColor(F1_MID_BG))
        pal.setColor(QPalette.Text,            QColor(F1_TEXT))
        pal.setColor(QPalette.Button,          QColor(F1_MID_BG))
        pal.setColor(QPalette.ButtonText,      QColor(ISC_GREEN))
        pal.setColor(QPalette.Highlight,       QColor(ISC_GREEN))
        pal.setColor(QPalette.HighlightedText, QColor(F1_DARK_BG))
        self.setPalette(pal)
        self.setStyleSheet(f"""
            QMainWindow {{ background:{F1_DARK_BG}; }}
            QTabWidget::pane {{ border:1px solid {ISC_GREEN}; background:{F1_DARK_BG}; }}
            QTabBar::tab {{
                background:{F1_MID_BG}; color:{F1_TEXT};
                padding:8px 22px; margin-right:1px;
                border-top:1px solid {ISC_GREEN};
                border-left:1px solid {ISC_GREEN};
                border-right:1px solid {ISC_GREEN};
                font-size:11px;
            }}
            QTabBar::tab:selected {{ background:{ISC_GREEN}; color:{F1_DARK_BG}; font-weight:bold; }}
            QLabel {{ color:{F1_TEXT}; }}
            QGroupBox {{
                color:{ISC_GREEN}; border:1px solid #252525;
                margin-top:14px; font-size:9px; font-weight:bold;
            }}
            QGroupBox::title {{
                subcontrol-origin:margin; subcontrol-position:top left;
                padding:0 5px; color:{ISC_GREEN}; background:{F1_DARK_BG};
            }}
            QScrollBar:vertical {{ background:{F1_DARK_BG}; width:7px; }}
            QScrollBar::handle:vertical {{ background:#333; border-radius:3px; }}
        """)

    # ── UI construction ───────────────────────────────────────────────────────
    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        vbox = QVBoxLayout(root)
        vbox.setSpacing(4)
        vbox.setContentsMargins(8, 8, 8, 8)

        vbox.addWidget(self._make_top_bar())

        self._alert_banner = AlertBanner()
        vbox.addWidget(self._alert_banner)

        self._tabs = QTabWidget()
        self._tabs.setFont(QFont("Segoe UI", 10, QFont.Bold))
        self._tabs.addTab(self._tab_overview(),   "Overview")
        self._tabs.addTab(self._tab_customize(),  "Customize")
        self._tabs.addTab(self._tab_powertrain(), "Powertrain")
        self._tabs.addTab(self._tab_dynamics(),   "Dynamics")
        self._tabs.addTab(self._tab_post_race(),  "Post-Race")
        vbox.addWidget(self._tabs, stretch=10)

        vbox.addWidget(self._make_log_strip(), stretch=1)

        attr = QLabel("Andrés Sánchez de Ágreda © 2025/2026  —  ICAI Racing Formula Student")
        attr.setAlignment(Qt.AlignCenter)
        attr.setStyleSheet("color:#252525; font-size:8px;")
        vbox.addWidget(attr)

    # ── Top bar ───────────────────────────────────────────────────────────────
    def _make_top_bar(self) -> QFrame:
        bar = QFrame()
        self._top_bar = bar
        bar.setStyleSheet(f"QFrame {{ background:{F1_MID_BG}; border-radius:4px; }}")
        bar.setFixedHeight(98)
        h = QHBoxLayout(bar)
        h.setSpacing(10)
        h.setContentsMargins(8, 5, 8, 5)

        # Logo
        lf = QFrame(); lf.setStyleSheet("background:transparent; border:none;")
        ll = QHBoxLayout(lf); ll.setContentsMargins(0,0,0,0); ll.setSpacing(6)
        logo_lbl = QLabel()
        logo_path = Path(__file__).resolve().parent / "isc_logo.png"
        if logo_path.exists():
            px = QPixmap(str(logo_path))
            if not px.isNull():
                logo_lbl.setPixmap(px.scaled(52, 52, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        if not logo_lbl.pixmap() or logo_lbl.pixmap().isNull():
            logo_lbl.setText("ISC")
            logo_lbl.setStyleSheet(f"color:{ISC_GREEN}; font-size:20px; font-weight:bold;")
        ll.addWidget(logo_lbl)
        vn = QVBoxLayout()
        vn.addWidget(self._lbl("ISCmetrics", f"color:{ISC_GREEN}; font-size:15px; font-weight:bold;"))
        vn.addWidget(self._lbl(f"Formula Student Telemetry v{APP_VERSION}", "color:#666; font-size:9px;"))
        ll.addLayout(vn)
        h.addWidget(lf)
        h.addWidget(self._vsep())

        # Pilot / Circuit
        fg = QGridLayout(); fg.setSpacing(4)
        ls = f"color:{ISC_GREEN}; font-size:11px; font-weight:bold;"
        ins = self.get_input_style()
        fg.addWidget(self._lbl("PILOT:",   ls), 0, 0)
        self._inp_pilot = QLineEdit("Piloto_Test"); self._inp_pilot.setStyleSheet(ins)
        self._inp_pilot.setMinimumWidth(170); fg.addWidget(self._inp_pilot, 0, 1)
        fg.addWidget(self._lbl("CIRCUIT:", ls), 1, 0)
        self._inp_circuit = QLineEdit("Circuito_Test"); self._inp_circuit.setStyleSheet(ins)
        self._inp_circuit.setMinimumWidth(170); fg.addWidget(self._inp_circuit, 1, 1)
        h.addLayout(fg)
        h.addWidget(self._vsep())

        # Mini metrics
        mv = QVBoxLayout()
        self._mini_rpm  = QLabel("0 rpm");  self._mini_rpm.setStyleSheet("color:#777; font-size:9px;")
        self._mini_vbus = QLabel("0 V");    self._mini_vbus.setStyleSheet("color:#777; font-size:9px;")
        self._mini_temp = QLabel("0 °C");   self._mini_temp.setStyleSheet("color:#777; font-size:9px;")
        for l in (self._mini_rpm, self._mini_vbus, self._mini_temp): mv.addWidget(l)
        h.addLayout(mv)
        h.addStretch()

        # Status badge
        self._status_lbl = QLabel("IDLE")
        self._status_lbl.setAlignment(Qt.AlignCenter)
        self._status_lbl.setFixedWidth(88)
        self._status_lbl.setStyleSheet(
            f"background:{F1_MID_BG}; color:#555; font-size:14px; font-weight:bold;"
            f"padding:8px 10px; border:2px solid #333; border-radius:4px;")
        h.addWidget(self._status_lbl)
        h.addWidget(self._vsep())

        # Buttons
        bg = QGridLayout(); bg.setSpacing(4)
        self._btn_start = QPushButton("START")
        self._btn_start.setStyleSheet(self.get_button_style('accent'))
        self._btn_start.clicked.connect(self._start)
        bg.addWidget(self._btn_start, 0, 0)

        self._btn_stop = QPushButton("STOP")
        self._btn_stop.setStyleSheet(self.get_button_style())
        self._btn_stop.setEnabled(False)
        self._btn_stop.clicked.connect(self._stop)
        bg.addWidget(self._btn_stop, 0, 1)

        self._btn_settings = QPushButton("Settings")
        self._btn_settings.setStyleSheet(self.get_button_style())
        self._btn_settings.clicked.connect(self._open_settings)
        bg.addWidget(self._btn_settings, 1, 0)

        btn_post = QPushButton("Post-Race")
        btn_post.setStyleSheet(self.get_button_style())
        btn_post.clicked.connect(self._open_post_race)
        bg.addWidget(btn_post, 1, 1)

        self._btn_theme = QPushButton("☀️  Light" if self.theme_mode == "dark" else "🌙  Dark")
        self._btn_theme.setStyleSheet(self.get_button_style())
        self._btn_theme.clicked.connect(self._toggle_theme)
        bg.addWidget(self._btn_theme, 2, 0, 1, 2)

        h.addLayout(bg)
        return bar

    # ── Tab 1 — Overview ──────────────────────────────────────────────────────
    def _tab_overview(self) -> QWidget:
        w = QWidget()
        v = QVBoxLayout(w); v.setSpacing(6); v.setContentsMargins(8,8,8,8)

        # Metric cards row (10 cards)
        cr = QHBoxLayout(); cr.setSpacing(6)
        self._ov_rpm     = MetricCard("RPM",         "rpm",  ISC_GREEN)
        self._ov_vbus    = MetricCard("DC BUS",      "V",    ISC_GREEN)
        self._ov_tm2     = MetricCard("MOTOR 2 TEMP","ºC", F1_WARNING)
        self._ov_temp    = MetricCard("MAX TEMP",    "ºC", F1_ERROR)
        self._ov_soc     = MetricCard("SOC (VTC6)",  "%",    F1_BLUE)
        self._ov_time_rem= MetricCard("EST REMAINING","",    ISC_GREEN)
        self._ov_torque  = MetricCard("TORQUE REQ",  "%",    ISC_GREEN)
        self._ov_cur     = MetricCard("MOTOR I",     "A",    F1_PURPLE)
        self._ov_vcell   = MetricCard("MIN CELL",    "mV",   F1_WARNING)
        self._ov_state   = MetricCard("INV STATE",   "",     ISC_GREEN)
        for c in (self._ov_rpm, self._ov_vbus, self._ov_tm2, self._ov_temp, self._ov_soc,
                  self._ov_time_rem, self._ov_torque, self._ov_cur, self._ov_vcell, self._ov_state):
            cr.addWidget(c)
        v.addLayout(cr, stretch=2)

        # Rolling plots row: RPM | DC Bus | Max Temp | Throttle+Brake overlay
        pr = QHBoxLayout(); pr.setSpacing(6)
        self._ov_plot_rpm  = MplCanvas("Motor Speed  [RPM]",      ISC_GREEN)
        self._ov_plot_vbus = MplCanvas("DC Bus Voltage  [V]",     F1_WARNING)
        self._ov_plot_temp = MplCanvas("Max Battery Temp  [ºC]",F1_ERROR)

        # Throttle + Brake dual-line canvas
        self._ov_thr_hist: Deque[float] = deque([0.0] * HISTORY_LEN, maxlen=HISTORY_LEN)
        self._ov_brk_hist: Deque[float] = deque([0.0] * HISTORY_LEN, maxlen=HISTORY_LEN)
        fig_tb = Figure(figsize=(4, 2), tight_layout=True)
        fig_tb.patch.set_facecolor(F1_PANEL_BG)
        self._ax_tb = fig_tb.add_subplot(111)
        self._line_thr, = self._ax_tb.plot([], [], color=ISC_GREEN,  linewidth=1.4, label='Throttle [%]')
        self._line_brk, = self._ax_tb.plot([], [], color=F1_ERROR,   linewidth=1.4, label='Brake [%]')
        self._ax_tb.set_facecolor(F1_DARK_BG)
        self._ax_tb.set_title('Throttle / Brake  [%]', color=F1_WARNING,
                               fontsize=8, fontweight='bold', pad=2)
        self._ax_tb.set_ylim(-5, 105)
        self._ax_tb.set_xlim(0, HISTORY_LEN)
        self._ax_tb.tick_params(labelsize=7.5, colors='#ffffff' if F1_TEXT == '#e0e0e0' else '#111111')
        self._ax_tb.grid(True, color='#444444' if F1_TEXT == '#e0e0e0' else '#cccccc', alpha=0.35)
        self._ax_tb.legend(fontsize=7, loc='upper left',
                           facecolor=F1_PANEL_BG, labelcolor=F1_TEXT,
                           edgecolor='#555555' if F1_TEXT == '#e0e0e0' else '#aaaaaa', framealpha=0.8)
        for sp in self._ax_tb.spines.values(): sp.set_color('#555555' if F1_TEXT == '#e0e0e0' else '#aaaaaa')
        self._canvas_tb = FigureCanvas(fig_tb)
        tb_widget = QWidget()
        tb_lay = QVBoxLayout(tb_widget); tb_lay.setContentsMargins(0,0,0,0)
        tb_lay.addWidget(self._canvas_tb)

        pr.addWidget(self._ov_plot_rpm)
        pr.addWidget(self._ov_plot_vbus)
        pr.addWidget(self._ov_plot_temp)
        pr.addWidget(tb_widget)
        v.addLayout(pr, stretch=5)

        # State / indicator row
        ir = QHBoxLayout(); ir.setSpacing(16)
        self._ind_precharge = QLabel("● PRECHARGE")
        self._ind_inv_ok    = QLabel("● INV OK")
        self._ind_ams       = QLabel("● AMS")
        self._lbl_seq       = QLabel("SEQ: —")
        self._lbl_tick      = QLabel("TICK: —")
        # Signal-strength bar widget + percentage label
        self._signal_bars   = SignalBarsWidget()
        self._lbl_lqi       = QLabel("100%")
        
        # Live decoded fault label
        self._lbl_inv_errors = QLabel("")
        self._lbl_inv_errors.setStyleSheet("color:#ff3333; font-size:10px; font-weight:bold;")
        self._lbl_inv_errors.hide()

        for l in (self._ind_precharge, self._ind_inv_ok, self._ind_ams):
            l.setStyleSheet("color:#333; font-size:10px; font-weight:bold;")
        for l in (self._lbl_seq, self._lbl_tick):
            l.setStyleSheet("color:#444; font-size:9px; font-family:'Courier New';")
        self._lbl_lqi.setStyleSheet("color:#00c853; font-size:9px; font-family:'Courier New';")
        for w2 in (self._ind_precharge, self._ind_inv_ok, self._ind_ams,
                   self._lbl_seq, self._lbl_tick, self._signal_bars, self._lbl_lqi,
                   self._lbl_inv_errors):
            ir.addWidget(w2)
        ir.addStretch()
        v.addLayout(ir, stretch=1)
        return w

    # ── Tab 2 — Customize ─────────────────────────────────────────────────────
    def _tab_customize(self) -> QWidget:
        w = QWidget()
        h = QHBoxLayout(w); h.setSpacing(6); h.setContentsMargins(8,8,8,8)

        # Left: channel list
        lv = QVBoxLayout()
        lv.addWidget(self._lbl("Available Channels",
                                f"color:{ISC_GREEN}; font-size:11px; font-weight:bold;"))
        lv.addWidget(self._lbl("Drag onto a panel to plot it.",
                                "color:#444; font-size:9px;"))
        self._ch_list = ChannelListWidget()
        lv.addWidget(self._ch_list)
        lw = QWidget(); lw.setLayout(lv); lw.setFixedWidth(196)
        h.addWidget(lw)

        # Right: 2 × 3 drop panels
        self._drop_panels: List[DroppablePlotPanel] = []
        gw = QWidget()
        grid = QGridLayout(gw); grid.setSpacing(6)
        for idx in range(6):
            p = DroppablePlotPanel(idx)
            self._drop_panels.append(p)
            grid.addWidget(p, idx // 3, idx % 3)
        h.addWidget(gw)
        return w

    # ── Tab 3 — Powertrain ────────────────────────────────────────────────────
    def _tab_powertrain(self) -> QWidget:
        w = QWidget()
        v = QVBoxLayout(w); v.setSpacing(6); v.setContentsMargins(8,8,8,8)

        # ── Top section ────────────────────────────────────────────────────────
        top = QHBoxLayout(); top.setSpacing(8)

        # RPM gauge
        eng = QGroupBox("ENGINE")
        ev = QVBoxLayout(eng)
        self._rpm_gauge = RPMGauge()
        ev.addWidget(self._rpm_gauge)
        self._pt_speed = MetricCard("Speed (actual)", "", ISC_GREEN)
        ev.addWidget(self._pt_speed)
        top.addWidget(eng, stretch=2)

        # Inverter status & diagnostics
        itb = QGroupBox("INVERTER STATUS & DIAGNOSTICS")
        itg = QGridLayout(itb)
        self._pt_tm1   = MetricCard("Sensor 1",   "°C", F1_WARNING)
        self._pt_tm2   = MetricCard("Motor 2 (NTC)", "°C", F1_WARNING)
        self._pt_tbd   = MetricCard("Board",     "°C", ISC_GREEN)
        self._pt_tdcdc = MetricCard("DC-DC",     "°C", ISC_GREEN)
        self._pt_dem   = MetricCard("DEM Code",  "",   F1_ERROR)
        self._pt_foc   = MetricCard("FOC BitState", "", ISC_GREEN)
        itg.addWidget(self._pt_tm1,   0, 0); itg.addWidget(self._pt_tm2, 0, 1)
        itg.addWidget(self._pt_tbd,   1, 0); itg.addWidget(self._pt_tdcdc,1, 1)
        itg.addWidget(self._pt_dem,   2, 0); itg.addWidget(self._pt_foc,  2, 1)
        top.addWidget(itb, stretch=2)

        # Battery summary
        bsb = QGroupBox("BATTERY SUMMARY")
        bsg = QGridLayout(bsb)
        self._pt_vbus  = MetricCard("DC Bus",       "",  ISC_GREEN)
        self._pt_soc   = MetricCard("SOC (VTC6)",   "",  F1_BLUE)
        self._pt_iaccu = MetricCard("Pack Current",  "",  F1_PURPLE)
        self._pt_idcdc = MetricCard("DC-DC Current", "",  ISC_GREEN)
        self._pt_vcell = MetricCard("Min Cell V",    "",  F1_WARNING)
        self._pt_ams   = MetricCard("AMS State",     "",  ISC_GREEN)
        self._pt_trem  = MetricCard("Est. Cut-off",  "",  ISC_GREEN)
        bsg.addWidget(self._pt_vbus,  0, 0); bsg.addWidget(self._pt_soc,   0, 1)
        bsg.addWidget(self._pt_iaccu, 1, 0); bsg.addWidget(self._pt_idcdc, 1, 1)
        bsg.addWidget(self._pt_vcell, 2, 0); bsg.addWidget(self._pt_ams,   2, 1)
        bsg.addWidget(self._pt_trem,  3, 0, 1, 2)
        top.addWidget(bsb, stretch=2)
        v.addLayout(top, stretch=3)

        # ── Bottom section: per-module bars ────────────────────────────────────
        bot = QHBoxLayout(); bot.setSpacing(8)

        alert_temp = self.settings.get("alert_temp_c", 40.0)
        alert_cell = self.settings.get("alert_cell_mv", 3400.0)

        self._vbox_v = QGroupBox(f"PER-MODULE CELL VOLTAGE  [mV]   (min to max)   —   ALERT < {alert_cell:.0f} mV")
        vbv = QVBoxLayout(self._vbox_v)
        self._mod_v_bars: List[ModuleBarWidget] = []
        for i in range(5):
            b = ModuleBarWidget(i, "mV", lo=2800, hi=4250, warn_lo=alert_cell)
            vbv.addWidget(b); self._mod_v_bars.append(b)
        bot.addWidget(self._vbox_v, stretch=1)

        self._vbox_t = QGroupBox(f"PER-MODULE MAX TEMPERATURE  [ºC]   —   ALERT > {alert_temp:.0f} ºC")
        vbt = QVBoxLayout(self._vbox_t)
        self._mod_t_bars: List[ModuleBarWidget] = []
        for i in range(5):
            b = ModuleBarWidget(i, "ºC", lo=0, hi=80, warn_hi=alert_temp)
            vbt.addWidget(b); self._mod_t_bars.append(b)
        bot.addWidget(self._vbox_t, stretch=1)
        v.addLayout(bot, stretch=3)
        return w

    # ── Tab 4 — Dynamics ──────────────────────────────────────────────────────
    def _tab_dynamics(self) -> QWidget:
        w = QWidget()
        h = QHBoxLayout(w); h.setSpacing(8); h.setContentsMargins(8,8,8,8)

        # Pedals
        ped = QGroupBox("PEDAL INPUTS")
        pv = QHBoxLayout(ped); pv.setSpacing(20)
        self._ped_thr = PedalWidget("THROTTLE", ISC_GREEN)
        self._ped_brk = PedalWidget("BRAKE",    F1_ERROR)
        pv.addWidget(self._ped_thr); pv.addWidget(self._ped_brk)
        h.addWidget(ped, stretch=1)

        # Driver signals
        dsb = QGroupBox("DRIVER & CONTROL SIGNALS")
        dsv = QVBoxLayout(dsb); dsv.setSpacing(4)
        def _mc(t, u="", c=ISC_GREEN):
            card = MetricCard(t, u, c); dsv.addWidget(card); return card
        self._dyn_apps1  = _mc("APPS 1 (raw)")
        self._dyn_apps2  = _mc("APPS 2 (raw)")
        self._dyn_brake  = _mc("Brake (raw)",   c=F1_ERROR)
        self._dyn_torque = _mc("Torque %",      c=ISC_GREEN)
        self._dyn_start  = _mc("Start Button",  c=ISC_GREEN)
        self._dyn_ev23   = _mc("EV 2/3")
        self._dyn_t11    = _mc("T11 8/9")
        self._dyn_state  = _mc("Ctrl State",    c=F1_BLUE)
        h.addWidget(dsb, stretch=1)

        # G-force + IMU
        gbox = QGroupBox("IMU G-FORCE")
        gv   = QVBoxLayout(gbox); gv.setSpacing(6)
        self._g_circle = GCircleWidget()
        gv.addWidget(self._g_circle)
        self._g_long = QLabel("Long G:   0.00")
        self._g_lat  = QLabel("Lat  G:   0.00")
        self._g_tot  = QLabel("Total G:  0.00")
        for l in (self._g_long, self._g_lat, self._g_tot):
            l.setStyleSheet(f"color:{ISC_GREEN}; font-size:11px; font-family:'Courier New';")
            gv.addWidget(l)
        gv.addStretch()
        note = QLabel("IMU channels not yet in\nradio snapshot — placeholder.")
        note.setStyleSheet("color:#333; font-size:9px;")
        gv.addWidget(note)
        h.addWidget(gbox, stretch=1)
        return w

    # ── Log strip ─────────────────────────────────────────────────────────────
    def _make_log_strip(self) -> QGroupBox:
        box = QGroupBox("SYSTEM LOG & PIT-WALL NOTES")
        box.setStyleSheet(f"QGroupBox {{ color:{ISC_GREEN}; border:1px solid #222; }}")
        v = QVBoxLayout(box); v.setContentsMargins(6, 6, 6, 6)
        
        self._log = QTextEdit()
        self._log.setReadOnly(True)
        self._log.setMinimumHeight(150)
        self._log.setStyleSheet(
            f"background:{F1_DARK_BG}; color:{F1_TEXT}; "
            f"font-family:'Courier New'; font-size:9px; border:none;")
        v.addWidget(self._log)
        
        # Pit-Wall Notes Entry Row
        nh = QHBoxLayout()
        nh.setSpacing(6)
        
        self._inp_note = QLineEdit()
        self._inp_note.setPlaceholderText("Escriba una nota de telemetría (ej. 'Cambio de neumáticos') y pulse Enter para guardar...")
        self._inp_note.setStyleSheet(self.get_input_style())
        self._inp_note.returnPressed.connect(self._submit_note)
        nh.addWidget(self._inp_note, stretch=8)
        
        self._btn_add_note = QPushButton("Add Note")
        self._btn_add_note.setStyleSheet(self.get_button_style())
        self._btn_add_note.setFixedWidth(100)
        self._btn_add_note.clicked.connect(self._submit_note)
        nh.addWidget(self._btn_add_note)
        
        v.addLayout(nh)
        return box

    # ── Data access ───────────────────────────────────────────────────────────
    @staticmethod
    def _snap() -> dict:
        return rtt.get_latest_data().get('snapshot', {})

    # ── Alert checking ────────────────────────────────────────────────────────
    def _check_alerts(self, s: dict) -> None:
        if not (self.is_receiving or self.demo_mode):
            self._alert_banner.set_alerts([])
            self._ov_temp.set_alert(False)
            self._ov_vbus.set_alert(False)
            self._ov_vcell.set_alert(False)
            return
        alerts = []

        # Check receiver hardware/signal status from serial module
        rx_status = rtt.get_latest_data().get("__RECEIVER_STATUS__", {})
        hw_st = rx_status.get("hw_status", "OK")
        if hw_st == "NO_RADIO_HW":
            alerts.append(("RECEIVER HARDWARE FAULT: nRF24L01 module disconnected!", 'critical'))
        elif hw_st == "NO_SIGNAL":
            alerts.append(("RADIO SIGNAL LOST: No packets received from car!", 'warning'))

        # Check USB serial connection state (reconectando)
        st = rtt.get_latest_data().get("__STATUS__", {})
        badge = st.get("badge", "IDLE")
        reason = st.get("reason", "")
        if badge == "STALE" and reason == "reconectando...":
            alerts.append(("USB DISCONNECTED: Searching for RF-Nano receiver...", 'critical'))

        alert_temp = self.settings.get("alert_temp_c", 40.0)
        alert_volt = self.settings.get("alert_volt_v", 380.0)
        alert_cell = self.settings.get("alert_cell_mv", 3400.0)

        tmax = s.get('temp_max_modulo', [])
        valid_t = [t for t in tmax if t != 0]
        if valid_t and max(valid_t) > alert_temp:
            alerts.append((f"BATTERY TEMP {max(valid_t):.0f} ºC > {alert_temp:.0f} ºC", 'critical'))
        vbus = s.get('inv_dc_bus_V', 0)
        if 0 < vbus < alert_volt:
            alerts.append((f"DC BUS {vbus} V < {alert_volt:.0f} V", 'warning'))
        vcell = s.get('v_cell_min_mV', 0)
        if 0 < vcell < alert_cell:
            alerts.append((f"MIN CELL {vcell} mV < {alert_cell:.0f} mV", 'critical'))
        self._alert_banner.set_alerts(alerts)
        self._ov_temp.set_alert(any(a[1] == 'critical' and 'TEMP' in a[0] for a in alerts))
        self._ov_vbus.set_alert(any('DC BUS' in a[0] for a in alerts))
        self._ov_vcell.set_alert(any('MIN CELL' in a[0] for a in alerts))

        # Speak alarms if active stream
        if self.is_receiving or self.demo_mode:
            self._process_tts_alerts(alerts)

    # ── Main update loop ──────────────────────────────────────────────────────
    def _update(self):
        snap = self._snap()
        self._check_alerts(snap)
        self._update_badge()
        self._update_overview(snap)
        self._update_powertrain(snap)
        self._update_dynamics(snap)
        self._update_customize(snap)
        # Log new data strings
        if rtt.new_data_flag == 1:
            self._log_append(rtt.data_str)
            rtt.new_data_flag = 0
        # Detect dead RX thread
        if self.is_receiving and not self.demo_mode:
            if self.rx_thread and not self.rx_thread.is_alive():
                self._stop()

    def _update_badge(self):
        st    = rtt.get_latest_data().get("__STATUS__", {})
        badge = st.get("badge", "IDLE")
        base  = "font-size:14px; font-weight:bold; padding:8px 10px; border-radius:4px;"
        if badge == "LIVE":
            self._status_lbl.setText("LIVE")
            self._status_lbl.setStyleSheet(f"background:{ISC_GREEN}; color:{F1_DARK_BG}; {base} border:2px solid {ISC_GREEN};")
        elif badge == "STALE":
            self._status_lbl.setText("STALE")
            self._status_lbl.setStyleSheet(f"background:{F1_MID_BG}; color:{F1_WARNING}; {base} border:2px solid {F1_WARNING};")
        elif badge == "BAD":
            self._status_lbl.setText("BAD")
            self._status_lbl.setStyleSheet(f"background:{F1_MID_BG}; color:{F1_ERROR}; {base} border:2px solid {F1_ERROR};")
        else:
            self._status_lbl.setText("IDLE")
            self._status_lbl.setStyleSheet(f"background:{F1_MID_BG}; color:#555; {base} border:2px solid #333;")

    def _update_overview(self, s: dict):
        rpm    = s.get('inv_rpm',           0)
        vbus   = s.get('inv_dc_bus_V',      0)
        vcell  = s.get('v_cell_min_mV',     0)
        soc    = s.get('soc',               0)
        tpct   = s.get('torque_pct',        0)
        icur   = s.get('inv_current_actual',0)
        istate = s.get('inv_state',         0)
        pre    = s.get('ok_precharge',      0)
        ams    = s.get('ams_fsm_state',     0)
        ierr   = s.get('inv_error',         0)
        seq    = s.get('seq',               0)
        tick   = s.get('tick_ms',           0)
        tmax   = s.get('temp_max_modulo',  [0]*5)
        max_t  = max((t for t in tmax if t != 0), default=0)

        # SoC from VTC6 OCV table with IR compensation (uses corriente_accu to prevent sag dip)
        soc_vtc6 = soc_from_cell_mv(vcell, s.get('corriente_accu', 0)) if vcell > 0 else float(soc)

        # Real-time remaining battery duration estimator (30-second rolling average current)
        i_curr = max(0.0, float(s.get('corriente_accu', 0)))
        if not hasattr(self, '_i_rolling_hist'):
            self._i_rolling_hist = deque(maxlen=300) # 30 seconds at 10Hz
        self._i_rolling_hist.append(i_curr)
        i_avg = sum(self._i_rolling_hist) / len(self._i_rolling_hist) if self._i_rolling_hist else 0.0

        # Pack capacity: 18.0 Ah (95s6p Sony VTC6). Usable down to 5% SoC cutoff = ~17.1 Ah
        # Usable Ah remaining = 17.1 * (soc_vtc6 - 5) / 100
        usable_ah_rem = max(0.0, 17.1 * (soc_vtc6 - 5.0) / 100.0)
        if i_avg > 0.5:
            min_rem = (usable_ah_rem / i_avg) * 60.0
            time_str = f"{int(min_rem)}m {int((min_rem % 1) * 60):02d}s"
        else:
            time_str = "STANDBY"
            min_rem = 999.0

        tm2_val = s.get('inv_temp_motor2', s.get('inv_temp_pwrstg', 0))

        self._ov_rpm.set_value(f"{int(rpm):,}")
        self._ov_vbus.set_value(f"{vbus}")
        self._ov_tm2.set_value(f"{tm2_val:.0f}" if isinstance(tm2_val, (int, float)) else str(tm2_val))
        self._ov_temp.set_value(f"{max_t:.0f}")
        self._ov_soc.set_value(f"{soc_vtc6:.1f}")
        self._ov_time_rem.set_value(time_str)
        self._ov_torque.set_value(f"{tpct}")
        self._ov_cur.set_value(f"{icur}")
        self._ov_vcell.set_value(f"{vcell}")
        self._ov_state.set_value(f"{istate}")

        self._mini_rpm.setText(f"{int(rpm):,} rpm")
        self._mini_vbus.setText(f"{vbus} V")
        self._mini_temp.setText(f"{max_t:.0f} ºC")

        self._ov_plot_rpm.update_plot(rpm)
        self._ov_plot_vbus.update_plot(vbus)
        self._ov_plot_temp.update_plot(max_t)

        # Throttle + Brake overlay plot
        a1    = s.get('apps1_raw', 0)
        a2    = s.get('apps2_raw', 0)
        brk   = s.get('brake_raw', 0)
        a1_norm = max(0.0, min(1.0, (a1 - APPS1_MIN) / (APPS1_MAX - APPS1_MIN))) if APPS1_MAX > APPS1_MIN else 0.0
        a2_norm = max(0.0, min(1.0, (a2 - APPS2_MIN) / (APPS2_MAX - APPS2_MIN))) if APPS2_MAX > APPS2_MIN else 0.0
        thr_pct = max(a1_norm, a2_norm) * 100.0
        brk_pct = brk / ADC_MAX * 100.0
        self._ov_thr_hist.append(thr_pct)
        self._ov_brk_hist.append(brk_pct)
        xt  = list(range(HISTORY_LEN))
        self._line_thr.set_data(xt, list(self._ov_thr_hist))
        self._line_brk.set_data(xt, list(self._ov_brk_hist))
        self._ax_tb.set_xlim(0, HISTORY_LEN)
        self._canvas_tb.draw_idle()

        def _ind(lbl, text, on):
            lbl.setText(f"● {text}")
            lbl.setStyleSheet(f"color:{'#00c853' if on else '#333'}; font-size:10px; font-weight:bold;")
        _ind(self._ind_precharge, "PRECHARGE OK", bool(pre))
        _ind(self._ind_inv_ok,    f"INV {decode_inverter_state(istate)}", istate in _INV_ACTIVE_STATES)
        _ind(self._ind_ams,       f"AMS {ams}",   ams > 0)

        # Inverter fault decoder — pass istate for soft-fault substate awareness
        if ierr > 0:
            err_descs = decode_inverter_errors(ierr, istate)
            self._lbl_inv_errors.setText("\u274c FAULTS: " + " | ".join(err_descs))
            self._lbl_inv_errors.show()
        else:
            self._lbl_inv_errors.setText("")
            self._lbl_inv_errors.hide()

        self._lbl_seq.setText(f"SEQ: {seq}")
        self._lbl_tick.setText(f"TICK: {tick} ms")
        if not (self.is_receiving or self.demo_mode):
            lqi = 0.0
        else:
            lqi = rtt.get_latest_data().get('lqi', 100.0)
        self._signal_bars.set_lqi(lqi)
        self._lbl_lqi.setText(f"{lqi:.0f}%")
        if lqi >= 85:
            self._lbl_lqi.setStyleSheet("color:#00c853; font-size:9px; font-family:'Courier New';")
        elif lqi >= 70:
            self._lbl_lqi.setStyleSheet("color:#8bc34a; font-size:9px; font-family:'Courier New';")
        elif lqi >= 50:
            self._lbl_lqi.setStyleSheet("color:#f0b429; font-size:9px; font-family:'Courier New';")
        else:
            self._lbl_lqi.setStyleSheet("color:#ef4444; font-size:9px; font-family:'Courier New';")

    def _update_powertrain(self, s: dict):
        self._rpm_gauge.set_rpm(s.get('inv_rpm', 0))
        self._pt_speed.set_value(f"{s.get('inv_speed_actual', 0):.1f} km/h")
        _tm1_raw = s.get('inv_temp_motor1', 0)
        # After -50 offset applied in decoder: >=200°C means disconnected/out-of-range (DEM 21)
        _tm1_str = "N/C (Disconnected)" if _tm1_raw >= 200 else f"{_tm1_raw} ºC"
        self._pt_tm1.set_value(_tm1_str)
        _tm2_val = s.get('inv_temp_motor2', s.get('inv_temp_pwrstg', 0))
        self._pt_tm2.set_value(f"{_tm2_val} ºC")
        self._pt_tbd.set_value(f"{s.get('inv_temp_board', 0)} ºC")
        _tdcdc_raw = s.get('temp_dcdc', 0)
        _tdcdc_str = "N/C" if _tdcdc_raw <= -100 or _tdcdc_raw == -32768 else f"{_tdcdc_raw} ºC"
        self._pt_tdcdc.set_value(_tdcdc_str)
        
        dem_val = s.get('dem_code', s.get('inv_error', 0))
        dem_desc = INVERTER_ERRORS_MAP.get(dem_val, "Unknown")
        self._pt_dem.set_value(f"{dem_val} - {dem_desc}" if dem_val > 0 else "0 - No Fault")
        foc_val = s.get('emctrl_foc_bitstate', 0)
        self._pt_foc.set_value(f"0b{foc_val:08b}" if foc_val > 0 else "0 (OK)")
        
        self._pt_vbus.set_value(f"{s.get('inv_dc_bus_V', 0)} V")
        vcell_pt = s.get('v_cell_min_mV', 0)
        i_accu_pt = s.get('corriente_accu', 0)
        soc_vtc6_pt = soc_from_cell_mv(vcell_pt, i_accu_pt) if vcell_pt > 0 else float(s.get('soc', 0))
        self._pt_soc.set_value(f"{soc_vtc6_pt:.1f} %")
        # corriente_accu and corriente_dcdc are stored in Amperes (A)
        self._pt_iaccu.set_value(f"{s.get('corriente_accu', 0):.1f} A")
        self._pt_idcdc.set_value(f"{s.get('corriente_dcdc', 0):.1f} A")
        self._pt_vcell.set_value(f"{vcell_pt} mV")
        self._pt_ams.set_value(f"{s.get('ams_fsm_state', 0)}")
        
        # Est. cut-off duration metric
        i_hist = getattr(self, '_i_rolling_hist', None)
        i_avg_pt = (sum(i_hist) / len(i_hist)) if i_hist else 0.0
        usable_ah = max(0.0, 17.1 * (soc_vtc6_pt - 5.0) / 100.0)
        if i_avg_pt > 0.5:
            m_rem = (usable_ah / i_avg_pt) * 60.0
            self._pt_trem.set_value(f"{int(m_rem)}m {int((m_rem % 1)*60):02d}s")
        else:
            self._pt_trem.set_value("STANDBY")

        vmin = s.get('vmin_modulo',      [0]*5)
        vmax = s.get('vmax_modulo',      [0]*5)
        tmax = s.get('temp_max_modulo',  [0]*5)
        for i, bar in enumerate(self._mod_v_bars):
            bar.set_values(vmin[i] if i < len(vmin) else 0,
                           vmax[i] if i < len(vmax) else 0)
        for i, bar in enumerate(self._mod_t_bars):
            t = tmax[i] if i < len(tmax) else 0
            bar.set_values(0, t)

    def _update_dynamics(self, s: dict):
        a1    = s.get('apps1_raw',  0)
        a2    = s.get('apps2_raw',  0)
        brake = s.get('brake_raw',  0)
        
        # Normalise APPS sensors to 0-100% using empirical minimums/maximums
        a1_norm = max(0.0, min(1.0, (a1 - APPS1_MIN) / (APPS1_MAX - APPS1_MIN))) if APPS1_MAX > APPS1_MIN else 0.0
        a2_norm = max(0.0, min(1.0, (a2 - APPS2_MIN) / (APPS2_MAX - APPS2_MIN))) if APPS2_MAX > APPS2_MIN else 0.0
        thr_pct = max(a1_norm, a2_norm)
        
        self._ped_thr.set_value(thr_pct, int(max(a1, a2)))
        self._ped_brk.set_value(brake / ADC_MAX, int(brake))

        self._dyn_apps1.set_value(str(a1))
        self._dyn_apps2.set_value(str(a2))
        self._dyn_brake.set_value(str(brake))
        self._dyn_torque.set_value(f"{s.get('torque_pct', 0)}")
        self._dyn_start.set_value("ON" if s.get('start_button', 0) else "OFF")
        self._dyn_ev23.set_value(str(s.get('ev_2_3', 0)))
        self._dyn_t11.set_value(str(s.get('t11_8_9', 0)))
        self._dyn_state.set_value(str(s.get('state', 0)))

        # IMU updates
        ax = s.get('imu_ax_g', 0.0)
        ay = s.get('imu_ay_g', 0.0)
        self._g_circle.set_g_force(ax, ay)
        self._g_long.setText(f"Long G:   {ax:+.2f}")
        self._g_lat.setText(f"Lat  G:   {ay:+.2f}")
        self._g_tot.setText(f"Total G:  {math.sqrt(ax**2 + ay**2):.2f}")

    def _update_customize(self, s: dict):
        for panel in self._drop_panels:
            panel.update_value(s)

    # ── Reception ─────────────────────────────────────────────────────────────
    def _start(self):
        if self.is_receiving:
            return
        piloto   = self._inp_pilot.text()
        circuito = self._inp_circuit.text()
        port     = self.settings.get("port")
        baud     = int(self.settings.get("baud", 115200))
        use_mpl  = self.settings.get("use_influx", False)
        debug    = self.settings.get("debug", False)

        if self.demo_mode and DEMO_AVAILABLE:
            demo.start_demo(use_marple=use_mpl, piloto=piloto, circuito=circuito)
            self._log_append("[DEMO] Demo data feed started.")
        else:
            if not port:
                QMessageBox.warning(self, "No port", "No COM port selected. Open Settings.")
                return
            bucket = rtt.create_bucket(piloto, circuito)
            self._log_append(f"Serial start: {port} @ {baud}  session={bucket}")
            def _worker():
                try:
                    rtt.receive_data(bucket_id=bucket, piloto=piloto, circuito=circuito,
                                     port=port, baud=baud, use_influx=use_mpl, debug=debug)
                except Exception as ex:
                    signaler.log_message.emit(f"RX ERROR: {ex}")
                finally:
                    self.is_receiving = False
            self.rx_thread = threading.Thread(target=_worker, daemon=True)
            self.rx_thread.start()

        self.is_receiving = True
        self._btn_start.setEnabled(False)
        self._btn_stop.setEnabled(True)
        self._btn_settings.setEnabled(False)

    def _stop(self):
        if not self.is_receiving:
            return
        self._log_append("Stopping reception…")
        if self.demo_mode and DEMO_AVAILABLE:
            demo.stop_demo()
        else:
            rtt.new_data_flag = -1
            if self.rx_thread and self.rx_thread.is_alive():
                self.rx_thread.join(timeout=1.5)
        self.is_receiving = False
        self._btn_start.setEnabled(True)
        self._btn_stop.setEnabled(False)
        self._btn_settings.setEnabled(True)

    def _open_settings(self):
        self._settings_dlg = SettingsDialog(self)
        if self._settings_dlg.exec_():
            new = self._settings_dlg.get_settings()
            demo_changed = new["demo_mode"] != self.settings["demo_mode"]
            self.settings.update(new)
            current_settings.update(new)
            rtt.DEFAULT_PORT = new["port"]
            rtt.DEFAULT_BAUD = new["baud"]

            # Save settings to file
            self._save_settings_to_file()
            # Update warning thresholds in widgets
            self._update_widget_thresholds()

            if demo_changed:
                self.demo_mode = new["demo_mode"]
                self._log_append(f"Demo mode {'ENABLED' if self.demo_mode else 'DISABLED'}")
            self._log_append(f"Settings: port={new['port']} baud={new['baud']} marple={new['use_influx']} temp={new['alert_temp_c']} cell={new['alert_cell_mv']}")

    def _load_settings_from_file(self):
        import json
        settings_file = rtt.USER_DIR / "settings.json"
        if settings_file.exists():
            try:
                with open(settings_file, "r") as f:
                    saved = json.load(f)
                    current_settings.update(saved)
                    self.settings.update(saved)
            except Exception as e:
                self._log_append(f"Error loading settings.json: {e}")

    def _save_settings_to_file(self):
        import json
        settings_file = rtt.USER_DIR / "settings.json"
        try:
            to_save = {
                "port": self.settings.get("port"),
                "baud": self.settings.get("baud"),
                "use_influx": self.settings.get("use_influx"),
                "debug": self.settings.get("debug"),
                "demo_mode": self.settings.get("demo_mode"),
                "alert_temp_c": self.settings.get("alert_temp_c"),
                "alert_volt_v": self.settings.get("alert_volt_v"),
                "alert_cell_mv": self.settings.get("alert_cell_mv"),
            }
            with open(settings_file, "w") as f:
                json.dump(to_save, f, indent=4)
        except Exception as e:
            self._log_append(f"Error saving settings.json: {e}")

    def _update_widget_thresholds(self):
        temp_c = self.settings.get("alert_temp_c", 40.0)
        cell_mv = self.settings.get("alert_cell_mv", 3400.0)
        if hasattr(self, '_mod_v_bars') and self._mod_v_bars:
            for bar in self._mod_v_bars:
                bar._warn_lo = cell_mv
        if hasattr(self, '_mod_t_bars') and self._mod_t_bars:
            for bar in self._mod_t_bars:
                bar._warn_hi = temp_c
        if hasattr(self, '_vbox_v') and self._vbox_v:
            self._vbox_v.setTitle(f"PER-MODULE CELL VOLTAGE  [mV]   (min to max)   —   ALERT < {cell_mv:.0f} mV")
        if hasattr(self, '_vbox_t') and self._vbox_t:
            self._vbox_t.setTitle(f"PER-MODULE MAX TEMPERATURE  [ºC]   —   ALERT > {temp_c:.0f} ºC")

    def _open_post_race(self):
        if self._post_race_win is None or not self._post_race_win.isVisible():
            self._post_race_win = PostRaceWindow()
            self._post_race_win.show()

    def _log_append(self, msg: str):
        if self._log is None:
            return
        ts = datetime.now().strftime("%H:%M:%S")
        self._log.append(f"[{ts}] {msg}")
        doc = self._log.document()
        while doc.blockCount() > 30:
            cur = self._log.textCursor()
            cur.movePosition(cur.Start)
            cur.select(cur.BlockUnderCursor)
            cur.removeSelectedText(); cur.deleteChar()

    def _speak_alert(self, text: str):
        # Run speech synthesis in a background daemon thread so it doesn't block PyQt GUI thread
        def _speak():
            try:
                # 1. Try Windows native SAPI voice synthesis via win32com
                import win32com.client
                speaker = win32com.client.Dispatch("SAPI.SpVoice")
                speaker.Rate = -2       # Slower rate makes it much more intelligible over background noise
                speaker.Volume = 100
                
                # Prefer high-intelligibility female voices (Zira or Helena/Sabina)
                voices = speaker.GetVoices()
                for i in range(voices.Count):
                    desc = voices.Item(i).GetDescription()
                    if any(name in desc for name in ["Zira", "Hazel", "Helena", "Sabina"]):
                        speaker.Voice = voices.Item(i)
                        break
                speaker.Speak(text)
            except Exception:
                try:
                    # 2. Fallback to PowerShell System.Speech (native on all Windows)
                    # Configures a slower speech rate, max volume, and selects Zira or Helena if available.
                    safe_text = text.replace("'", "''")
                    ps_cmd = (
                        "Add-Type -AssemblyName System.Speech; "
                        "$s = New-Object System.Speech.Synthesis.SpeechSynthesizer; "
                        "$s.Rate = -2; "
                        "$s.Volume = 100; "
                        "$v = $s.GetInstalledVoices() | ForEach-Object { $_.VoiceInfo } | "
                        "Where-Object { $_.Name -like '*Zira*' -or $_.Name -like '*Helena*' -or $_.Name -like '*Hazel*' } | "
                        "Select-Object -First 1; "
                        "if ($v) { $s.SelectVoice($v.Name) } else { "
                        "try { $s.SelectVoiceByHints([System.Speech.Synthesis.VoiceGender]::Female) } catch {} }; "
                        f"$s.Speak('{safe_text}')"
                    )
                    subprocess.run(["powershell", "-Command", ps_cmd], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                except Exception:
                    pass
        threading.Thread(target=_speak, daemon=True).start()

    def _process_tts_alerts(self, alerts: List[tuple]):
        if not hasattr(self, '_spoken_alerts_timestamps'):
            self._spoken_alerts_timestamps = {}
        
        now = time.time()
        for alert_text, severity in alerts:
            alert_key = alert_text
            if "BATTERY TEMP" in alert_text:
                alert_key = "BATTERY_TEMP_ALERT"
            elif "DC BUS" in alert_text:
                alert_key = "DC_BUS_ALERT"
            elif "MIN CELL" in alert_text:
                alert_key = "MIN_CELL_ALERT"
            elif "RECEIVER HARDWARE" in alert_text:
                alert_key = "RECEIVER_HW_ALERT"
            elif "RADIO SIGNAL" in alert_text:
                alert_key = "RADIO_SIGNAL_ALERT"
            elif "USB DISCONNECTED" in alert_text:
                alert_key = "USB_DISCONNECT_ALERT"

            # Trigger warnings/beeps at most once every 20 seconds per warning type
            last_time = self._spoken_alerts_timestamps.get(alert_key, 0.0)
            if now - last_time > 20.0:
                self._spoken_alerts_timestamps[alert_key] = now
                
                # 1. Play the loud audible beep alarm (independent of TTS settings)
                self._play_alarm_sound(alert_key)
                
                # 2. Text-to-speech announcement (if enabled)
                if self.settings.get("enable_tts", True):
                    friendly_text = alert_text
                    if "ºC" in friendly_text:
                        friendly_text = friendly_text.replace("ºC", "degrees Celsius")
                    if "mV" in friendly_text:
                        friendly_text = friendly_text.replace("mV", "millivolts")
                    if "V" in friendly_text:
                        friendly_text = friendly_text.replace("V", "volts")
                    self._speak_alert(friendly_text)

    def closeEvent(self, ev):
        if self.is_receiving:
            self._stop(); time.sleep(0.3)
        ev.accept()

    def _toggle_theme(self):
        global F1_DARK_BG, F1_MID_BG, F1_PANEL_BG, F1_TEXT
        if self.theme_mode == "dark":
            self.theme_mode = "light"
            F1_DARK_BG = '#f0f2f5'
            F1_MID_BG = '#ffffff'
            F1_PANEL_BG = '#f9f9fa'
            F1_TEXT = '#1a1a1a'
        else:
            self.theme_mode = "dark"
            F1_DARK_BG = '#111111'
            F1_MID_BG = '#1a1a1a'
            F1_PANEL_BG = '#222222'
            F1_TEXT = '#e0e0e0'
        
        self._btn_theme.setText("☀️  Light" if self.theme_mode == "dark" else "🌙  Dark")
        self._apply_theme_to_all()
        self._log_append(f"[THEME] Switch to {self.theme_mode.upper()} mode.")

    def _apply_theme_to_all(self):
        # 1. Update the main window palette and stylesheet
        self._apply_theme()
        
        # 2. Update top bar background and inputs
        self._top_bar.setStyleSheet(f"QFrame {{ background:{F1_MID_BG}; border-radius:4px; }}")
        ins = self.get_input_style()
        self._inp_pilot.setStyleSheet(ins)
        self._inp_circuit.setStyleSheet(ins)
        
        is_light = (F1_TEXT == '#1a1a1a')
        txt_col = '#111111' if is_light else '#ffffff'
        spine_col = '#aaaaaa' if is_light else '#555555'
        grid_col = '#cccccc' if is_light else '#444444'

        # 3. Restyle self._ax_tb (Overview Throttle/Brake plot)
        if hasattr(self, '_ax_tb') and hasattr(self, '_canvas_tb'):
            self._ax_tb.set_facecolor(F1_PANEL_BG)
            self._canvas_tb.figure.patch.set_facecolor(F1_PANEL_BG)
            self._ax_tb.tick_params(labelsize=7.5, colors=txt_col)
            self._ax_tb.grid(True, color=grid_col, alpha=0.5 if is_light else 0.35)
            for s in self._ax_tb.spines.values():
                s.set_color(spine_col)
            self._ax_tb.legend(fontsize=7, loc='upper left',
                               facecolor=F1_PANEL_BG, labelcolor=F1_TEXT,
                               edgecolor=spine_col, framealpha=0.9)
            self._canvas_tb.draw_idle()
        
        # 4. Recursively update all child widgets
        def _restyle(w):
            if isinstance(w, MetricCard):
                w._value.setStyleSheet(f"color:{F1_TEXT}; font-size:19px; font-weight:bold; background:transparent; border:none;")
                col = F1_ERROR if w._alerting else w._color
                w._set_border(col)
                w._title.setStyleSheet(f"color:{col}; font-size:9px; font-weight:bold; background:transparent; border:none;")
                w.update()

            elif isinstance(w, MplCanvas):
                w._ax.set_facecolor(F1_PANEL_BG)
                w._canvas.figure.patch.set_facecolor(F1_PANEL_BG)
                w._ax.tick_params(labelsize=7.5, colors=txt_col)
                w._ax.grid(True, color=grid_col, alpha=0.5 if is_light else 0.35)
                for s in w._ax.spines.values():
                    s.set_color(spine_col)
                w._canvas.draw_idle()
                
            elif isinstance(w, DroppablePlotPanel):
                w._title_lbl.setStyleSheet(f"color:{ISC_GREEN if w._channel else ('#666' if is_light else '#888')}; font-size:9px; font-weight:bold; background:transparent; border:none;")
                w._val_lbl.setStyleSheet(f"color:{ISC_GREEN}; font-size:15px; font-weight:bold; background:transparent; border:none;")
                w.setStyleSheet(f"QFrame {{ background:{F1_PANEL_BG}; color:{F1_TEXT}; border:1px solid {'#ccc' if is_light else '#333'}; }}")
                w._ax.set_facecolor(F1_PANEL_BG)
                w._canvas.figure.patch.set_facecolor(F1_PANEL_BG)
                w._ax.tick_params(labelsize=7.5, colors=txt_col)
                w._ax.grid(True, color=grid_col, alpha=0.5 if is_light else 0.35)
                for s in w._ax.spines.values():
                    s.set_color(spine_col)
                w._canvas.draw_idle()
                
            elif isinstance(w, QTextEdit):
                w.setStyleSheet(f"background:{F1_PANEL_BG}; color:{F1_TEXT}; border:1px solid {'#ccc' if is_light else '#333'}; font-family:'Courier New'; font-size:9px;")
                
            elif isinstance(w, QComboBox) or isinstance(w, QLineEdit):
                w.setStyleSheet(self.get_input_style())

            elif isinstance(w, QTableWidget):
                w.setStyleSheet(f"""
                    QTableWidget {{ background:{F1_MID_BG}; color:{F1_TEXT}; gridline-color:{'#ddd' if is_light else '#222'}; border:1px solid {'#ccc' if is_light else '#222'}; }}
                    QHeaderView::section {{ background:{F1_PANEL_BG}; color:{ISC_GREEN}; padding:4px; border:1px solid {'#ccc' if is_light else '#222'}; font-weight:bold; }}
                """)
                for r in range(w.rowCount()):
                    for c in range(w.columnCount()):
                        item = w.item(r, c)
                        if item:
                            item.setForeground(QColor(F1_TEXT))

            elif isinstance(w, QCheckBox):
                w.setStyleSheet(f"color:{F1_TEXT}; font-size:10px;")

            elif isinstance(w, QProgressBar):
                w.setStyleSheet(f"""
                    QProgressBar {{ background:{F1_MID_BG}; border:1px solid {'#ccc' if is_light else '#333'}; border-radius:3px; text-align:center; color:{F1_TEXT}; }}
                    QProgressBar::chunk {{ background:{ISC_GREEN}; }}
                """)
                
            elif isinstance(w, AlertBanner):
                w.setStyleSheet(f"QFrame {{ background:{F1_PANEL_BG}; border:1px dashed {'#ccc' if is_light else '#333'}; border-radius:4px; }}")
                
            elif isinstance(w, GCircleWidget):
                w.update()
                
            elif isinstance(w, PedalWidget):
                w.update()
                
            elif isinstance(w, RPMGauge):
                w.update()
                
            elif isinstance(w, ModuleBarWidget):
                w.update()
                
            # Restyle buttons
            elif isinstance(w, QPushButton):
                if w == self._btn_start:
                    w.setStyleSheet(self.get_button_style('accent'))
                else:
                    w.setStyleSheet(self.get_button_style())
                    
            for child in w.findChildren(QWidget):
                _restyle(child)
                
        _restyle(self)

    def _submit_note(self):
        text = self._inp_note.text().strip()
        if not text:
            return
        
        self._inp_note.clear()
        self._log_append(f"🗒️ NOTE: {text}")
        
        import ISC_RTT_serial
        import ISC_RTT_demo
        written = False
        
        # Real-time Serial mode
        if hasattr(ISC_RTT_serial, "_excel_logger") and ISC_RTT_serial._excel_logger is not None:
            try:
                ISC_RTT_serial._excel_logger.write_note(text)
                written = True
            except Exception as e:
                logger.error(f"Failed writing real-time note: {e}")
                
        # Demo mode
        if hasattr(ISC_RTT_demo, "_gen") and ISC_RTT_demo._gen is not None:
            gen = ISC_RTT_demo._gen
            if gen.running and gen.logger is not None:
                try:
                    gen.logger.write_note(text)
                    written = True
                except Exception as e:
                    logger.error(f"Failed writing demo note: {e}")
                    
        if written:
            self._log_append("✓ Note written to CSV log file.")
        else:
            self._log_append("⚠ Warning: No active logging session to save the note.")

    def _play_alarm_sound(self, alert_key: str):
        """Play distinct beep frequencies in a background thread to warn the engineer."""
        import winsound
        def _beep():
            try:
                if alert_key == "BATTERY_TEMP_ALERT":
                    # High pitch fast warning beeps
                    for _ in range(3):
                        winsound.Beep(1800, 100)
                        time.sleep(0.08)
                elif alert_key in ("DC_BUS_ALERT", "MIN_CELL_ALERT"):
                    # High-low alarm sirens
                    winsound.Beep(1200, 180)
                    winsound.Beep(900, 180)
                elif alert_key in ("RADIO_SIGNAL_ALERT", "USB_DISCONNECT_ALERT", "RECEIVER_HW_ALERT"):
                    # Low double beeps
                    winsound.Beep(600, 120)
                    time.sleep(0.05)
                    winsound.Beep(450, 150)
            except Exception as e:
                logger.warning("[ALARM] Winsound beep failed: %s", e)
        threading.Thread(target=_beep, daemon=True).start()

    def _tab_post_race(self) -> QWidget:
        w = QWidget()
        h = QHBoxLayout(w); h.setSpacing(8); h.setContentsMargins(8,8,8,8)

        # Left Column: Connection & Status
        lv = QVBoxLayout()
        
        # Connection Group Box
        conn_box = QGroupBox("CAN CONNECTION SETTINGS")
        conn_box.setStyleSheet(f"QGroupBox {{ color:{ISC_GREEN}; font-weight:bold; }}")
        cv = QVBoxLayout(conn_box); cv.setSpacing(6)
        
        cv.addWidget(self._lbl("CAN Interface:", "color:#888; font-size:10px;"))
        self._pr_interface = QComboBox()
        self._pr_interface.addItems(["pcan", "slcan", "vector", "socketcan", "virtual"])
        self._pr_interface.setStyleSheet(self.get_input_style())
        cv.addWidget(self._pr_interface)
        
        cv.addWidget(self._lbl("CAN Channel:", "color:#888; font-size:10px;"))
        self._pr_channel = QLineEdit("PCAN_USBBUS1")
        self._pr_channel.setStyleSheet(self.get_input_style())
        cv.addWidget(self._pr_channel)
        
        cv.addWidget(self._lbl("Bitrate (bps):", "color:#888; font-size:10px;"))
        self._pr_bitrate = QComboBox()
        self._pr_bitrate.addItems(["500000", "250000", "1000000", "125000"])
        self._pr_bitrate.setStyleSheet(self.get_input_style())
        cv.addWidget(self._pr_bitrate)
        
        cv.addWidget(self._lbl("AMS Node ID (Node 1 = 1, Node 2 = 2):", "color:#888; font-size:10px;"))
        self._pr_node_id = QLineEdit("1")
        self._pr_node_id.setStyleSheet(self.get_input_style())
        cv.addWidget(self._pr_node_id)
        
        # Virtual Simulator Checkbox
        self._pr_use_sim = QCheckBox("Enable Virtual Loopback Simulator")
        self._pr_use_sim.setStyleSheet("color:#aaa; font-size:10px;")
        self._pr_use_sim.toggled.connect(self._toggle_sim_mode)
        cv.addWidget(self._pr_use_sim)
        
        self._pr_btn_connect = QPushButton("Connect to CAN")
        self._pr_btn_connect.setStyleSheet(self.get_button_style())
        self._pr_btn_connect.clicked.connect(self._on_post_race_connect)
        cv.addWidget(self._pr_btn_connect)
        
        lv.addWidget(conn_box)
        
        # Status Box
        status_box = QGroupBox("EXTRACTION STATUS")
        status_box.setStyleSheet(f"QGroupBox {{ color:{ISC_GREEN}; font-weight:bold; }}")
        sv = QVBoxLayout(status_box); sv.setSpacing(6)
        
        self._pr_lbl_status = QLabel("Disconnected")
        self._pr_lbl_status.setStyleSheet("color:#aaa; font-size:11px; font-family:'Courier New';")
        sv.addWidget(self._pr_lbl_status)
        
        # Progress Bar
        self._pr_progress = QProgressBar()
        self._pr_progress.setValue(0)
        self._pr_progress.setStyleSheet(f"""
            QProgressBar {{ background:{F1_DARK_BG}; border:1px solid #333; border-radius:3px; text-align:center; color:#fff; }}
            QProgressBar::chunk {{ background:{ISC_GREEN}; }}
        """)
        sv.addWidget(self._pr_progress)
        
        lv.addWidget(status_box)

        self._pr_btn_open_merge = QPushButton("Open Import/Merge Options")
        self._pr_btn_open_merge.setStyleSheet(self.get_button_style('accent'))
        self._pr_btn_open_merge.clicked.connect(self._open_post_race)
        lv.addWidget(self._pr_btn_open_merge)
        
        lv.addStretch()
        
        lw = QWidget(); lw.setLayout(lv); lw.setFixedWidth(220)
        h.addWidget(lw)

        # Right Column: Files list
        rv = QVBoxLayout()
        rv.addWidget(self._lbl("MicroSD Card File Log Directory", f"color:{ISC_GREEN}; font-size:12px; font-weight:bold;"))
        
        self._pr_table = QTableWidget(0, 5)
        self._pr_table.setHorizontalHeaderLabels(["Index", "File Name", "Size (bytes)", "Date / Modified", "Action"])
        self._pr_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self._pr_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self._pr_table.setStyleSheet(f"""
            QTableWidget {{ background:{F1_DARK_BG}; color:#fff; gridline-color:#222; border:1px solid #222; }}
            QHeaderView::section {{ background:{F1_MID_BG}; color:{ISC_GREEN}; padding:4px; border:1px solid #222; }}
        """)
        self._pr_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        rv.addWidget(self._pr_table)
        
        self._pr_btn_refresh = QPushButton("Refresh File Directory")
        self._pr_btn_refresh.setStyleSheet(self.get_button_style())
        self._pr_btn_refresh.clicked.connect(self._on_post_race_refresh)
        self._pr_btn_refresh.setEnabled(False)
        rv.addWidget(self._pr_btn_refresh)
        
        rw = QWidget(); rw.setLayout(rv)
        h.addWidget(rw)
        
        return w

    def _toggle_sim_mode(self, enabled):
        if enabled:
            self._pr_interface.setCurrentText("virtual")
            self._pr_channel.setText("test_channel")
            self._pr_interface.setEnabled(False)
            self._pr_channel.setEnabled(False)
        else:
            self._pr_interface.setEnabled(True)
            self._pr_channel.setEnabled(True)
            self._pr_interface.setCurrentText("pcan")
            self._pr_channel.setText("PCAN_USBBUS1")

    def _on_post_race_connect(self):
        if not _CAN_OK:
            QMessageBox.critical(self, "CAN Error", "python-can library is not installed or import failed.")
            return
            
        if hasattr(self, "_pr_is_connected") and self._pr_is_connected:
            # Disconnect
            if hasattr(self, "_sim_thread") and self._sim_thread is not None:
                self._sim_thread.running = False
                self._sim_thread.wait()
                self._sim_thread = None
                
            self._pr_is_connected = False
            self._pr_btn_connect.setText("Connect to CAN")
            self._pr_btn_refresh.setEnabled(False)
            self._pr_lbl_status.setText("Disconnected")
            self._pr_table.setRowCount(0)
            return
            
        # Start connection process
        interface = self._pr_interface.currentText()
        channel = self._pr_channel.text()
        bitrate = int(self._pr_bitrate.currentText())
        try:
            node_id = int(self._pr_node_id.text())
        except ValueError:
            QMessageBox.warning(self, "Input Error", "AMS Node ID must be an integer.")
            return
            
        if self._pr_use_sim.isChecked():
            # Start loopback simulator
            self._sim_thread = AMSSimulatorThread(channel=channel)
            self._sim_thread.start()
            
        self._pr_lbl_status.setText("Connecting...")
        
        # Start extraction list thread
        self._pr_thread = AMSExtractionThread(
            interface=interface,
            channel=channel,
            bitrate=bitrate,
            ams_node_id=node_id,
            action="list"
        )
        self._pr_thread.files_listed.connect(self._on_extraction_files_listed)
        self._pr_thread.finished.connect(self._on_extraction_finished)
        self._pr_thread.status.connect(self._on_extraction_status)
        self._pr_thread.start()

    def _on_post_race_refresh(self):
        if not hasattr(self, "_pr_is_connected") or not self._pr_is_connected:
            return
        interface = self._pr_interface.currentText()
        channel = self._pr_channel.text()
        bitrate = int(self._pr_bitrate.currentText())
        node_id = int(self._pr_node_id.text())
        
        self._pr_lbl_status.setText("Refreshing directory...")
        self._pr_thread = AMSExtractionThread(
            interface=interface,
            channel=channel,
            bitrate=bitrate,
            ams_node_id=node_id,
            action="list"
        )
        self._pr_thread.files_listed.connect(self._on_extraction_files_listed)
        self._pr_thread.finished.connect(self._on_extraction_finished)
        self._pr_thread.status.connect(self._on_extraction_status)
        self._pr_thread.start()

    def _on_extraction_files_listed(self, files):
        self._pr_is_connected = True
        self._pr_btn_connect.setText("Disconnect")
        self._pr_btn_refresh.setEnabled(True)
        self._pr_table.setRowCount(0)
        
        for file in files:
            row = self._pr_table.rowCount()
            self._pr_table.insertRow(row)
            
            # File Index
            idx_item = QTableWidgetItem(str(file["index"]))
            idx_item.setTextAlignment(Qt.AlignCenter)
            self._pr_table.setItem(row, 0, idx_item)
            
            # File Name
            name_item = QTableWidgetItem(file["name"])
            name_item.setTextAlignment(Qt.AlignCenter)
            self._pr_table.setItem(row, 1, name_item)
            
            # File Size
            size_item = QTableWidgetItem(f"{file['size']:,}")
            size_item.setTextAlignment(Qt.AlignCenter)
            self._pr_table.setItem(row, 2, size_item)
            
            # Date / Modified
            mtime = file.get("mtime", 0)
            if mtime > 0:
                try:
                    dt_str = datetime.fromtimestamp(mtime).strftime("%Y-%m-%d %H:%M:%S")
                except Exception:
                    dt_str = "Unknown"
            else:
                dt_str = "N/A"
            date_item = QTableWidgetItem(dt_str)
            date_item.setTextAlignment(Qt.AlignCenter)
            self._pr_table.setItem(row, 3, date_item)
            
            # Action Download Button
            btn = QPushButton("Download")
            btn.setStyleSheet(self.get_button_style())
            file_idx = file["index"]
            file_name = file["name"]
            btn.clicked.connect(lambda checked=False, f_idx=file_idx, f_name=file_name: self._start_file_download(f_idx, f_name))
            self._pr_table.setCellWidget(row, 4, btn)
            
        self._pr_lbl_status.setText(f"Connected. Found {len(files)} logs.")

    def _start_file_download(self, file_idx, file_name):
        interface = self._pr_interface.currentText()
        channel = self._pr_channel.text()
        bitrate = int(self._pr_bitrate.currentText())
        node_id = int(self._pr_node_id.text())
        
        self._pr_lbl_status.setText(f"Starting download of {file_name}...")
        self._pr_progress.setValue(0)
        
        self._pr_thread = AMSExtractionThread(
            interface=interface,
            channel=channel,
            bitrate=bitrate,
            ams_node_id=node_id,
            action="download",
            selected_file_index=file_idx,
            selected_file_name=file_name
        )
        self._pr_thread.progress.connect(self._on_extraction_progress)
        self._pr_thread.status.connect(self._on_extraction_status)
        self._pr_thread.finished.connect(self._on_extraction_finished)
        self._pr_thread.start()

    def _on_extraction_progress(self, downloaded, total, speed):
        pct = int(downloaded / total * 100) if total > 0 else 0
        self._pr_progress.setValue(pct)
        self._pr_lbl_status.setText(f"Downloading: {pct}% ({downloaded:,}/{total:,} B) - {speed:.1f} KB/s")

    def _on_extraction_status(self, text):
        self._pr_lbl_status.setText(text)

    def _show_custom_msgbox(self, title: str, message: str, is_error: bool = False):
        msg = QMessageBox(self)
        msg.setIcon(QMessageBox.Critical if is_error else QMessageBox.Information)
        msg.setWindowTitle(title)
        msg.setText(message)
        msg.setStyleSheet(f"""
            QMessageBox {{
                background-color: {F1_DARK_BG};
            }}
            QLabel {{
                color: #ffffff;
                font-size: 11px;
                font-weight: bold;
                padding: 6px;
            }}
            QPushButton {{
                background-color: {F1_MID_BG};
                color: #ffffff;
                border: 1px solid #555;
                border-radius: 4px;
                padding: 5px 18px;
                font-weight: bold;
                min-width: 65px;
            }}
            QPushButton:hover {{
                background-color: {ISC_GREEN};
                color: #000000;
            }}
        """)
        msg.exec_()

    def _on_extraction_finished(self, success, result):
        if success:
            if "Files listed" in result:
                return
            self._pr_progress.setValue(100)
            self._pr_lbl_status.setText(f"Success! Saved to {Path(result).name}")
            self._show_custom_msgbox("Extraction Complete", f"File downloaded successfully to:\n\n{result}", is_error=False)
        else:
            self._pr_lbl_status.setText(f"Error: {result}")
            self._show_custom_msgbox("Extraction Error", f"Extraction failed:\n\n{result}", is_error=True)


# ══════════════════════════════════════════════════════════════════════════════
#  CAN LOG EXTRACTION MODULE (LOGFS PROTOCOL CLIENT)
# ══════════════════════════════════════════════════════════════════════════════

def convert_card_csv_to_telemetry_csv(input_bytes: bytes, output_path: Path):
    import csv
    import io
    from datetime import datetime
    
    text = input_bytes.decode('utf-8', errors='ignore')
    reader = csv.reader(io.StringIO(text))
    rows = list(reader)
    if not rows:
        raise Exception("Log file is empty or invalid CSV.")
    
    header = [name.strip() for name in rows[0]]
    col_map = {name: idx for idx, name in enumerate(header)}
    
    headers = [
        "time", "time_elapsed_s", "seq", "tick_ms", "start_button",
        "apps1_raw", "apps2_raw", "brake_raw", "torque_pct", "ev_2_3",
        "t11_8_9", "ctrl_state", "ok_precharge", "ams_fsm_state",
        "v_cell_min_mV", "soc",
        "vmin_mod0", "vmin_mod1", "vmin_mod2", "vmin_mod3", "vmin_mod4",
        "vmax_mod0", "vmax_mod1", "vmax_mod2", "vmax_mod3", "vmax_mod4",
        "corriente_accu", "corriente_dcdc", "temp_dcdc",
        "tmax_mod0", "tmax_mod1", "tmax_mod2", "tmax_mod3", "tmax_mod4",
        "inv_state", "inv_vconfig_active", "inv_error", "inv_dc_bus_V",
        "inv_temp_motor1", "inv_temp_pwrstg", "inv_temp_board",
        "inv_rpm", "inv_speed_actual", "inv_current_actual",
        "imu_ax_g", "imu_ay_g", "imu_az_g", "imu_gx_dps", "imu_gy_dps",
        "imu_gz_dps", "imu_roll_deg", "imu_pitch_deg", "notes"
    ]
    
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    with open(output_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        
        start_tick = None
        for i, row in enumerate(rows[1:]):
            if not row or len(row) < len(col_map):
                continue
            
            def get_val(col_name, default=0.0):
                idx = col_map.get(col_name)
                if idx is not None and idx < len(row):
                    try:
                        return float(row[idx])
                    except ValueError:
                        return default
                return default

            tick_ms = get_val("tick_ms")
            if start_tick is None:
                start_tick = tick_ms
            elapsed = (tick_ms - start_tick) / 1000.0
            
            vmin = [0.0] * 5
            vmax = [0.0] * 5
            tmax = [0.0] * 5
            
            for m in range(5):
                m_cells = []
                for c in range(19):
                    val = get_val(f"c{m}_{c}", None)
                    if val is not None:
                        m_cells.append(val)
                if m_cells:
                    vmin[m] = min(m_cells)
                    vmax[m] = max(m_cells)
                
                m_temps = []
                for t in range(40):
                    val = get_val(f"t{m}_{t}", None)
                    if val is not None:
                        m_temps.append(val)
                if m_temps:
                    tmax[m] = max(m_temps)
            
            vmin_global = get_val("vmin_mV", 0.0)
            soc_val = soc_from_cell_mv(vmin_global) if vmin_global > 0 else get_val("soc", 0.0)
            
            pack_current_mA = get_val("I_filt_mA", 0.0)
            corriente_accu = pack_current_mA / 1000.0
            
            dcdc_current_mA = get_val("Idcdc_mA", 0.0)
            corriente_dcdc = dcdc_current_mA / 1000.0
            
            out_row = [
                datetime.now().isoformat(),
                f"{elapsed:.3f}",
                i,
                int(tick_ms),
                0,
                0, 0, 0,
                0,
                0,
                0,
                0,
                int(get_val("ams_ok", 0)),
                int(get_val("fsm", 0)),
                int(vmin_global),
                f"{soc_val:.1f}",
                vmin[0], vmin[1], vmin[2], vmin[3], vmin[4],
                vmax[0], vmax[1], vmax[2], vmax[3], vmax[4],
                f"{corriente_accu:.1f}",
                f"{corriente_dcdc:.1f}",
                0,
                tmax[0], tmax[1], tmax[2], tmax[3], tmax[4],
                0, 0, 0,
                int(get_val("dcbus_V", 0)),
                0, 0, 0,
                0, 0, 0,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0,
                ""
            ]
            writer.writerow(out_row)


class AMSExtractionThread(QThread):
    progress = pyqtSignal(int, int, float)
    status = pyqtSignal(str)
    files_listed = pyqtSignal(list)
    finished = pyqtSignal(bool, str)

    def __init__(self, interface, channel, bitrate, ams_node_id=1, action="list", selected_file_index=None, selected_file_name=None):
        super().__init__()
        self.interface = interface
        self.channel = channel
        self.bitrate = bitrate
        self.ams_node_id = ams_node_id
        self.action = action
        self.selected_file_index = selected_file_index
        self.selected_file_name = selected_file_name

    def run(self):
        if not _CAN_OK:
            self.finished.emit(False, "python-can library is not installed.")
            return
            
        bus = None
        try:
            self.status.emit("Opening CAN bus...")
            bus = can.Bus(interface=self.interface, channel=self.channel, bitrate=self.bitrate)
            
            rx_id = 0x010 + self.ams_node_id
            tx_id = 0x000 + self.ams_node_id
            
            self.status.emit("Establishing diagnostic session (CONNECT)...")
            self.isotp_send(bus, tx_id, [0x00, 0x01])
            resp = self.isotp_recv(bus, rx_id)
            if not resp or resp[0] != 0x01 or resp[1] != 0x01:
                raise Exception("Failed to connect to AMS diagnostic service.")
                
            if self.action == "list":
                self.status.emit("Fetching file directory list...")
                self.isotp_send(bus, tx_id, [0x00, 0x21, 0x00, 0x00])
                resp = self.isotp_recv(bus, rx_id)
                if not resp:
                    raise Exception("No response to file listing request.")
                if resp[0] == 0x02:
                    raise Exception(f"LOGFS_LIST NACK received: code {resp[2] if len(resp) > 2 else 0}")
                if resp[0] != 0x01 or resp[1] != 0x21:
                    raise Exception("Invalid response to file listing request.")
                    
                next_cursor = struct.unpack("<H", resp[2:4])[0]
                count = resp[4]
                offset = 5
                entries = []
                
                rem_len = len(resp) - offset
                entry_size = 22
                if count > 0:
                    entry_size = rem_len // count
                    
                for _ in range(count):
                    if offset + entry_size > len(resp):
                        break
                    entry_bytes = resp[offset : offset + entry_size]
                    index = struct.unpack("<H", entry_bytes[0:2])[0]
                    if entry_size >= 24:
                        size = struct.unpack("<I", entry_bytes[4:8])[0]
                        mtime = struct.unpack("<I", entry_bytes[8:12])[0]
                        name_bytes = entry_bytes[12:24]
                    else:
                        size = struct.unpack("<I", entry_bytes[2:6])[0]
                        mtime = struct.unpack("<I", entry_bytes[6:10])[0]
                        name_bytes = entry_bytes[10:22]
                    name = name_bytes.decode('utf-8', errors='ignore').split('\x00', 1)[0].strip()
                    entries.append({
                        "index": index,
                        "size": size,
                        "mtime": mtime,
                        "name": name
                    })
                    offset += entry_size
                
                self.isotp_send(bus, tx_id, [0x00, 0x02])
                self.isotp_recv(bus, rx_id)
                
                self.files_listed.emit(entries)
                self.finished.emit(True, "Files listed successfully.")
                
            elif self.action == "download":
                if self.selected_file_index is None:
                    raise Exception("No file selected for download.")
                
                self.status.emit(f"Opening file index {self.selected_file_index}...")
                self.isotp_send(bus, tx_id, struct.pack("<BBH", 0x00, 0x22, self.selected_file_index))
                resp = self.isotp_recv(bus, rx_id)
                if not resp or resp[0] != 0x01 or resp[1] != 0x22:
                    if resp and resp[0] == 0x02:
                        raise Exception(f"LOGFS_OPEN NACK received: code {resp[2] if len(resp) > 2 else 0}")
                    raise Exception("Failed to open file on microSD card.")
                    
                handle = resp[2]
                file_size = struct.unpack("<I", resp[3:7])[0]
                expected_crc = struct.unpack("<I", resp[7:11])[0]
                
                self.status.emit(f"Downloading file content ({file_size:,} bytes)...")
                file_bytes = bytearray()
                offset = 0
                block_size = 256
                
                start_time = time.time()
                
                while offset < file_size:
                    req_payload = struct.pack("<BBBIH", 0x00, 0x23, handle, offset, block_size)
                    self.isotp_send(bus, tx_id, req_payload)
                    resp = self.isotp_recv(bus, rx_id)
                    
                    if not resp or resp[0] != 0x01 or resp[1] != 0x23:
                        if resp and resp[0] == 0x02:
                            raise Exception(f"LOGFS_READ NACK: code {resp[2] if len(resp) > 2 else 0}")
                        raise Exception("Failed to read file block from CAN.")
                        
                    chunk = resp[2:]
                    if not chunk:
                        break
                        
                    file_bytes.extend(chunk)
                    offset += len(chunk)
                    
                    elapsed = time.time() - start_time
                    speed = (offset / 1024.0) / elapsed if elapsed > 0 else 0.0
                    self.progress.emit(offset, file_size, speed)
                    
                self.isotp_send(bus, tx_id, [0x00, 0x25, handle])
                self.isotp_recv(bus, rx_id)
                
                self.isotp_send(bus, tx_id, [0x00, 0x02])
                self.isotp_recv(bus, rx_id)
                
                self.status.emit("Verifying file integrity...")
                actual_crc = zlib.crc32(file_bytes)
                if expected_crc != 0:
                    if actual_crc != expected_crc:
                        raise Exception(f"File integrity check failed! Expected CRC {expected_crc:08X}, got {actual_crc:08X}")
                    logger.info("[LOGFS] CRC32 verified: %08X", actual_crc)
                else:
                    logger.info("[LOGFS] Server returned CRC32=0 (bypassing pre-check CRC verification)")
                
                self.status.emit("Saving raw log to AMS_data folder...")
                ams_data_dir = Path("AMS_data")
                ams_data_dir.mkdir(exist_ok=True)
                out_path = ams_data_dir / (self.selected_file_name or "LOG.CSV")
                
                with open(out_path, 'wb') as f:
                    f.write(file_bytes)
                self.finished.emit(True, str(out_path.resolve()))
                
        except Exception as e:
            self.finished.emit(False, str(e))
        finally:
            if bus:
                try:
                    bus.shutdown()
                except Exception:
                    pass

    def isotp_send(self, bus, tx_id, payload):
        payload = bytes(payload)
        if len(payload) <= 7:
            data = bytearray(8)
            data[0] = len(payload) & 0x0F
            data[1:1+len(payload)] = payload
            for idx in range(1+len(payload), 8):
                data[idx] = 0xAA
            msg = can.Message(arbitration_id=tx_id, data=data, is_extended_id=False)
            bus.send(msg)
        else:
            data = bytearray(8)
            data[0] = 0x10 | ((len(payload) >> 8) & 0x0F)
            data[1] = len(payload) & 0xFF
            data[2:8] = payload[0:6]
            msg = can.Message(arbitration_id=tx_id, data=data, is_extended_id=False)
            bus.send(msg)
            
            fc_received = False
            rx_id = tx_id | 0x010
            start_t = time.time()
            while time.time() - start_t < 2.0:
                rx_msg = bus.recv(timeout=0.1)
                if rx_msg and rx_msg.arbitration_id == rx_id:
                    if (rx_msg.data[0] & 0xF0) == 0x30:
                        flow_status = rx_msg.data[0] & 0x0F
                        if flow_status == 0:
                            fc_received = True
                            break
                        elif flow_status == 2:
                            raise Exception("ISO-TP Overflow received")
            if not fc_received:
                raise Exception("Timeout waiting for ISO-TP Flow Control frame")
                
            seq = 1
            offset = 6
            while offset < len(payload):
                chunk = payload[offset : offset + 7]
                cf_data = bytearray(8)
                cf_data[0] = 0x20 | (seq & 0x0F)
                cf_data[1:1+len(chunk)] = chunk
                for idx in range(1+len(chunk), 8):
                    cf_data[idx] = 0xAA
                msg = can.Message(arbitration_id=tx_id, data=cf_data, is_extended_id=False)
                bus.send(msg)
                seq = (seq + 1) % 16
                offset += len(chunk)
                time.sleep(0.002)

    def isotp_recv(self, bus, rx_id, timeout=2.5):
        tx_id = rx_id & 0x00F
        start_t = time.time()
        buffer = bytearray()
        total_len = 0
        seq = 1
        
        while time.time() - start_t < timeout:
            msg = bus.recv(timeout=0.1)
            if not msg or msg.arbitration_id != rx_id:
                continue
                
            pci = msg.data[0]
            frame_type = pci & 0xF0
            
            if frame_type == 0x00 or pci <= 0x07:
                length = pci & 0x0F
                return bytes(msg.data[1:1+length])
                
            elif frame_type == 0x10:
                total_len = ((pci & 0x0F) << 8) | msg.data[1]
                buffer.extend(msg.data[2:8])
                fc_data = [0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
                fc_msg = can.Message(arbitration_id=tx_id, data=fc_data, is_extended_id=False)
                bus.send(fc_msg)
                seq = 1
                
            elif frame_type == 0x20:
                if total_len == 0:
                    continue
                chunk = msg.data[1:8]
                rem = total_len - len(buffer)
                chunk_len = min(7, rem)
                buffer.extend(chunk[:chunk_len])
                if len(buffer) >= total_len:
                    return bytes(buffer)
                seq = (seq + 1) % 16
                
        raise Exception("ISO-TP receive timeout")


class AMSSimulatorThread(QThread):
    def __init__(self, channel="test_channel"):
        super().__init__()
        self.channel = channel
        self.running = True

    def run(self):
        if not _CAN_OK:
            return
        try:
            bus = can.Bus(interface='virtual', channel=self.channel)
        except Exception:
            return
            
        simulated_files = {
            1: {
                "name": "LOG0001.CSV",
                "content": (
                    "tick_ms,fsm,mode,ams_ok,fault,detail,tsms,dash_chg,mod_mask,pack_mV,I_raw_mA,I_filt_mA,Idcdc_mA,dcbus_V,vmin_mV,vmax_mV,tmin_C,tmax_C,tavg_C,"
                    + ",".join(f"c0_{i}" for i in range(19)) + "," + ",".join(f"c1_{i}" for i in range(19)) + "," + ",".join(f"c2_{i}" for i in range(19)) + "," + ",".join(f"c3_{i}" for i in range(19)) + "," + ",".join(f"c4_{i}" for i in range(19)) + ","
                    + ",".join(f"t0_{i}" for i in range(40)) + "," + ",".join(f"t1_{i}" for i in range(40)) + "," + ",".join(f"t2_{i}" for i in range(40)) + "," + ",".join(f"t3_{i}" for i in range(40)) + "," + ",".join(f"t4_{i}" for i in range(40)) + "\n"
                    + "1000,2,1,1,0,0,1,0,31,380000,-15000,-14800,2000,380,3850,3950,25,28,26,"
                    + ",".join("3900" for _ in range(95)) + ","
                    + ",".join("26" for _ in range(200)) + "\n"
                    + "1250,2,1,1,0,0,1,0,31,380100,-14800,-14700,2010,380,3860,3960,25,28,26,"
                    + ",".join("3910" for _ in range(95)) + ","
                    + ",".join("26" for _ in range(200)) + "\n"
                    + "1500,2,1,1,0,0,1,0,31,380200,-14500,-14600,1990,380,3870,3970,25,28,26,"
                    + ",".join("3920" for _ in range(95)) + ","
                    + ",".join("26" for _ in range(200)) + "\n"
                ).encode('utf-8')
            },
            2: {
                "name": "LOG0002.CSV",
                "content": (
                    "tick_ms,fsm,mode,ams_ok,fault,detail,tsms,dash_chg,mod_mask,pack_mV,I_raw_mA,I_filt_mA,Idcdc_mA,dcbus_V,vmin_mV,vmax_mV,tmin_C,tmax_C,tavg_C,"
                    + ",".join(f"c0_{i}" for i in range(19)) + "," + ",".join(f"c1_{i}" for i in range(19)) + "," + ",".join(f"c2_{i}" for i in range(19)) + "," + ",".join(f"c3_{i}" for i in range(19)) + "," + ",".join(f"c4_{i}" for i in range(19)) + ","
                    + ",".join(f"t0_{i}" for i in range(40)) + "," + ",".join(f"t1_{i}" for i in range(40)) + "," + ",".join(f"t2_{i}" for i in range(40)) + "," + ",".join(f"t3_{i}" for i in range(40)) + "," + ",".join(f"t4_{i}" for i in range(40)) + "\n"
                    + "2000,2,1,1,0,0,1,0,31,379000,-10000,-10200,1800,379,3750,3850,26,29,27,"
                    + ",".join("3800" for _ in range(95)) + ","
                    + ",".join("27" for _ in range(200)) + "\n"
                    + "2250,2,1,1,0,0,1,0,31,379500,-9800,-10000,1820,379,3760,3860,26,29,27,"
                    + ",".join("3810" for _ in range(95)) + ","
                    + ",".join("27" for _ in range(200)) + "\n"
                ).encode('utf-8')
            }
        }
        
        session_connected = False
        open_handle = None
        open_file_idx = None
        
        rx_id = 0x002
        tx_id = 0x012
        
        buffer = bytearray()
        total_len = 0
        
        while self.running:
            try:
                msg = bus.recv(timeout=0.05)
            except Exception:
                break
            if not msg or msg.arbitration_id != rx_id:
                continue
                
            pci = msg.data[0]
            frame_type = pci & 0xF0
            payload = None
            
            if frame_type == 0x00 or pci <= 0x07:
                length = pci & 0x0F
                payload = bytes(msg.data[1:1+length])
            elif frame_type == 0x10:
                total_len = ((pci & 0x0F) << 8) | msg.data[1]
                buffer = bytearray(msg.data[2:8])
                fc = [0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
                bus.send(can.Message(arbitration_id=tx_id, data=fc, is_extended_id=False))
                continue
            elif frame_type == 0x20:
                if total_len == 0:
                    continue
                chunk = msg.data[1:8]
                rem = total_len - len(buffer)
                chunk_len = min(7, rem)
                buffer.extend(chunk[:chunk_len])
                if len(buffer) >= total_len:
                    payload = bytes(buffer)
                    total_len = 0
                else:
                    continue
                    
            if payload is None or len(payload) < 2:
                continue
                
            msg_type = payload[0]
            opcode = payload[1]
            
            if msg_type == 0x00:  # Cmd
                if opcode == 0x01:  # CONNECT
                    session_connected = True
                    self.send_isotp(bus, tx_id, [0x01, 0x01])
                elif opcode == 0x02:  # DISCONNECT
                    session_connected = False
                    self.send_isotp(bus, tx_id, [0x01, 0x02])
                elif session_connected:
                    if opcode == 0x21:  # LOGFS_LIST
                        entries_payload = bytearray([0x01, 0x21, 0x00, 0x00, 2])
                        for idx, info in simulated_files.items():
                            name_padded = info["name"].encode('utf-8').ljust(12, b'\x00')
                            size = len(info["content"])
                            entry = struct.pack("<HxxII12s", idx, size, 1718000000, name_padded)
                            entries_payload.extend(entry)
                        self.send_isotp(bus, tx_id, entries_payload)
                        
                    elif opcode == 0x22:  # LOGFS_OPEN
                        file_idx = struct.unpack("<H", payload[2:4])[0]
                        if file_idx in simulated_files:
                            open_handle = 0x42
                            open_file_idx = file_idx
                            size = len(simulated_files[file_idx]["content"])
                            crc32_val = zlib.crc32(simulated_files[file_idx]["content"])
                            resp = struct.pack("<BBBII", 0x01, 0x22, open_handle, size, crc32_val)
                            self.send_isotp(bus, tx_id, resp)
                        else:
                            self.send_isotp(bus, tx_id, [0x02, 0x22, 0x04])
                            
                    elif opcode == 0x23:  # LOGFS_READ
                        handle = payload[2]
                        offset = struct.unpack("<I", payload[3:7])[0]
                        length = struct.unpack("<H", payload[7:9])[0]
                        if handle == open_handle and open_file_idx in simulated_files:
                            content = simulated_files[open_file_idx]["content"]
                            chunk = content[offset : offset + length]
                            resp = bytearray([0x01, 0x23])
                            resp.extend(chunk)
                            self.send_isotp(bus, tx_id, resp)
                        else:
                            self.send_isotp(bus, tx_id, [0x02, 0x23, 0x08])
                            
                    elif opcode == 0x25:  # LOGFS_CLOSE
                        open_handle = None
                        open_file_idx = None
                        self.send_isotp(bus, tx_id, [0x01, 0x25])
        try:
            bus.shutdown()
        except Exception:
            pass

    def send_isotp(self, bus, tx_id, payload):
        payload = bytes(payload)
        if len(payload) <= 7:
            data = bytearray(8)
            data[0] = len(payload)
            data[1:1+len(payload)] = payload
            for idx in range(1+len(payload), 8):
                data[idx] = 0xAA
            bus.send(can.Message(arbitration_id=tx_id, data=data, is_extended_id=False))
        else:
            data = bytearray(8)
            data[0] = 0x10 | ((len(payload) >> 8) & 0x0F)
            data[1] = len(payload) & 0xFF
            data[2:8] = payload[0:6]
            bus.send(can.Message(arbitration_id=tx_id, data=data, is_extended_id=False))
            seq = 1
            offset = 6
            while offset < len(payload):
                chunk = payload[offset : offset + 7]
                cf_data = bytearray(8)
                cf_data[0] = 0x20 | (seq & 0x0F)
                cf_data[1:1+len(chunk)] = chunk
                for idx in range(1+len(chunk), 8):
                    cf_data[idx] = 0xAA
                bus.send(can.Message(arbitration_id=tx_id, data=cf_data, is_extended_id=False))
                seq = (seq + 1) % 16
                offset += len(chunk)
                time.sleep(0.002)


# ══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════════════════════════════════════
def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setFont(QFont("Segoe UI", 9))
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()