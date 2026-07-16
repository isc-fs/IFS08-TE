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
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Optional, Dict, List, Deque

import numpy as np
import matplotlib
matplotlib.use("Qt5Agg")

from PyQt5.QtCore import (
    QTimer, Qt, QMimeData, QObject, pyqtSignal, QPoint
)
from PyQt5.QtGui import (
    QFont, QPalette, QColor, QPixmap, QIcon,
    QPainter, QPen, QBrush, QLinearGradient, QDrag
)
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QGridLayout,
    QWidget, QLabel, QPushButton, QLineEdit, QComboBox, QTextEdit,
    QMessageBox, QTabWidget, QFrame, QGroupBox, QCheckBox,
    QListWidget, QListWidgetItem, QDialog, QSizePolicy, QInputDialog
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

def soc_from_cell_mv(cell_mv: float) -> float:
    """Estimate SoC (0–100 %) from minimum cell voltage using VTC6 OCV table."""
    cell_v = cell_mv / 1000.0
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
    'corriente_accu':       ('Pack Current',           'dA'),
    'corriente_dcdc':       ('DC-DC Current',          'dA'),
    'temp_dcdc':            ('DC-DC Temperature',      'degC'),
    # ── Battery / AMS — per-module max temperature ─────────────────────────
    'tmax_modulo_0':        ('Tmax Module 0',          'degC'),
    'tmax_modulo_1':        ('Tmax Module 1',          'degC'),
    'tmax_modulo_2':        ('Tmax Module 2',          'degC'),
    'tmax_modulo_3':        ('Tmax Module 3',          'degC'),
    'tmax_modulo_4':        ('Tmax Module 4',          'degC'),
    # ── Inverter status ───────────────────────────────────────────────────
    'inv_state':            ('Inverter State',         ''),
    'inv_vconfig_active':   ('Vconfig Active',         '0/1'),
    'inv_error':            ('Inverter Error',         ''),
    # ── Inverter electrical ───────────────────────────────────────────────
    'inv_dc_bus_V':         ('DC Bus Voltage',         'V'),
    'inv_temp_motor1':      ('Motor Temperature',      'degC'),
    'inv_temp_pwrstg':      ('Power Stage Temp',       'degC'),
    'inv_temp_board':       ('Inverter Board Temp',    'degC'),
    # ── Motor speed & current ─────────────────────────────────────────────
    'inv_rpm':              ('Motor Speed',            'RPM'),
    'inv_speed_actual':     ('Speed Feedback',         'RPM'),
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
        self._ax.tick_params(labelsize=6, colors='#555')
        self._ax.grid(True, alpha=0.3)
        for s in self._ax.spines.values():
            s.set_color('#2a2a2a')

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
        self.setMinimumSize(160, 160)

    def set_g_force(self, g_long: float, g_lat: float) -> None:
        self._gx = g_lat
        self._gy = g_long
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

        # G-dot
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
        self._ax.tick_params(labelsize=6, colors='#444')
        self._ax.grid(True, alpha=0.2)
        for s in self._ax.spines.values():
            s.set_color('#2a2a2a')
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
        self._ax.tick_params(labelsize=6, colors='#444')
        self._ax.grid(True, alpha=0.2)
        for s in self._ax.spines.values():
            s.set_color('#2a2a2a')
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

        # Coming-soon notice
        notice = QFrame()
        notice.setStyleSheet(
            f"QFrame {{ background:#2a1a00; border:1px solid {F1_WARNING}; border-radius:4px; }}")
        nl = QVBoxLayout(notice)
        nl.setContentsMargins(10, 8, 10, 8)
        nt = QLabel("⚠  AMS SD-card log format is not yet finalised.")
        nt.setStyleSheet(f"color:{F1_WARNING}; font-size:10px; font-weight:bold;")
        nd = QLabel(
            "Import will be enabled once the on-board AMS logger firmware\n"
            "and output format are defined.")
        nd.setStyleSheet("color:#888; font-size:9px;")
        nd.setWordWrap(True)
        nl.addWidget(nt)
        nl.addWidget(nd)
        v.addWidget(notice)

        # File selector (visible but disabled)
        file_row = QHBoxLayout()
        self._ams_file_lbl = QLabel("No file selected.")
        self._ams_file_lbl.setStyleSheet("color:#333; font-size:9px; font-family:'Courier New';")
        file_row.addWidget(self._ams_file_lbl, stretch=1)

        btn_browse = QPushButton("Browse…")
        btn_browse.setStyleSheet(
            f"background:{F1_MID_BG}; color:#555; border:1px solid #444; "
            f"padding:4px 10px; font-size:10px; border-radius:2px;")
        btn_browse.clicked.connect(self._browse_ams)
        file_row.addWidget(btn_browse)
        v.addLayout(file_row)

        v.addStretch()

        # Status
        self._ams_status = QLabel("Not yet implemented.")
        self._ams_status.setStyleSheet("color:#444; font-size:9px; font-family:'Courier New';")
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

        ok, msg = rtt.merge_gps_into_session(
            self._session_path, self._gps_file_path, utc_offset_hours=utc_off)

        if ok:
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

        ok, msg = rtt.merge_ams_temps_into_session(self._session_path, self._ams_file_path)

        if ok:
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
                ok, msg = rtt.merge_gps_into_session(
                    self._session_path, self._gps_file_path, utc_offset_hours=utc_off)
                if ok:
                    merged_any = True
                    gps_msg = f"GPS: {msg}\n"
                    self._log_append(f"[AUTO-MERGE] GPS integrated: {msg}")
                else:
                    self._log_append(f"[AUTO-MERGE] GPS failed: {msg}")
            
            if self._ams_file_path:
                ok, msg = rtt.merge_ams_temps_into_session(
                    self._session_path, self._ams_file_path)
                if ok:
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

        # Metric cards row (8 cards)
        cr = QHBoxLayout(); cr.setSpacing(6)
        self._ov_rpm    = MetricCard("RPM",         "rpm",  ISC_GREEN)
        self._ov_vbus   = MetricCard("DC BUS",      "V",    ISC_GREEN)
        self._ov_temp   = MetricCard("MAX TEMP",    "degC", F1_ERROR)
        self._ov_soc    = MetricCard("SOC (VTC6)",  "%",    F1_BLUE)
        self._ov_torque = MetricCard("TORQUE REQ",  "%",    ISC_GREEN)
        self._ov_cur    = MetricCard("MOTOR I",     "A",    F1_PURPLE)
        self._ov_vcell  = MetricCard("MIN CELL",    "mV",   F1_WARNING)
        self._ov_state  = MetricCard("INV STATE",   "",     ISC_GREEN)
        for c in (self._ov_rpm, self._ov_vbus, self._ov_temp, self._ov_soc,
                  self._ov_torque, self._ov_cur, self._ov_vcell, self._ov_state):
            cr.addWidget(c)
        v.addLayout(cr, stretch=2)

        # Rolling plots row: RPM | DC Bus | Max Temp | Throttle+Brake overlay
        pr = QHBoxLayout(); pr.setSpacing(6)
        self._ov_plot_rpm  = MplCanvas("Motor Speed  [RPM]",      ISC_GREEN)
        self._ov_plot_vbus = MplCanvas("DC Bus Voltage  [V]",     F1_WARNING)
        self._ov_plot_temp = MplCanvas("Max Battery Temp  [degC]",F1_ERROR)

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
        self._ax_tb.tick_params(labelsize=6, colors='#555')
        self._ax_tb.grid(True, alpha=0.3)
        self._ax_tb.legend(fontsize=6, loc='upper left',
                           facecolor=F1_PANEL_BG, labelcolor=F1_TEXT,
                           edgecolor='#333', framealpha=0.8)
        for sp in self._ax_tb.spines.values(): sp.set_color('#2a2a2a')
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
        for l in (self._ind_precharge, self._ind_inv_ok, self._ind_ams):
            l.setStyleSheet("color:#333; font-size:10px; font-weight:bold;")
        for l in (self._lbl_seq, self._lbl_tick):
            l.setStyleSheet("color:#444; font-size:9px; font-family:'Courier New';")
        self._lbl_lqi.setStyleSheet("color:#00c853; font-size:9px; font-family:'Courier New';")
        for w2 in (self._ind_precharge, self._ind_inv_ok, self._ind_ams,
                   self._lbl_seq, self._lbl_tick, self._signal_bars, self._lbl_lqi):
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

        # Inverter temps
        itb = QGroupBox("INVERTER TEMPERATURES")
        itg = QGridLayout(itb)
        self._pt_tm1   = MetricCard("Motor 1",   "°C", F1_WARNING)
        self._pt_tpwr  = MetricCard("PWRSTG",    "°C", F1_WARNING)
        self._pt_tbd   = MetricCard("Board",     "°C", ISC_GREEN)
        self._pt_tdcdc = MetricCard("DC-DC",     "°C", ISC_GREEN)
        itg.addWidget(self._pt_tm1,   0, 0); itg.addWidget(self._pt_tpwr, 0, 1)
        itg.addWidget(self._pt_tbd,   1, 0); itg.addWidget(self._pt_tdcdc,1, 1)
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
        bsg.addWidget(self._pt_vbus,  0, 0); bsg.addWidget(self._pt_soc,   0, 1)
        bsg.addWidget(self._pt_iaccu, 1, 0); bsg.addWidget(self._pt_idcdc, 1, 1)
        bsg.addWidget(self._pt_vcell, 2, 0); bsg.addWidget(self._pt_ams,   2, 1)
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

        self._vbox_t = QGroupBox(f"PER-MODULE MAX TEMPERATURE  [degC]   —   ALERT > {alert_temp:.0f} degC")
        vbt = QVBoxLayout(self._vbox_t)
        self._mod_t_bars: List[ModuleBarWidget] = []
        for i in range(5):
            b = ModuleBarWidget(i, "degC", lo=0, hi=80, warn_hi=alert_temp)
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
        box = QGroupBox("SYSTEM LOG")
        box.setStyleSheet(f"QGroupBox {{ color:{ISC_GREEN}; border:1px solid #222; }}")
        v = QVBoxLayout(box); v.setContentsMargins(4, 4, 4, 4)
        self._log = QTextEdit()
        self._log.setReadOnly(True)
        self._log.setMaximumHeight(80)
        self._log.setStyleSheet(
            f"background:{F1_DARK_BG}; color:{F1_TEXT}; "
            f"font-family:'Courier New'; font-size:9px; border:none;")
        v.addWidget(self._log)
        return box

    # ── Data access ───────────────────────────────────────────────────────────
    @staticmethod
    def _snap() -> dict:
        return rtt.get_latest_data().get('snapshot', {})

    # ── Alert checking ────────────────────────────────────────────────────────
    def _check_alerts(self, s: dict) -> None:
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
            alerts.append((f"BATTERY TEMP {max(valid_t):.0f} degC > {alert_temp:.0f} degC", 'critical'))
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

        # SoC from VTC6 OCV table (overrides raw ECU value if cell voltage available)
        soc_vtc6 = soc_from_cell_mv(vcell) if vcell > 0 else float(soc)

        self._ov_rpm.set_value(f"{int(rpm):,}")
        self._ov_vbus.set_value(f"{vbus}")
        self._ov_temp.set_value(f"{max_t:.0f}")
        self._ov_soc.set_value(f"{soc_vtc6:.1f}")
        self._ov_torque.set_value(f"{tpct}")
        self._ov_cur.set_value(f"{icur}")
        self._ov_vcell.set_value(f"{vcell}")
        self._ov_state.set_value(f"{istate}")

        self._mini_rpm.setText(f"{int(rpm):,} rpm")
        self._mini_vbus.setText(f"{vbus} V")
        self._mini_temp.setText(f"{max_t:.0f} degC")

        self._ov_plot_rpm.update_plot(rpm)
        self._ov_plot_vbus.update_plot(vbus)
        self._ov_plot_temp.update_plot(max_t)

        # Throttle + Brake overlay plot
        a1    = s.get('apps1_raw', 0)
        a2    = s.get('apps2_raw', 0)
        brk   = s.get('brake_raw', 0)
        thr_pct = max(a1, a2) / ADC_MAX * 100.0
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
        _ind(self._ind_inv_ok,    "INV OK",       ierr == 0 and istate > 0)
        _ind(self._ind_ams,       f"AMS {ams}",   ams > 0)
        self._lbl_seq.setText(f"SEQ: {seq}")
        self._lbl_tick.setText(f"TICK: {tick} ms")
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
        self._pt_speed.set_value(str(s.get('inv_speed_actual', 0)))
        self._pt_tm1.set_value(f"{s.get('inv_temp_motor1', 0)} degC")
        self._pt_tpwr.set_value(f"{s.get('inv_temp_pwrstg', 0)} degC")
        self._pt_tbd.set_value(f"{s.get('inv_temp_board', 0)} degC")
        self._pt_tdcdc.set_value(f"{s.get('temp_dcdc', 0)} degC")
        self._pt_vbus.set_value(f"{s.get('inv_dc_bus_V', 0)} V")
        vcell_pt = s.get('v_cell_min_mV', 0)
        soc_vtc6_pt = soc_from_cell_mv(vcell_pt) if vcell_pt > 0 else float(s.get('soc', 0))
        self._pt_soc.set_value(f"{soc_vtc6_pt:.1f} %")
        # corriente_accu is in dA; convert to A for display
        self._pt_iaccu.set_value(f"{s.get('corriente_accu', 0) / 10.0:.1f} A")
        self._pt_idcdc.set_value(f"{s.get('corriente_dcdc', 0) / 10.0:.1f} A")
        self._pt_vcell.set_value(f"{vcell_pt} mV")
        self._pt_ams.set_value(f"{s.get('ams_fsm_state', 0)}")

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
        # Normalise — use max of both APPS sensors for throttle
        self._ped_thr.set_value(max(a1, a2) / ADC_MAX, int(max(a1, a2)))
        self._ped_brk.set_value(brake / ADC_MAX,        int(brake))

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
            self._vbox_t.setTitle(f"PER-MODULE MAX TEMPERATURE  [degC]   —   ALERT > {temp_c:.0f} degC")

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
        if not self.settings.get("enable_tts", True):
            return
            
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

            # Speak at most once every 20 seconds per warning type
            last_time = self._spoken_alerts_timestamps.get(alert_key, 0.0)
            if now - last_time > 20.0:
                self._spoken_alerts_timestamps[alert_key] = now
                friendly_text = alert_text
                if "degC" in friendly_text:
                    friendly_text = friendly_text.replace("degC", "degrees Celsius")
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
        
        # 3. Recursively update all child widgets
        def _restyle(w):
            if isinstance(w, MetricCard):
                w._value.setStyleSheet(f"color:{F1_TEXT}; font-size:19px; font-weight:bold; background:transparent; border:none;")
                col = F1_ERROR if w._alerting else w._color
                w._set_border(col)
                w._title.setStyleSheet(f"color:{col}; font-size:9px; font-weight:bold; background:transparent; border:none;")
                w.update()
                
            elif isinstance(w, DroppablePlotPanel):
                w._title_lbl.setStyleSheet(f"color:#444; font-size:9px; background:transparent; border:none;")
                w._val_lbl.setStyleSheet(f"color:{ISC_GREEN}; font-size:15px; font-weight:bold; background:transparent; border:none;")
                w.setStyleSheet(f"QFrame {{ background:{F1_PANEL_BG}; color:{F1_TEXT}; border:1px solid #333; }}")
                w._ax.set_facecolor(F1_PANEL_BG)
                w._canvas.figure.patch.set_facecolor(F1_PANEL_BG)
                w._ax.spines['bottom'].set_color('#555' if F1_TEXT == '#1a1a1a' else '#333')
                w._ax.spines['left'].set_color('#555' if F1_TEXT == '#1a1a1a' else '#333')
                w._ax.tick_params(colors='#1a1a1a' if F1_TEXT == '#1a1a1a' else '#e0e0e0')
                w._canvas.draw_idle()
                
            elif isinstance(w, QTextEdit):
                w.setStyleSheet(f"background:{F1_PANEL_BG}; color:{F1_TEXT}; border:1px solid #333; font-family:'Courier New'; font-size:9px;")
                
            elif isinstance(w, QComboBox) or isinstance(w, QLineEdit):
                w.setStyleSheet(self.get_input_style())
                
            elif isinstance(w, AlertBanner):
                w.setStyleSheet(f"QFrame {{ background:{F1_PANEL_BG}; border:1px dashed #333; border-radius:4px; }}")
                
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