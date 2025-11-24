"""
ISCmetrics - ISC Formula Student Telemetry System
Developed by Andrés Sánchez de Ágreda © 2025/2026
Modified for F1-style modern aesthetic and improved layout
"""

from __future__ import annotations
import sys
import os
import threading
import time
import logging
from datetime import datetime
from typing import Optional
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Qt5Agg")

from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QObject
from PyQt5.QtGui import QFont, QPalette, QColor, QPixmap, QIcon
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QGridLayout,
    QWidget, QLabel, QPushButton, QLineEdit, QComboBox, QTextEdit,
    QMessageBox, QTabWidget, QFrame, QGroupBox, QCheckBox, QFileDialog,
    QListWidget, QSplitter, QScrollArea, QDialog
)

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

import ISC_RTT_serial as rtt
import ISC_RTT_demo as demo

# ============== CONSTANTS ==============
NUM_MODULES = 5
TEMPS_PER_MODULE = 38
CELLS_PER_MODULE = 19

# F1/MODERN COLOR SCHEME using Irish Green
ISC_GREEN = '#008000'
F1_DARK_BG = '#101010'
F1_MID_BG = '#282828'
F1_TEXT = '#FFFFFF'
F1_ACCENT = ISC_GREEN
F1_WARNING = '#FFFF00'
F1_ERROR = '#FF3333'

PLOT_COLOR = ISC_GREEN
PLOT_BG = F1_DARK_BG
WIDGET_BG = F1_MID_BG
TEXT_COLOR = F1_TEXT
ACCENT_COLOR = F1_ACCENT
WARNING_COLOR = F1_WARNING
ERROR_COLOR = F1_ERROR

current_settings = {
    "port": rtt.DEFAULT_PORT,
    "baud": rtt.DEFAULT_BAUD,
    "use_influx": rtt.INFLUX_ENABLE_DEFAULT,
    "debug": rtt.DEBUG_ENABLE_DEFAULT,
    "demo_mode": False,
}

# ============== LOGGING OVERRIDE ==============
class QtHandler(logging.Handler):
    """Custom logging handler to emit PyQt signals."""
    def __init__(self, signaler: 'Signaler'):
        super().__init__()
        self.signaler = signaler
        
    def emit(self, record):
        msg = self.format(record)
        self.signaler.log_message.emit(msg)

# ============== SIGNAL EMITTER ==============
class Signaler(QObject):
    """Thread-safe signal emitter"""
    new_data = pyqtSignal()
    log_message = pyqtSignal(str)

signaler = Signaler()

logger_rtt = logging.getLogger("ISC_RTT_USB")
logger_rtt.addHandler(QtHandler(signaler))

# ============== MATPLOTLIB F1 STYLE ==============
plt.style.use('dark_background')
plt.rcParams.update({
    'axes.facecolor': PLOT_BG,
    'figure.facecolor': PLOT_BG,
    'text.color': F1_TEXT,
    'axes.labelcolor': F1_TEXT,
    'xtick.color': F1_TEXT,
    'ytick.color': F1_TEXT,
    'axes.edgecolor': ISC_GREEN,
    'grid.color': F1_MID_BG,
    'grid.alpha': 0.5,
})

# ============== SETTINGS DIALOG ==============
class SettingsDialog(QDialog):
    def __init__(self, parent: 'MainWindow' = None):
        super().__init__(parent)
        self.setWindowTitle("ISCmetrics - Ajustes (Settings)")
        self.setWindowFlags(self.windowFlags() & ~Qt.WindowContextHelpButtonHint)
        self.setGeometry(200, 200, 450, 300)
        
        self.parent_ui = parent
        self.apply_f1_theme()
        
        layout = QGridLayout()
        layout.setSpacing(10)
        
        input_style = parent.get_input_style()
        label_style = f"color: {F1_TEXT}; font-size: 11px; font-weight: bold;"
        
        lbl_port = QLabel("COM Port:")
        lbl_port.setStyleSheet(label_style)
        self.combo_port = QComboBox()
        self.combo_port.setStyleSheet(input_style)
        self.refresh_ports()
        layout.addWidget(lbl_port, 0, 0)
        layout.addWidget(self.combo_port, 0, 1)
        
        lbl_baud = QLabel("Baud Rate:")
        lbl_baud.setStyleSheet(label_style)
        self.input_baud = QLineEdit(str(rtt.DEFAULT_BAUD))
        self.input_baud.setStyleSheet(input_style)
        layout.addWidget(lbl_baud, 1, 0)
        layout.addWidget(self.input_baud, 1, 1)

        self.chk_marple = QCheckBox("Enable Marple Logging")
        self.chk_marple.setStyleSheet(f"color: {F1_TEXT}; font-size: 11px;")
        self.chk_marple.setChecked(self.parent_ui.settings["use_influx"])
        layout.addWidget(self.chk_marple, 2, 0)
        
        self.chk_debug = QCheckBox("Enable Debug Output")
        self.chk_debug.setStyleSheet(f"color: {F1_TEXT}; font-size: 11px;")
        self.chk_debug.setChecked(self.parent_ui.settings["debug"])
        layout.addWidget(self.chk_debug, 3, 0)
        
        # ADDED: Demo Mode Checkbox
        self.chk_demo = QCheckBox("Enable Demo Mode (Simulated Data)")
        self.chk_demo.setStyleSheet(f"color: {F1_ACCENT}; font-size: 11px; font-weight: bold;")
        self.chk_demo.setChecked(self.parent_ui.settings["demo_mode"])
        layout.addWidget(self.chk_demo, 4, 0, 1, 2)
        
        btn_layout = QHBoxLayout()
        btn_ok = QPushButton("Apply & Close")
        btn_ok.setStyleSheet(parent.get_button_style('accent'))
        btn_ok.clicked.connect(self.accept)
        btn_layout.addWidget(btn_ok)
        
        layout.addLayout(btn_layout, 5, 0, 1, 2)
        self.setLayout(layout)

    def apply_f1_theme(self):
        palette = QPalette()
        palette.setColor(QPalette.Window, QColor(F1_DARK_BG))
        palette.setColor(QPalette.WindowText, QColor(F1_TEXT))
        self.setPalette(palette)
        self.setStyleSheet(f"QDialog {{ background-color: {F1_DARK_BG}; }}")

    def refresh_ports(self):
        self.combo_port.clear()
        ports = rtt.list_serial_ports()
        default_idx = -1
        for i, (port, desc) in enumerate(ports):
            display_text = f"{port} ({desc})"
            self.combo_port.addItem(display_text, port)
            if port == self.parent_ui.settings["port"]:
                 default_idx = i
        
        if ports:
            self.combo_port.setCurrentIndex(default_idx if default_idx != -1 else 0)
        
    def get_settings(self):
        """Returns the current settings from the dialog fields"""
        try:
            baud = int(self.input_baud.text())
        except ValueError:
            baud = self.parent_ui.settings["baud"]
        
        return {
            "port": self.combo_port.currentData(),
            "baud": baud,
            "use_influx": self.chk_marple.isChecked(),
            "debug": self.chk_debug.isChecked(),
            "demo_mode": self.chk_demo.isChecked(),
        }

# ============== MAIN WINDOW ==============
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ISCmetrics - Formula Student Telemetry")
        self.setGeometry(100, 100, 1920, 1080) 

        self.settings = current_settings.copy()
        self.demo_mode = self.settings["demo_mode"]
        
        icon_path_ico = Path("isc_logo.ico")
        if icon_path_ico.exists():
            self.setWindowIcon(QIcon(str(icon_path_ico)))

        self.log_text: Optional[QTextEdit] = None 
        
        self.init_ui()
        self.apply_f1_theme()
        
        self.rx_thread: Optional[threading.Thread] = None
        self.is_receiving = False
        
        self.settings_dialog: Optional[SettingsDialog] = None
        self.session_viewer: Optional['SessionViewerWindow'] = None
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_displays)
        self.timer.start(500)
        
        signaler.log_message.connect(self.append_log)
        
        self.append_log("UI Initialized. Ready to connect.")
        
        # Start demo if enabled in settings
        if self.demo_mode:
            self.activate_demo_mode()

    def get_input_style(self):
        """Reusable style for QLineEdit and QComboBox"""
        return f"background: {F1_MID_BG}; color: {F1_TEXT}; border: 1px solid {F1_ACCENT}; font-size: 14px; padding: 4px; border-radius: 2px;"

    def get_button_style(self, type='default'):
        """Reusable style for QPushButton"""
        if type == 'accent':
            bg = F1_ACCENT
            text_color = F1_DARK_BG
            border_style = 'none'
        else:
            bg = F1_MID_BG
            text_color = F1_ACCENT
            border_style = f'1px solid {F1_ACCENT}'

        return f"""
            QPushButton {{
                background: {bg};
                color: {text_color};
                border: {border_style};
                border-radius: 3px;
                padding: 6px 12px;
                font-size: 12px;
                font-weight: bold;
                min-width: 60px;
            }}
            QPushButton:hover {{
                background: {'#009900' if type=='accent' else F1_ACCENT};
                color: {F1_DARK_BG};
            }}
            QPushButton:disabled {{
                background: {F1_MID_BG};
                color: {F1_MID_BG};
                border: 1px solid {F1_MID_BG};
            }}
        """
    
    def apply_f1_theme(self):
        """Apply F1-style modern theme using ISC Green"""
        palette = QPalette()
        palette.setColor(QPalette.Window, QColor(F1_DARK_BG))
        palette.setColor(QPalette.WindowText, QColor(F1_TEXT))
        palette.setColor(QPalette.Base, QColor(F1_MID_BG))
        palette.setColor(QPalette.AlternateBase, QColor(F1_MID_BG))
        palette.setColor(QPalette.Text, QColor(F1_TEXT))
        palette.setColor(QPalette.Button, QColor(F1_MID_BG))
        palette.setColor(QPalette.ButtonText, QColor(F1_ACCENT))
        palette.setColor(QPalette.Highlight, QColor(F1_ACCENT))
        palette.setColor(QPalette.HighlightedText, QColor(F1_DARK_BG))
        self.setPalette(palette)
        
        self.setStyleSheet(f"""
            QMainWindow {{ background-color: {F1_DARK_BG}; }}
            QTabWidget::pane {{ border: 2px solid {F1_ACCENT}; border-top: none; background: {F1_DARK_BG}; }}
            QTabBar::tab {{
                background: {WIDGET_BG};
                color: {F1_TEXT};
                padding: 8px 20px;
                margin-right: 1px;
                border-top: 1px solid {F1_ACCENT};
                border-left: 1px solid {F1_ACCENT};
                border-right: 1px solid {F1_ACCENT};
            }}
            QTabBar::tab:selected {{
                background: {F1_ACCENT};
                color: {F1_DARK_BG};
                font-weight: bold;
            }}
            QLabel {{ color: {F1_TEXT}; }}
            QGroupBox {{ color: {F1_ACCENT}; border: 1px solid {F1_MID_BG}; margin-top: 10px; }}
            QGroupBox::title {{ subcontrol-origin: margin; subcontrol-position: top center; padding: 0 3px; background-color: {F1_DARK_BG}; color: {F1_ACCENT}; font-size: 11px; }}
        """)

    def init_ui(self):
        """Initialize main UI layout with new structure"""
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setSpacing(5)
        main_layout.setContentsMargins(10, 10, 10, 10)
        
        log_frame = self.create_single_log_frame()
        
        top_section = self.create_compact_top_section()
        main_layout.addWidget(top_section, stretch=1) 
        
        self.tabs = QTabWidget()
        self.tabs.setFont(QFont("Arial", 10, QFont.Bold))

        self.tabs.addTab(self.create_overview_tab(), "Overview")
        self.tabs.addTab(self.create_ams_tab(), "AMS Modules")
        self.tabs.addTab(self.create_motor_tab(), "Motor")
        self.tabs.addTab(self.create_driver_tab(), "Driver")
        self.tabs.addTab(self.create_accu_tab(), "Accumulator")
        
        main_layout.addWidget(self.tabs, stretch=8) 
        main_layout.addWidget(log_frame, stretch=1)
        
        attribution = QLabel("Andrés Sánchez de Ágreda © 2025/2026 - ICAI Racing Formula Student")
        attribution.setAlignment(Qt.AlignCenter)
        attribution.setStyleSheet(f"color: {F1_MID_BG}; font-size: 8px; padding: 2px;")
        main_layout.addWidget(attribution)

    def create_compact_top_section(self):
        """Create super compact top section."""
        container = QFrame()
        container.setStyleSheet(f"background: {F1_DARK_BG}; border: 1px solid {F1_MID_BG}; border-radius: 3px;")
        container.setMaximumHeight(85)
        
        main_layout = QHBoxLayout(container)
        main_layout.setSpacing(15)
        main_layout.setContentsMargins(5, 5, 5, 5)
        
        logo_section = self.create_logo_section_compact()
        main_layout.addWidget(logo_section, stretch=1) 
        
        config_section = self.create_session_config_section()
        main_layout.addWidget(config_section, stretch=3)
        
        self.status_label = QLabel("IDLE")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setMinimumWidth(100)
        self.status_label.setStyleSheet(f"""
            background: {F1_MID_BG};
            color: {F1_TEXT};
            font-size: 16px;
            font-weight: bold;
            padding: 8px 15px;
            border: 2px solid {F1_ACCENT};
            border-radius: 4px;
        """)
        main_layout.addWidget(self.status_label)
        
        buttons_section = self.create_buttons_section_compact()
        main_layout.addWidget(buttons_section, stretch=1)
        
        return container
    
    def create_logo_section_compact(self):
        """Minimal logo section with image and mini-metrics."""
        frame = QFrame()
        frame.setMinimumWidth(180)
        frame.setStyleSheet(f"background: {F1_DARK_BG}; border: none;")
        
        layout = QHBoxLayout(frame)
        layout.setSpacing(5)
        layout.setContentsMargins(3, 3, 3, 3)
        
        logo_label = QLabel()
        logo_path = Path(__file__).resolve().parent / "isc_logo.png"
        
        if logo_path.exists():
            pixmap = QPixmap(str(logo_path))
            if not pixmap.isNull():
                scaled_pixmap = pixmap.scaled(50, 50, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                logo_label.setPixmap(scaled_pixmap)
                logo_label.setMinimumSize(50, 50)
            else:
                logo_label.setText("ISC")
                logo_label.setStyleSheet(f"color: {F1_ACCENT}; font-size: 18px; font-weight: bold;")
        
        logo_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        layout.addWidget(logo_label)
        
        metrics_layout = QVBoxLayout()
        metrics_layout.setSpacing(0)
        
        app_name = QLabel("ISCmetrics")
        app_name.setStyleSheet(f"color: {F1_ACCENT}; font-size: 14px; font-weight: bold;")
        metrics_layout.addWidget(app_name)
        
        self.mini_speed = QLabel("0.0 km/h")
        self.mini_voltage = QLabel("0.0 V")
        self.mini_temp = QLabel("0 °C")
        
        for lbl in [self.mini_speed, self.mini_voltage, self.mini_temp]:
            lbl.setStyleSheet(f"color: {F1_TEXT}; font-size: 9px;")
            metrics_layout.addWidget(lbl)
        
        layout.addLayout(metrics_layout)
        return frame

    def create_session_config_section(self):
        """Piloto/Circuito selection section"""
        frame = QFrame()
        layout = QGridLayout(frame)
        layout.setSpacing(5)
        layout.setContentsMargins(5, 5, 5, 5)
        
        label_style = f"color: {F1_ACCENT}; font-size: 12px; font-weight: bold;"
        
        lbl_pilot = QLabel("PILOT:")
        lbl_pilot.setStyleSheet(label_style)
        layout.addWidget(lbl_pilot, 0, 0)
        
        self.input_pilot = QLineEdit("Piloto_Test")
        self.input_pilot.setStyleSheet(self.get_input_style())
        self.input_pilot.setMinimumWidth(200)
        layout.addWidget(self.input_pilot, 0, 1)
        
        lbl_circuit = QLabel("CIRCUIT:")
        lbl_circuit.setStyleSheet(label_style)
        layout.addWidget(lbl_circuit, 1, 0)
        
        self.input_circuit = QLineEdit("Circuito_Test")
        self.input_circuit.setStyleSheet(self.get_input_style())
        self.input_circuit.setMinimumWidth(200)
        layout.addWidget(self.input_circuit, 1, 1)
        
        return frame

    def create_buttons_section_compact(self):
        """MODIFIED: Compact action buttons without demo button"""
        frame = QFrame()
        layout = QGridLayout(frame)
        layout.setSpacing(4)
        layout.setContentsMargins(0, 0, 0, 0)
        
        self.btn_start = QPushButton("START")
        self.btn_start.setStyleSheet(self.get_button_style('accent'))
        self.btn_start.clicked.connect(self.start_reception)
        layout.addWidget(self.btn_start, 0, 0)
        
        self.btn_stop = QPushButton("STOP")
        self.btn_stop.setStyleSheet(self.get_button_style())
        self.btn_stop.setEnabled(False)
        self.btn_stop.clicked.connect(self.stop_reception)
        layout.addWidget(self.btn_stop, 0, 1)
        
        self.btn_settings = QPushButton("Ajustes")
        self.btn_settings.setStyleSheet(self.get_button_style())
        self.btn_settings.clicked.connect(self.open_settings)
        layout.addWidget(self.btn_settings, 1, 0)
        
        btn_sessions = QPushButton("Sessions")
        btn_sessions.setStyleSheet(self.get_button_style())
        btn_sessions.clicked.connect(self.open_session_viewer)
        layout.addWidget(btn_sessions, 1, 1)
        
        return frame

    def activate_demo_mode(self):
        """Activate demo mode"""
        self.demo_mode = True
        self.append_log("[DEMO] Demo mode ENABLED - Using simulated data")
        demo.start_demo()
        self.btn_start.setEnabled(False)
        self.status_label.setText("TEST")
        self.status_label.setStyleSheet(f"""
            background: {F1_ACCENT}; 
            color: {F1_DARK_BG}; 
            font-size: 16px; 
            font-weight: bold; 
            padding: 8px 15px; 
            border-radius: 4px;
            border: 2px solid {F1_ACCENT};
        """)
    
    def deactivate_demo_mode(self):
        """Deactivate demo mode"""
        self.demo_mode = False
        self.append_log("[DEMO] Demo mode DISABLED - Ready for real data")
        demo.stop_demo()
        if not self.is_receiving:
            self.btn_start.setEnabled(True)
        self.status_label.setText("IDLE")
        self.status_label.setStyleSheet(f"""
            background: {F1_MID_BG}; 
            color: {F1_TEXT}; 
            font-size: 16px; 
            font-weight: bold; 
            padding: 8px 15px; 
            border-radius: 4px;
            border: 2px solid {F1_MID_BG};
        """)

    def open_settings(self):
        """Opens the Settings dialog and updates internal settings."""
        self.settings_dialog = SettingsDialog(self)
        
        if self.settings_dialog.exec_():
            new_settings = self.settings_dialog.get_settings()
            
            # Check if demo mode changed
            demo_changed = new_settings["demo_mode"] != self.settings["demo_mode"]
            
            self.settings.update(new_settings)
            
            global current_settings
            current_settings.update(new_settings)
            rtt.DEFAULT_PORT = new_settings['port']
            rtt.DEFAULT_BAUD = new_settings['baud']
            rtt.INFLUX_ENABLE_DEFAULT = new_settings['use_influx']
            rtt.DEBUG_ENABLE_DEFAULT = new_settings['debug']

            self.append_log(f"Settings updated: Port={self.settings['port']}, Baud={self.settings['baud']}, Marple={self.settings['use_influx']}, Debug={self.settings['debug']}, Demo={self.settings['demo_mode']}")
            
            # Handle demo mode changes
            if demo_changed:
                if new_settings["demo_mode"]:
                    self.activate_demo_mode()
                else:
                    self.deactivate_demo_mode()

    def create_overview_tab(self):
        """Create overview dashboard with greater vertical space for graphs."""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        metrics_grid = QGridLayout()
        metrics_grid.setSpacing(6)
        
        self.lbl_dc_bus = self.create_metric_label("DC BUS", "--- V", F1_ACCENT, compact=True)
        metrics_grid.addWidget(self.lbl_dc_bus, 0, 0)
        self.lbl_rpm = self.create_metric_label("RPM", "---", F1_ACCENT, compact=True)
        metrics_grid.addWidget(self.lbl_rpm, 0, 1)
        self.lbl_torque = self.create_metric_label("TORQUE", "--- Nm", F1_ACCENT, compact=True)
        metrics_grid.addWidget(self.lbl_torque, 0, 2)
        self.lbl_current = self.create_metric_label("CURRENT", "--- A", F1_ACCENT, compact=True)
        metrics_grid.addWidget(self.lbl_current, 0, 3)
        
        self.lbl_min_cell = self.create_metric_label("MIN CELL V", "--- mV", F1_WARNING, compact=True)
        metrics_grid.addWidget(self.lbl_min_cell, 1, 0)
        self.lbl_stack = self.create_metric_label("STACK V", "--- V", F1_ACCENT, compact=True)
        metrics_grid.addWidget(self.lbl_stack, 1, 1)
        self.lbl_max_temp = self.create_metric_label("MAX TEMP", "--- °C", F1_ERROR, compact=True)
        metrics_grid.addWidget(self.lbl_max_temp, 1, 2)
        self.lbl_throttle = self.create_metric_label("THROTTLE", "--- %", F1_ACCENT, compact=True)
        metrics_grid.addWidget(self.lbl_throttle, 1, 3)
        
        layout.addLayout(metrics_grid, stretch=2) 
        
        plot_layout = QHBoxLayout()
        self.plot_rpm = MplCanvas(title="RPM History", color=F1_ACCENT)
        plot_layout.addWidget(self.plot_rpm)
        self.plot_voltage = MplCanvas(title="Min Cell Voltage (mV)", color=F1_WARNING)
        plot_layout.addWidget(self.plot_voltage)
        self.plot_temp = MplCanvas(title="Max Temperature (°C)", color=F1_ERROR)
        plot_layout.addWidget(self.plot_temp)
        layout.addLayout(plot_layout, stretch=5) 
        
        return widget
    
    def create_ams_tab(self):
        widget = QWidget()
        layout = QVBoxLayout()
        
        global_stats = QGroupBox("GLOBAL BATTERY METRICS")
        global_layout = QGridLayout()
        
        self.lbl_global_min = QLabel("Global Min: --- mV")
        self.lbl_global_max = QLabel("Global Max: --- mV")
        self.lbl_stack_total = QLabel("Stack Total: --- V")
        self.lbl_ams_current = QLabel("Current: --- A")
        
        for lbl in [self.lbl_global_min, self.lbl_global_max, self.lbl_stack_total, self.lbl_ams_current]:
             lbl.setStyleSheet(f"color: {F1_TEXT}; font-size: 12px; font-weight: bold; padding: 5px;")
        
        global_layout.addWidget(self.lbl_global_min, 0, 0)
        global_layout.addWidget(self.lbl_global_max, 0, 1)
        global_layout.addWidget(self.lbl_stack_total, 1, 0)
        global_layout.addWidget(self.lbl_ams_current, 1, 1)
        
        global_stats.setLayout(global_layout)
        layout.addWidget(global_stats)
        
        modules_group = QGroupBox("MODULE SUMMARY")
        modules_layout = QHBoxLayout()
        self.module_cards = []
        
        for i in range(NUM_MODULES):
            card = self.create_module_card(i)
            modules_layout.addWidget(card)
            self.module_cards.append(card)
        
        modules_group.setLayout(modules_layout)
        layout.addWidget(modules_group)
        
        widget.setLayout(layout)
        return widget
    
    def create_module_card(self, module_id):
        card = QGroupBox(f"MODULE {module_id}")
        card.setStyleSheet(f"""
            QGroupBox {{ font-size: 10px; color: {F1_ACCENT}; border: 1px solid {F1_ACCENT}; background: {F1_MID_BG}; }}
            QGroupBox::title {{ color: {F1_ACCENT}; }}
        """)
        
        layout = QVBoxLayout()
        
        lbl_min_v = QLabel("Min V: --- mV")
        lbl_max_v = QLabel("Max V: --- mV")
        lbl_min_t = QLabel("Min T: --- °C")
        lbl_max_t = QLabel("Max T: --- °C")
        lbl_age = QLabel("Age: ---")
        
        for lbl in [lbl_min_v, lbl_max_v, lbl_min_t, lbl_max_t, lbl_age]:
            lbl.setStyleSheet(f"font-size: 10px; color: {F1_TEXT};")
        
        layout.addWidget(lbl_min_v)
        layout.addWidget(lbl_max_v)
        layout.addWidget(lbl_min_t)
        layout.addWidget(lbl_max_t)
        layout.addWidget(lbl_age)
        
        card.setLayout(layout)
        
        card.lbl_min_v = lbl_min_v
        card.lbl_max_v = lbl_max_v
        card.lbl_min_t = lbl_min_t
        card.lbl_max_t = lbl_max_t
        card.lbl_age = lbl_age
        
        return card

    def create_motor_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        metrics_grid = QGridLayout()
        metrics_grid.setSpacing(6)
        
        self.motor_lbl_rpm = self.create_metric_label("RPM", "---", F1_ACCENT, compact=True)
        metrics_grid.addWidget(self.motor_lbl_rpm, 0, 0)
        self.motor_lbl_torque = self.create_metric_label("TORQUE REQ", "--- Nm", F1_ACCENT, compact=True)
        metrics_grid.addWidget(self.motor_lbl_torque, 0, 1)
        self.motor_lbl_current = self.create_metric_label("ACTUAL CURRENT", "--- A", F1_ACCENT, compact=True)
        metrics_grid.addWidget(self.motor_lbl_current, 1, 0)
        self.motor_lbl_temp = self.create_metric_label("MOTOR TEMP", "--- °C", F1_WARNING, compact=True)
        metrics_grid.addWidget(self.motor_lbl_temp, 1, 1)
        
        layout.addLayout(metrics_grid, stretch=2)
        
        plot_layout = QHBoxLayout()
        self.motor_plot_torque = MplCanvas(title="Total Torque (Nm)", color=F1_ACCENT)
        self.motor_plot_current = MplCanvas(title="Actual Current (A)", color=F1_ACCENT)
        plot_layout.addWidget(self.motor_plot_torque)
        plot_layout.addWidget(self.motor_plot_current)
        layout.addLayout(plot_layout, stretch=5)
        
        return widget

    def create_driver_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        metrics_grid = QGridLayout()
        metrics_grid.setSpacing(6)
        
        self.driver_lbl_throttle = self.create_metric_label("THROTTLE", "--- %", F1_ACCENT, compact=True)
        metrics_grid.addWidget(self.driver_lbl_throttle, 0, 0)
        self.driver_lbl_brake = self.create_metric_label("BRAKE", "--- %", F1_ACCENT, compact=True)
        metrics_grid.addWidget(self.driver_lbl_brake, 0, 1)
        self.driver_lbl_s1 = self.create_metric_label("APPS 1 (Raw)", "---", F1_WARNING, compact=True)
        metrics_grid.addWidget(self.driver_lbl_s1, 1, 0)
        self.driver_lbl_s2 = self.create_metric_label("APPS 2 (Raw)", "---", F1_WARNING, compact=True)
        metrics_grid.addWidget(self.driver_lbl_s2, 1, 1)
        
        layout.addLayout(metrics_grid, stretch=2)
        
        plot_layout = QHBoxLayout()
        self.driver_plot_throttle = MplCanvas(title="Throttle Position %", color=F1_ACCENT)
        self.driver_plot_brake = MplCanvas(title="Brake Pedal Position %", color=F1_ERROR)
        plot_layout.addWidget(self.driver_plot_throttle)
        plot_layout.addWidget(self.driver_plot_brake)
        layout.addLayout(plot_layout, stretch=5)
        
        return widget
    
    def create_accu_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        stats_layout = QHBoxLayout()
        self.accu_lbl_stack = self.create_metric_label("STACK V", "--- V", F1_ACCENT, compact=True)
        self.accu_lbl_current = self.create_metric_label("CURRENT", "--- A", F1_ACCENT, compact=True)
        self.accu_lbl_min_cell = self.create_metric_label("MIN CELL", "--- mV", F1_WARNING, compact=True)
        self.accu_lbl_max_temp = self.create_metric_label("MAX TEMP", "--- °C", F1_ERROR, compact=True)
        
        stats_layout.addWidget(self.accu_lbl_stack)
        stats_layout.addWidget(self.accu_lbl_current)
        stats_layout.addWidget(self.accu_lbl_min_cell)
        stats_layout.addWidget(self.accu_lbl_max_temp)
        layout.addLayout(stats_layout, stretch=1)
        
        heatmap_layout = QGridLayout()
        self.accu_heatmaps = []
        for i in range(NUM_MODULES):
            heatmap = HeatmapCanvas(title=f"MODULE {i}")
            heatmap_layout.addWidget(heatmap, i // 3, i % 3)
            self.accu_heatmaps.append(heatmap)
        
        layout.addLayout(heatmap_layout, stretch=4)
        
        return widget
    
    def create_metric_label(self, title, value, color, compact=False):
        """Create a styled metric display label with F1 aesthetic"""
        frame = QFrame()
        frame.setStyleSheet(f"""
            QFrame {{
                background: {F1_MID_BG};
                border: 2px solid {color};
                border-radius: 4px;
                padding: {'3px' if compact else '6px'};
            }}
        """)
        
        layout = QVBoxLayout()
        layout.setSpacing(1)
        
        title_lbl = QLabel(title)
        title_lbl.setAlignment(Qt.AlignCenter)
        title_lbl.setStyleSheet(f"color: {color}; font-size: {'10px' if compact else '12px'}; font-weight: bold;")
        
        value_lbl = QLabel(value)
        value_lbl.setAlignment(Qt.AlignCenter)
        value_lbl.setStyleSheet(f"color: {F1_TEXT}; font-size: {'18px' if compact else '22px'}; font-weight: bold;")
        
        layout.addWidget(title_lbl)
        layout.addWidget(value_lbl)
        frame.setLayout(layout)
        
        frame.value_label = value_lbl
        return frame
    
    def create_single_log_frame(self):
        """Create single consolidated log frame at the bottom."""
        frame = QGroupBox("SYSTEM LOG & DEBUG DATA")
        frame.setStyleSheet(f"QGroupBox {{ font-size: 10px; color: {F1_ACCENT}; border: 1px solid {F1_ACCENT}; }}")
        
        layout = QVBoxLayout()
        layout.setContentsMargins(5, 5, 5, 5)
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(120)
        self.log_text.setStyleSheet(f"""
            background: {F1_DARK_BG}; 
            color: {F1_TEXT}; 
            font-family: 'Courier New'; 
            font-size: 10px; 
            border: 1px solid {F1_ACCENT};
        """)
        layout.addWidget(self.log_text)
        
        frame.setLayout(layout)
        return frame
    
    def refresh_ports(self):
        """Refreshes ports in the settings dialog (if open) and updates the default setting."""
        ports = rtt.list_serial_ports()
        
        if ports:
            if self.settings["port"] not in [p[0] for p in ports]:
                self.settings["port"] = ports[0][0]
                self.append_log(f"Port auto-selected: {self.settings['port']}")
        
        if self.settings_dialog and self.settings_dialog.isVisible():
            self.settings_dialog.refresh_ports()
        
    def start_reception(self):
        """Start data reception thread"""
        if self.is_receiving or self.demo_mode: 
            return
        
        piloto = self.input_pilot.text()
        circuito = self.input_circuit.text()
        port = self.settings["port"]
        use_influx = self.settings["use_influx"]
        debug = self.settings["debug"]
        
        try:
            baud = int(self.settings["baud"])
        except ValueError:
            QMessageBox.critical(self, "Error", "Invalid baudrate in settings.")
            return
        
        if not port:
            QMessageBox.warning(self, "Error", "No COM port selected. Check Ajustes.")
            return
        
        bucket_id = rtt.create_bucket(piloto, circuito, use_influx=use_influx)
        self.append_log(f"Starting: {port} @ {baud} bps. Marple logging: {use_influx}")
        self.append_log(f"Session ID: {bucket_id}")
        
        self.is_receiving = True
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.btn_settings.setEnabled(False)
        
        def rx_worker():
            try:
                rtt.receive_data(
                    bucket_id=bucket_id,
                    piloto=piloto,
                    circuito=circuito,
                    port=port,
                    baud=baud,
                    use_influx=use_influx,
                    debug=debug
                )
            except Exception as e:
                signaler.log_message.emit(f"FATAL ERROR IN RX THREAD: {e}")
            finally:
                self.is_receiving = False

        self.rx_thread = threading.Thread(target=rx_worker, daemon=True)
        self.rx_thread.start()
        
    def stop_reception(self):
        """Stop data reception"""
        if not self.is_receiving: return
        
        self.append_log("Stopping reception...")
        rtt.new_data_flag = -1
        if self.rx_thread and self.rx_thread.is_alive():
            self.rx_thread.join(timeout=1.0)
            
        self.is_receiving = False
        if not self.demo_mode:
            self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.btn_settings.setEnabled(True)

    def update_displays(self):
        """Update all displays with latest data"""
        if self.demo_mode:
            data = demo.get_latest_data()
            status_info = {'badge': 'LIVE'}
        else:
            data = rtt.get_latest_data()
            status_info = data.get("__STATUS__", {'badge': 'IDLE', 'reason': 'no data', 'ts': 0})
        
        if self.is_receiving and rtt.new_data_flag == -1:
             self.stop_reception()
        
        badge = status_info.get("badge", "IDLE" if not self.is_receiving and not self.demo_mode else "STALE")
        style = f"background: {F1_MID_BG}; font-size: 16px; font-weight: bold; padding: 8px 15px; border-radius: 4px;"
        
        if badge == "LIVE" or self.demo_mode:
            self.status_label.setText("LIVE" if not self.demo_mode else "DEMO")
            self.status_label.setStyleSheet(style + f"color: {F1_ACCENT}; border: 2px solid {F1_ACCENT};")
        elif badge == "STALE":
            self.status_label.setText("STALE")
            self.status_label.setStyleSheet(style + f"color: {F1_WARNING}; border: 2px solid {F1_WARNING};")
        elif badge == "BAD":
            self.status_label.setText("BAD")
            self.status_label.setStyleSheet(style + f"color: {F1_ERROR}; border: 2px solid {F1_ERROR};")
        else:
             self.status_label.setText("IDLE")
             self.status_label.setStyleSheet(style + f"color: {F1_TEXT}; border: 2px solid {F1_MID_BG};")

        if self.demo_mode:
            data_600 = data.get(0x600, {})
            data_610 = data.get(0x610, {})
            data_620 = data.get(0x620, {})
            data_630 = data.get(0x630, {})
            data_201 = data.get(0x201, {})
            data_202 = data.get(0x202, {})
            data_208 = data.get(0x208, {})
            
            rpm = data_600.get('rpm', 0)
            dc_bus = data_600.get('dcbusvoltage', 0)
            torque = data_600.get('torquetotal', 0)
            cell_min_v = data_600.get('cellminv', 0)
            throttle = data_630.get('throttle', 0)
            current = data_610.get('iactual', 0)
            stack_mv = data_202.get('stacktotalmv', 0)
            max_temp = data_208.get('maxtempc', 0)
            brake = data_630.get('brake', 0)
            s1_raw = data_620.get('s1raw', 0)
            s2_raw = data_620.get('s2raw', 0)
            motor_temp = data_610.get('motortemp', 0)
            
        else:
            data_600 = data.get("0x600", {})
            data_610 = data.get("0x610", {})
            data_620 = data.get("0x620", {})
            data_630 = data.get("0x630", {})
            
            rpm = data_600.get('rpm', 0)
            dc_bus = data_600.get('dc_bus_voltage', 0)
            torque = data_600.get('torque_total', 0)
            cell_min_v = data_600.get('cell_min_v', 0)
            throttle = data_630.get('throttle', 0)
            current = data_610.get('i_actual', 0)
            
            ams_summary = data.get("ams_summary", {})
            stack_mv = ams_summary.get('stack_mv', 0)
            
            ams_temp = data.get("ams_temp_summary", {})
            max_temp = ams_temp.get('max_temp_c', 0)
            
            brake = data_630.get('brake', 0)
            s1_raw = data_620.get('s1_raw', 0)
            s2_raw = data_620.get('s2_raw', 0)
            motor_temp = data_610.get('motor_temp', 0)
        
        self.lbl_dc_bus.value_label.setText(f"{dc_bus:.1f} V")
        self.lbl_rpm.value_label.setText(f"{rpm:.0f}")
        self.lbl_torque.value_label.setText(f"{torque:.1f} Nm")
        self.lbl_min_cell.value_label.setText(f"{cell_min_v:.0f} mV")
        self.lbl_throttle.value_label.setText(f"{throttle:.1f} %")
        self.lbl_current.value_label.setText(f"{current:.1f} A")
        self.lbl_stack.value_label.setText(f"{stack_mv / 1000:.1f} V")
        self.lbl_max_temp.value_label.setText(f"{max_temp:.0f} °C")
        
        speed_kmh = rpm * 0.05
        self.mini_speed.setText(f"{speed_kmh:.1f} km/h")
        self.mini_voltage.setText(f"{dc_bus:.1f} V")
        self.mini_temp.setText(f"{max_temp:.0f} °C")
        
        if self.demo_mode:
            self.lbl_global_min.setText(f"Global Min: {data_202.get('mincellmv', 0):.0f} mV")
            self.lbl_global_max.setText(f"Global Max: {data_202.get('maxcellmv', 0):.0f} mV")
            self.lbl_stack_total.setText(f"Stack Total: {data_202.get('stacktotalmv', 0) / 1000:.1f} V")
            self.lbl_ams_current.setText(f"Current: {data_201.get('currentdA', 0) / 10:.1f} A")
        else:
            self.lbl_global_min.setText(f"Global Min: {rtt.ams_global_min_mv} mV")
            self.lbl_global_max.setText(f"Global Max: {rtt.ams_global_max_mv} mV")
            self.lbl_stack_total.setText(f"Stack Total: {rtt.ams_stack_total_mv / 1000:.1f} V")
            self.lbl_ams_current.setText(f"Current: {rtt.ams_current_dA / 10:.1f} A")
        
        now = time.time()
        for i, card in enumerate(self.module_cards):
            if self.demo_mode:
                mod = demo.get_ams_module_data(i)
                if mod:
                    voltages = mod.get('voltages', [])
                    temps = mod.get('tempsc', [])
                    if voltages and temps:
                        card.lbl_min_v.setText(f"Min: {min(voltages):.0f} mV")
                        card.lbl_max_v.setText(f"Max: {max(voltages):.0f} mV")
                        card.lbl_min_t.setText(f"Min T: {min(temps):.0f} °C")
                        card.lbl_max_t.setText(f"Max T: {max(temps):.0f} °C")
                        card.lbl_age.setText(f"Age: 0.0s")
            else:
                mod = rtt.get_ams_module_data(i)
                if mod:
                    card.lbl_min_v.setText(f"Min: {mod.min_cell_mv} mV")
                    card.lbl_max_v.setText(f"Max: {mod.max_cell_mv} mV")
                    card.lbl_min_t.setText(f"Min T: {mod.min_temp_c:.0f} °C")
                    card.lbl_max_t.setText(f"Max T: {mod.max_temp_c:.0f} °C")
                    age = now - mod.last_update_ts
                    card.lbl_age.setText(f"Age: {age:.1f}s")
        
        self.motor_lbl_rpm.value_label.setText(f"{rpm:.0f}")
        self.motor_lbl_torque.value_label.setText(f"{torque:.1f} Nm")
        self.motor_lbl_current.value_label.setText(f"{current:.1f} A")
        self.motor_lbl_temp.value_label.setText(f"{motor_temp:.0f} °C")
        
        self.driver_lbl_throttle.value_label.setText(f"{throttle:.1f} %")
        self.driver_lbl_brake.value_label.setText(f"{brake:.1f} %")
        self.driver_lbl_s1.value_label.setText(f"{s1_raw:.0f}")
        self.driver_lbl_s2.value_label.setText(f"{s2_raw:.0f}")
        
        self.plot_rpm.update_plot(rpm)
        self.plot_voltage.update_plot(cell_min_v)
        self.plot_temp.update_plot(max_temp)
        
        self.motor_plot_torque.update_plot(torque)
        self.motor_plot_current.update_plot(current)
        
        self.driver_plot_throttle.update_plot(throttle)
        self.driver_plot_brake.update_plot(brake)
        
        self.accu_lbl_stack.value_label.setText(f"{stack_mv / 1000:.1f} V")
        if self.demo_mode:
            self.accu_lbl_current.value_label.setText(f"{data_201.get('currentdA', 0) / 10:.1f} A")
            self.accu_lbl_min_cell.value_label.setText(f"{data_202.get('mincellmv', 0):.0f} mV")
        else:
            self.accu_lbl_current.value_label.setText(f"{rtt.ams_current_dA / 10:.1f} A")
            self.accu_lbl_min_cell.value_label.setText(f"{rtt.ams_global_min_mv} mV")
        self.accu_lbl_max_temp.value_label.setText(f"{max_temp:.0f} °C")
        
        for i, heatmap in enumerate(self.accu_heatmaps):
            if self.demo_mode:
                mod = demo.get_ams_module_data(i)
                if mod:
                    heatmap.update_heatmap(mod.get('tempsc', []))
            else:
                mod = rtt.get_ams_module_data(i)
                if mod:
                    heatmap.update_heatmap(mod.temps_c)
        
        if not self.demo_mode and rtt.new_data_flag == 1:
            self.append_log(rtt.data_str)
            rtt.new_data_flag = 0

    def append_log(self, msg: str):
        """Append message to log window"""
        if self.log_text:
            ts = datetime.now().strftime("%H:%M:%S")
            self.log_text.append(f"[{ts}] {msg}")
            
            doc = self.log_text.document()
            max_blocks = 20
            while doc.blockCount() > max_blocks:
                cursor = self.log_text.textCursor()
                cursor.movePosition(cursor.Start)
                cursor.select(cursor.BlockUnderCursor)
                cursor.removeSelectedText()
                cursor.deleteChar()
            
    def open_log_viewer(self):
        QMessageBox.information(self, "Log Viewer", "The system log and debug output are now consolidated at the bottom of the main window.")
        
    def open_session_viewer(self):
        if self.session_viewer is None or not self.session_viewer.isVisible():
            self.session_viewer = SessionViewerWindow()
            self.session_viewer.show()
    
    def export_current_session(self):
        QMessageBox.information(self, "Export", "Export feature coming soon!")
    
    def closeEvent(self, event):
        if self.is_receiving:
            self.stop_reception()
            time.sleep(0.5)
        if self.demo_mode:
            demo.stop_demo()
        event.accept()

# ============== SESSION VIEWER WINDOW ==============
class SessionViewerWindow(QWidget):
    """Window to view and analyze past Excel sessions"""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Session Viewer")
        self.setGeometry(150, 150, 1400, 800)
        self.current_session_data = {}
        
        self.init_ui()
        self.apply_f1_theme()
        
    def apply_f1_theme(self):
         palette = QPalette()
         palette.setColor(QPalette.Window, QColor(F1_DARK_BG))
         palette.setColor(QPalette.WindowText, QColor(F1_TEXT))
         self.setPalette(palette)
         self.setStyleSheet(f"QWidget {{ background-color: {F1_DARK_BG}; color: {F1_TEXT}; }}")
        
    def init_ui(self):
        layout = QHBoxLayout()
        
        left_panel = QWidget()
        left_layout = QVBoxLayout()
        
        title = QLabel("Sessions")
        title.setStyleSheet(f"font-size: 14px; font-weight: bold; color: {F1_ACCENT}; padding: 8px;")
        left_layout.addWidget(title)
        
        self.session_list = QListWidget()
        self.session_list.setStyleSheet(f"background: {F1_MID_BG}; color: {F1_TEXT}; border: 1px solid {F1_ACCENT}; font-size: 10px;")
        self.session_list.itemClicked.connect(self.load_session)
        left_layout.addWidget(self.session_list)
        
        btn_refresh = QPushButton("Refresh")
        btn_refresh.setStyleSheet(MainWindow.get_button_style(self, 'default'))
        btn_refresh.clicked.connect(self.refresh_session_list)
        left_layout.addWidget(btn_refresh)
        
        left_panel.setLayout(left_layout)
        left_panel.setMaximumWidth(300)
        
        right_panel = QWidget()
        right_layout = QVBoxLayout()
        
        self.session_info_label = QLabel("Select a session")
        self.session_info_label.setStyleSheet(f"font-size: 12px; color: {F1_TEXT}; padding: 8px;")
        right_layout.addWidget(self.session_info_label)
        
        self.data_tabs = QTabWidget()
        self.data_tabs.setStyleSheet(f"""
            QTabWidget::pane {{ border: 1px solid {F1_ACCENT}; background: {F1_DARK_BG}; }}
            QTabBar::tab {{ background: {F1_MID_BG}; color: {F1_TEXT}; padding: 6px 12px; border: 1px solid {F1_ACCENT}; }}
            QTabBar::tab:selected {{ background: {F1_ACCENT}; color: {F1_DARK_BG}; }}
        """)
        
        right_layout.addWidget(self.data_tabs)
        right_panel.setLayout(right_layout)
        
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)
        splitter.setStretchFactor(1, 3)
        
        layout.addWidget(splitter)
        self.setLayout(layout)
        
        self.refresh_session_list()
    
    def refresh_session_list(self):
        self.session_list.clear()
        sessions = rtt.list_excel_sessions()
        
        for session_file in sessions:
            mod_time = datetime.fromtimestamp(session_file.stat().st_mtime)
            display_text = f"{session_file.stem}\n  {mod_time.strftime('%Y-%m-%d %H:%M')}"
            item = QtWidgets.QListWidgetItem(display_text)
            item.setData(Qt.UserRole, session_file)
            self.session_list.addItem(item)
    
    def load_session(self, item):
        session_file = item.data(Qt.UserRole)
        
        try:
            self.session_info_label.setText(f"Loading: {session_file.name}...")
            self.current_session_data = rtt.load_excel_session(session_file)
            
            if not self.current_session_data:
                self.session_info_label.setText(f"Error loading")
                return
            
            if 'Metadata' in self.current_session_data:
                meta = self.current_session_data['Metadata']
                if len(meta) > 0:
                    info_text = f"{meta['Piloto'].iloc[0]} @ {meta['Circuito'].iloc[0]} ({meta['Duration (min)'].iloc[0]:.1f} min)"
                    self.session_info_label.setText(info_text)
            
            self.data_tabs.clear()
            
            for sheet_name, df in self.current_session_data.items():
                if sheet_name == 'Metadata':
                    continue
                tab = self.create_data_tab(sheet_name, df)
                self.data_tabs.addTab(tab, sheet_name)
        
        except Exception as e:
            self.session_info_label.setText(f"Error: {str(e)}")
    
    def create_data_tab(self, sheet_name, df):
        widget = QWidget()
        layout = QVBoxLayout()
        
        info = QLabel(f"{sheet_name}: {len(df)} records")
        info.setStyleSheet(f"color: {F1_ACCENT}; font-size: 10px; padding: 3px;")
        layout.addWidget(info)
        
        if 'timestamp' in df.columns:
            df['timestamp'] = pd.to_datetime(df['timestamp'])
            
            plot_layout = QGridLayout()
            
            numeric_cols = [col for col in df.columns if pd.api.types.is_numeric_dtype(df[col]) and col not in ['module_id']]
            
            if len(numeric_cols) >= 1:
                canvas1 = self.create_timeseries_plot(df, numeric_cols[0], numeric_cols[0])
                plot_layout.addWidget(canvas1, 0, 0)
            
            if len(numeric_cols) >= 2:
                canvas2 = self.create_timeseries_plot(df, numeric_cols[1], numeric_cols[1])
                plot_layout.addWidget(canvas2, 0, 1)

            layout.addLayout(plot_layout)
        
        else:
            text = QTextEdit()
            text.setReadOnly(True)
            text.setText(df.head(50).to_string())
            text.setStyleSheet(f"background: {F1_DARK_BG}; color: {F1_TEXT}; font-family: monospace; font-size: 9px;")
            layout.addWidget(text)
        
        widget.setLayout(layout)
        return widget
    
    def create_timeseries_plot(self, df, column, title):
        fig = Figure(figsize=(6, 3), facecolor=PLOT_BG)
        ax = fig.add_subplot(111)
        ax.set_facecolor(PLOT_BG)
        ax.set_title(title, color=F1_TEXT, fontweight='bold', fontsize=10)
        ax.tick_params(colors=F1_TEXT, labelsize=8)
        
        ax.plot(df['timestamp'], df[column], color=F1_ACCENT, linewidth=1.5)
        ax.grid(True, alpha=0.3, color=F1_MID_BG)
        ax.tick_params(axis='x', rotation=45)
        for spine in ax.spines.values():
            spine.set_color(F1_ACCENT)
        fig.tight_layout()
        
        canvas = FigureCanvas(fig)
        return canvas

# ============== MATPLOTLIB CANVAS ==============
class MplCanvas(FigureCanvas):
    def __init__(self, title="Plot", max_points=100, color=F1_ACCENT):
        figsize = (4, 2.2)
        self.fig = Figure(figsize=figsize, facecolor=PLOT_BG)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor(PLOT_BG)
        self.ax.set_title(title, color=F1_TEXT, fontweight='bold', fontsize=11)
        self.ax.tick_params(colors=F1_TEXT, labelsize=9)
        for spine in self.ax.spines.values():
            spine.set_color(F1_ACCENT)
        
        super().__init__(self.fig)
        
        self.data = []
        self.max_points = max_points
        self.line, = self.ax.plot([], [], color=color, linewidth=1.5)
        self.ax.grid(True, alpha=0.3, color=F1_MID_BG)
        self.fig.tight_layout()
    
    def update_plot(self, value):
        self.data.append(value)
        if len(self.data) > self.max_points:
            self.data.pop(0)
        
        self.line.set_data(range(len(self.data)), self.data)
        self.ax.relim()
        self.ax.autoscale_view()
        self.draw()

# ============== HEATMAP CANVAS ==============
class HeatmapCanvas(FigureCanvas):
    def __init__(self, title="Heatmap"):
        self.fig = Figure(figsize=(3, 3), facecolor=PLOT_BG)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor(PLOT_BG)
        self.ax.set_title(title, color=F1_TEXT, fontsize=10, fontweight='bold')
        super().__init__(self.fig)
        self.grid_data = np.zeros((7, 6)) 
        self.im = self.ax.imshow(self.grid_data, cmap='viridis', vmin=20, vmax=60, aspect='auto')
        
        cbar = self.fig.colorbar(self.im, ax=self.ax)
        cbar.set_label('°C', color=F1_TEXT)
        cbar.ax.yaxis.set_tick_params(color=F1_TEXT)
        plt.setp(plt.getp(cbar.ax.axes, 'yticklabels'), color=F1_TEXT)
        
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.fig.tight_layout()
    
    def update_heatmap(self, temps):
        temps_array = np.array(temps[:TEMPS_PER_MODULE])
        temps_array = np.nan_to_num(temps_array, nan=20.0)
        
        target_size = 42
        if len(temps_array) < target_size:
             padded = np.pad(temps_array, (0, target_size - len(temps_array)), constant_values=20.0)
        else:
             padded = temps_array[:target_size]
             
        grid = padded.reshape((7, 6))
        self.im.set_data(grid)
        self.im.set_clim(vmin=max(20, grid[grid > 20].min() if grid[grid > 20].size > 0 else 20), 
                         vmax=min(60, grid.max()))
        self.draw()

# ============== MAIN ==============
def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
