"""
ISCmetrics - ISC Formula Student Telemetry System
Developed by Andrés Sánchez de Ágreda © 2025/2026
"""

from __future__ import annotations
import sys
import os
import threading
import time
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
    QListWidget, QSplitter, QScrollArea
)

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

import ISC_RTT_serial as rtt

# ============== CONSTANTS ==============
NUM_MODULES = 5
TEMPS_PER_MODULE = 38
CELLS_PER_MODULE = 19

# ============== ISC COLOR SCHEME ==============
ISC_GREEN = '#002D0C'  # RGB(0, 45, 12)
ISC_LIGHT_GREEN = '#00b894'  # Lighter green for accents
ISC_DARK_BG = '#0a0a0a'  # Very dark background
ISC_WIDGET_BG = '#1a1a1a'  # Widget background
ISC_TEXT = '#e0e0e0'  # Light text
ISC_WARNING = '#fdcb6e'  # Yellow for warnings
ISC_ERROR = '#d63031'  # Red for errors

PLOT_BG = ISC_DARK_BG
WIDGET_BG = ISC_WIDGET_BG
TEXT_COLOR = ISC_TEXT
ACCENT_COLOR = ISC_LIGHT_GREEN
WARNING_COLOR = ISC_WARNING
ERROR_COLOR = ISC_ERROR

# ============== SIGNAL EMITTER ==============
class Signaler(QObject):
    """Thread-safe signal emitter"""
    new_data = pyqtSignal()
    log_message = pyqtSignal(str)

signaler = Signaler()

# ============== MATPLOTLIB DARK STYLE ==============
plt.style.use('dark_background')

# ============== MAIN WINDOW ==============
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ISCmetrics - Formula Student Telemetry")
        self.setGeometry(100, 100, 1700, 900)
        
        # Set window icon if available
        icon_path = Path("isc_logo.ico")
        if icon_path.exists():
            self.setWindowIcon(QIcon(str(icon_path)))
        
        # Apply ISC theme
        self.apply_isc_theme()
        
        # Data receiving thread
        self.rx_thread: Optional[threading.Thread] = None
        self.is_receiving = False
        
        # Sub-windows
        self.motor_window: Optional[MotorInverterWindow] = None
        self.accu_window: Optional[AccumulatorWindow] = None
        self.driver_window: Optional[DriverWindow] = None
        self.log_viewer: Optional[LogViewerWindow] = None
        self.session_viewer: Optional[SessionViewerWindow] = None
        
        # Build UI
        self.init_ui()
        
        # Update timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_displays)
        self.timer.start(100)  # 10 Hz refresh
        
        # Connect signals
        signaler.log_message.connect(self.append_log)
    
    def apply_isc_theme(self):
        """Apply ISC green color theme"""
        palette = QPalette()
        palette.setColor(QPalette.Window, QColor(ISC_DARK_BG))
        palette.setColor(QPalette.WindowText, QColor(ISC_TEXT))
        palette.setColor(QPalette.Base, QColor(ISC_WIDGET_BG))
        palette.setColor(QPalette.AlternateBase, QColor(30, 30, 30))
        palette.setColor(QPalette.ToolTipBase, QColor(ISC_TEXT))
        palette.setColor(QPalette.ToolTipText, QColor(ISC_TEXT))
        palette.setColor(QPalette.Text, QColor(ISC_TEXT))
        palette.setColor(QPalette.Button, QColor(0, 45, 12))  # ISC Green
        palette.setColor(QPalette.ButtonText, QColor(ISC_TEXT))
        palette.setColor(QPalette.BrightText, QColor(255, 255, 255))
        palette.setColor(QPalette.Highlight, QColor(ISC_LIGHT_GREEN))
        palette.setColor(QPalette.HighlightedText, QColor(0, 0, 0))
        self.setPalette(palette)
    
    def init_ui(self):
        """Initialize main UI layout"""
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setSpacing(5)
        
        # === TOP: Logo and Status Bar ===
        header_layout = QHBoxLayout()
        
        # Logo and branding section
        logo_section = self.create_logo_section()
        header_layout.addWidget(logo_section)
        
        # Status banner (smaller, centered, expanded)
        self.status_label = QLabel("⚪ IDLE")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet(f"""
            background: {ISC_GREEN};
            color: {ISC_TEXT};
            font-size: 12px;
            font-weight: bold;
            padding: 8px;
            border: 2px solid {ISC_LIGHT_GREEN};
            border-radius: 5px;
        """)
        header_layout.addWidget(self.status_label, stretch=2)
        
        main_layout.addLayout(header_layout)
        
        # === CONTROL PANEL (Compact) ===
        control_panel = self.create_compact_control_panel()
        main_layout.addWidget(control_panel)
        
        # === MIDDLE: Tab widget ===
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet(f"""
            QTabWidget::pane {{
                border: 1px solid {ISC_LIGHT_GREEN};
                background: {WIDGET_BG};
            }}
            QTabBar::tab {{
                background: {ISC_GREEN};
                color: {TEXT_COLOR};
                padding: 6px 14px;
                margin-right: 2px;
                border: 1px solid {ISC_LIGHT_GREEN};
            }}
            QTabBar::tab:selected {{
                background: {ISC_LIGHT_GREEN};
                color: {ISC_DARK_BG};
                font-weight: bold;
            }}
        """)
        
        # Tab 1: Overview Dashboard
        self.overview_tab = self.create_overview_tab()
        self.tabs.addTab(self.overview_tab, "📊 Overview")
        
        # Tab 2: AMS Module Details
        self.ams_tab = self.create_ams_tab()
        self.tabs.addTab(self.ams_tab, "🔋 AMS Modules")
        
        main_layout.addWidget(self.tabs, stretch=3)
        
        # === BOTTOM: Mini Log Window ===
        log_frame = self.create_mini_log_frame()
        main_layout.addWidget(log_frame, stretch=1)
        
        # === ATTRIBUTION ===
        attribution = QLabel("Andrés Sánchez de Ágreda © 2025/2026 - ICAI Racing Formula Student")
        attribution.setAlignment(Qt.AlignCenter)
        attribution.setStyleSheet(f"color: {ISC_LIGHT_GREEN}; font-size: 9px; padding: 3px;")
        main_layout.addWidget(attribution)
    
    def create_logo_section(self):
        """Create logo and ISCmetrics branding section"""
        frame = QFrame()
        frame.setStyleSheet(f"""
            QFrame {{
                background: {ISC_GREEN};
                border: 2px solid {ISC_LIGHT_GREEN};
                border-radius: 8px;
                padding: 8px;
            }}
        """)
        
        layout = QVBoxLayout()
        layout.setSpacing(3)
        
        # Logo
        logo_label = QLabel()
        logo_path = Path("isc_logo.ico")
        
        if logo_path.exists():
            pixmap = QPixmap(str(logo_path))
            scaled_pixmap = pixmap.scaled(80, 80, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            logo_label.setPixmap(scaled_pixmap)
        else:
            logo_label.setText("ISC Logo")
            logo_label.setStyleSheet(f"color: {TEXT_COLOR}; font-size: 12px;")
        
        logo_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(logo_label)
        
        # App name: ISCmetrics
        app_name = QLabel("ISCmetrics")
        app_name.setAlignment(Qt.AlignCenter)
        app_name.setStyleSheet(f"""
            color: {ISC_LIGHT_GREEN};
            font-size: 12px;
            font-weight: bold;
            letter-spacing: 1px;
        """)
        layout.addWidget(app_name)
        
        # Divider line
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setStyleSheet(f"background-color: {ISC_LIGHT_GREEN};")
        line.setMaximumHeight(2)
        layout.addWidget(line)
        
        # Metrics summary (compact)
        metrics_layout = QVBoxLayout()
        metrics_layout.setSpacing(2)
        
        self.mini_speed = QLabel("Speed: --- km/h")
        self.mini_voltage = QLabel("Voltage: --- V")
        self.mini_temp = QLabel("Temp: --- °C")
        
        for lbl in [self.mini_speed, self.mini_voltage, self.mini_temp]:
            lbl.setStyleSheet(f"color: {TEXT_COLOR}; font-size: 9px;")
            lbl.setAlignment(Qt.AlignCenter)
            metrics_layout.addWidget(lbl)
        
        layout.addLayout(metrics_layout)
        frame.setLayout(layout)
        frame.setMaximumWidth(140)
        return frame
    
    def create_compact_control_panel(self):
        """Create compact control panel"""
        panel = QGroupBox("Control Panel")
        panel.setStyleSheet(f"""
            QGroupBox {{
                border: 2px solid {ISC_LIGHT_GREEN};
                border-radius: 5px;
                margin-top: 5px;
                font-weight: bold;
                background: {ISC_GREEN};
                color: {ISC_TEXT};
            }}
            QGroupBox::title {{
                color: {ISC_LIGHT_GREEN};
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }}
        """)
        
        layout = QHBoxLayout()
        layout.setSpacing(10)
        
        # Config inputs
        config_layout = QGridLayout()
        config_layout.setSpacing(5)
        
        # Style for labels
        label_style = f"color: {ISC_TEXT}; font-size: 10px;"
        
        lbl_pilot = QLabel("Piloto:")
        lbl_pilot.setStyleSheet(label_style)
        config_layout.addWidget(lbl_pilot, 0, 0)
        self.input_pilot = QLineEdit("Piloto_Test")
        self.input_pilot.setMaximumWidth(120)
        self.input_pilot.setStyleSheet(f"background: {WIDGET_BG}; color: {TEXT_COLOR}; border: 1px solid {ISC_LIGHT_GREEN};")
        config_layout.addWidget(self.input_pilot, 0, 1)
        
        lbl_circuit = QLabel("Circuito:")
        lbl_circuit.setStyleSheet(label_style)
        config_layout.addWidget(lbl_circuit, 0, 2)
        self.input_circuit = QLineEdit("Circuito_Test")
        self.input_circuit.setMaximumWidth(120)
        self.input_circuit.setStyleSheet(f"background: {WIDGET_BG}; color: {TEXT_COLOR}; border: 1px solid {ISC_LIGHT_GREEN};")
        config_layout.addWidget(self.input_circuit, 0, 3)
        
        lbl_port = QLabel("Puerto:")
        lbl_port.setStyleSheet(label_style)
        config_layout.addWidget(lbl_port, 1, 0)
        self.combo_port = QComboBox()
        self.combo_port.setMaximumWidth(200)
        self.combo_port.setStyleSheet(f"background: {WIDGET_BG}; color: {TEXT_COLOR}; border: 1px solid {ISC_LIGHT_GREEN};")
        self.refresh_ports()
        config_layout.addWidget(self.combo_port, 1, 1, 1, 2)
        
        lbl_baud = QLabel("Baud:")
        lbl_baud.setStyleSheet(label_style)
        config_layout.addWidget(lbl_baud, 1, 3)
        self.input_baud = QLineEdit("115200")
        self.input_baud.setMaximumWidth(80)
        self.input_baud.setStyleSheet(f"background: {WIDGET_BG}; color: {TEXT_COLOR}; border: 1px solid {ISC_LIGHT_GREEN};")
        config_layout.addWidget(self.input_baud, 1, 4)
        
        # InfluxDB and Debug options
        lbl_influx = QLabel("InfluxDB:")
        lbl_influx.setStyleSheet(label_style)
        config_layout.addWidget(lbl_influx, 0, 4)
        self.chk_influx = QCheckBox("Enable")
        self.chk_influx.setStyleSheet(f"color: {TEXT_COLOR};")
        self.chk_influx.setChecked(False)
        config_layout.addWidget(self.chk_influx, 0, 5)
        
        lbl_debug = QLabel("Debug:")
        lbl_debug.setStyleSheet(label_style)
        config_layout.addWidget(lbl_debug, 0, 6)
        self.chk_debug = QCheckBox("Enable")
        self.chk_debug.setStyleSheet(f"color: {TEXT_COLOR};")
        self.chk_debug.setChecked(False)
        config_layout.addWidget(self.chk_debug, 0, 7)
        
        layout.addLayout(config_layout, stretch=3)
        
        # Button style
        btn_style = f"""
            QPushButton {{
                background: {ISC_GREEN};
                color: {ISC_TEXT};
                border: 1px solid {ISC_LIGHT_GREEN};
                border-radius: 3px;
                padding: 4px 8px;
                font-size: 10px;
            }}
            QPushButton:hover {{
                background: {ISC_LIGHT_GREEN};
                color: {ISC_DARK_BG};
            }}
            QPushButton:pressed {{
                background: #008866;
            }}
        """
        
        btn_accent_style = f"""
            QPushButton {{
                background: {ISC_LIGHT_GREEN};
                color: {ISC_DARK_BG};
                border: none;
                border-radius: 3px;
                padding: 4px 8px;
                font-size: 10px;
                font-weight: bold;
            }}
            QPushButton:hover {{
                background: #00cc9f;
            }}
            QPushButton:pressed {{
                background: #008866;
            }}
        """
        
        # Buttons column 1
        btn_layout1 = QVBoxLayout()
        btn_layout1.setSpacing(3)
        
        self.btn_refresh = QPushButton("🔄 Refresh")
        self.btn_refresh.setMaximumHeight(28)
        self.btn_refresh.setStyleSheet(btn_style)
        self.btn_refresh.clicked.connect(self.refresh_ports)
        btn_layout1.addWidget(self.btn_refresh)
        
        self.btn_start = QPushButton("▶ Start")
        self.btn_start.setMaximumHeight(28)
        self.btn_start.setStyleSheet(btn_accent_style)
        self.btn_start.clicked.connect(self.start_reception)
        btn_layout1.addWidget(self.btn_start)
        
        self.btn_stop = QPushButton("⏹ Stop")
        self.btn_stop.setMaximumHeight(28)
        self.btn_stop.setStyleSheet(btn_style)
        self.btn_stop.setEnabled(False)
        self.btn_stop.clicked.connect(self.stop_reception)
        btn_layout1.addWidget(self.btn_stop)
        
        layout.addLayout(btn_layout1)
        
        # Buttons column 2 - Data windows
        btn_layout2 = QVBoxLayout()
        btn_layout2.setSpacing(3)
        
        btn_motor = QPushButton("🔧 Motor")
        btn_motor.setMaximumHeight(28)
        btn_motor.setStyleSheet(btn_style)
        btn_motor.clicked.connect(self.open_motor_window)
        btn_layout2.addWidget(btn_motor)
        
        btn_accu = QPushButton("🔋 Accu")
        btn_accu.setMaximumHeight(28)
        btn_accu.setStyleSheet(btn_style)
        btn_accu.clicked.connect(self.open_accu_window)
        btn_layout2.addWidget(btn_accu)
        
        btn_driver = QPushButton("🏎️ Driver")
        btn_driver.setMaximumHeight(28)
        btn_driver.setStyleSheet(btn_style)
        btn_driver.clicked.connect(self.open_driver_window)
        btn_layout2.addWidget(btn_driver)
        
        layout.addLayout(btn_layout2)
        
        # Buttons column 3 - Tools
        btn_layout3 = QVBoxLayout()
        btn_layout3.setSpacing(3)
        
        btn_logs = QPushButton("📄 Logs")
        btn_logs.setMaximumHeight(28)
        btn_logs.setStyleSheet(btn_style)
        btn_logs.clicked.connect(self.open_log_viewer)
        btn_layout3.addWidget(btn_logs)
        
        btn_sessions = QPushButton("📊 Sessions")
        btn_sessions.setMaximumHeight(28)
        btn_sessions.setStyleSheet(btn_accent_style)
        btn_sessions.clicked.connect(self.open_session_viewer)
        btn_layout3.addWidget(btn_sessions)
        
        btn_export = QPushButton("💾 Export")
        btn_export.setMaximumHeight(28)
        btn_export.setStyleSheet(btn_style)
        btn_export.clicked.connect(self.export_current_session)
        btn_layout3.addWidget(btn_export)
        
        layout.addLayout(btn_layout3)
        
        panel.setLayout(layout)
        return panel
    
    def create_overview_tab(self):
        """Create overview dashboard with key metrics"""
        widget = QWidget()
        layout = QVBoxLayout()
        
        # Key metrics grid (smaller boxes)
        metrics_grid = QGridLayout()
        metrics_grid.setSpacing(5)
        
        # Row 1
        self.lbl_dc_bus = self.create_metric_label("DC Bus", "--- V", ISC_LIGHT_GREEN, compact=True)
        metrics_grid.addWidget(self.lbl_dc_bus, 0, 0)
        
        self.lbl_rpm = self.create_metric_label("RPM", "---", ISC_LIGHT_GREEN, compact=True)
        metrics_grid.addWidget(self.lbl_rpm, 0, 1)
        
        self.lbl_torque = self.create_metric_label("Torque", "--- Nm", ISC_LIGHT_GREEN, compact=True)
        metrics_grid.addWidget(self.lbl_torque, 0, 2)
        
        self.lbl_current = self.create_metric_label("Current", "--- A", ISC_LIGHT_GREEN, compact=True)
        metrics_grid.addWidget(self.lbl_current, 0, 3)
        
        # Row 2
        self.lbl_min_cell = self.create_metric_label("Min Cell", "--- mV", WARNING_COLOR, compact=True)
        metrics_grid.addWidget(self.lbl_min_cell, 1, 0)
        
        self.lbl_stack = self.create_metric_label("Stack", "--- V", ISC_LIGHT_GREEN, compact=True)
        metrics_grid.addWidget(self.lbl_stack, 1, 1)
        
        self.lbl_max_temp = self.create_metric_label("Max Temp", "--- °C", WARNING_COLOR, compact=True)
        metrics_grid.addWidget(self.lbl_max_temp, 1, 2)
        
        self.lbl_throttle = self.create_metric_label("Throttle", "--- %", ISC_LIGHT_GREEN, compact=True)
        metrics_grid.addWidget(self.lbl_throttle, 1, 3)
        
        layout.addLayout(metrics_grid)
        
        # Plots
        plot_layout = QHBoxLayout()
        
        self.plot_rpm = MplCanvas(title="RPM History", compact=True)
        plot_layout.addWidget(self.plot_rpm)
        
        self.plot_voltage = MplCanvas(title="Min Cell Voltage", compact=True)
        plot_layout.addWidget(self.plot_voltage)
        
        self.plot_temp = MplCanvas(title="Max Temperature", compact=True)
        plot_layout.addWidget(self.plot_temp)
        
        layout.addLayout(plot_layout)
        
        widget.setLayout(layout)
        return widget
    
    def create_ams_tab(self):
        """Create AMS module overview tab"""
        widget = QWidget()
        layout = QVBoxLayout()
        
        # Module summary
        summary_label = QLabel("AMS Module Summary")
        summary_label.setStyleSheet(f"font-size: 14px; font-weight: bold; color: {ISC_LIGHT_GREEN};")
        layout.addWidget(summary_label)
        
        # Create module cards
        modules_layout = QHBoxLayout()
        self.module_cards = []
        
        for i in range(NUM_MODULES):
            card = self.create_module_card(i)
            modules_layout.addWidget(card)
            self.module_cards.append(card)
        
        layout.addLayout(modules_layout)
        
        # Global AMS stats
        global_stats = QGroupBox("Global Statistics")
        global_stats.setStyleSheet(f"""
            QGroupBox {{
                border: 2px solid {ISC_LIGHT_GREEN};
                border-radius: 5px;
                background: {ISC_GREEN};
                color: {ISC_TEXT};
            }}
            QGroupBox::title {{
                color: {ISC_LIGHT_GREEN};
            }}
        """)
        global_layout = QGridLayout()
        
        self.lbl_global_min = QLabel("Global Min: --- mV")
        self.lbl_global_min.setStyleSheet(f"color: {TEXT_COLOR};")
        self.lbl_global_max = QLabel("Global Max: --- mV")
        self.lbl_global_max.setStyleSheet(f"color: {TEXT_COLOR};")
        self.lbl_stack_total = QLabel("Stack Total: --- V")
        self.lbl_stack_total.setStyleSheet(f"color: {TEXT_COLOR};")
        self.lbl_ams_current = QLabel("Current: --- A")
        self.lbl_ams_current.setStyleSheet(f"color: {TEXT_COLOR};")
        
        global_layout.addWidget(self.lbl_global_min, 0, 0)
        global_layout.addWidget(self.lbl_global_max, 0, 1)
        global_layout.addWidget(self.lbl_stack_total, 1, 0)
        global_layout.addWidget(self.lbl_ams_current, 1, 1)
        
        global_stats.setLayout(global_layout)
        layout.addWidget(global_stats)
        
        widget.setLayout(layout)
        return widget
    
    def create_module_card(self, module_id):
        """Create a card widget for an AMS module"""
        card = QGroupBox(f"Module {module_id}")
        card.setStyleSheet(f"""
            QGroupBox {{
                border: 2px solid {ISC_LIGHT_GREEN};
                border-radius: 5px;
                margin-top: 10px;
                background: {ISC_GREEN};
                color: {ISC_TEXT};
            }}
            QGroupBox::title {{
                color: {ISC_LIGHT_GREEN};
                font-weight: bold;
            }}
        """)
        
        layout = QVBoxLayout()
        
        lbl_min_v = QLabel("Min: --- mV")
        lbl_max_v = QLabel("Max: --- mV")
        lbl_min_t = QLabel("Min T: --- °C")
        lbl_max_t = QLabel("Max T: --- °C")
        lbl_age = QLabel("Age: ---")
        
        for lbl in [lbl_min_v, lbl_max_v, lbl_min_t, lbl_max_t, lbl_age]:
            lbl.setStyleSheet(f"font-size: 10px; color: {TEXT_COLOR};")
        
        layout.addWidget(lbl_min_v)
        layout.addWidget(lbl_max_v)
        layout.addWidget(lbl_min_t)
        layout.addWidget(lbl_max_t)
        layout.addWidget(lbl_age)
        
        card.setLayout(layout)
        
        # Store labels as attributes
        card.lbl_min_v = lbl_min_v
        card.lbl_max_v = lbl_max_v
        card.lbl_min_t = lbl_min_t
        card.lbl_max_t = lbl_max_t
        card.lbl_age = lbl_age
        
        return card
    
    def create_metric_label(self, title, value, color, compact=False):
        """Create a styled metric display label"""
        frame = QFrame()
        frame.setStyleSheet(f"""
            QFrame {{
                background: {ISC_GREEN};
                border: 2px solid {color};
                border-radius: 5px;
                padding: {'5px' if compact else '10px'};
            }}
        """)
        
        layout = QVBoxLayout()
        layout.setSpacing(2)
        
        title_lbl = QLabel(title)
        title_lbl.setAlignment(Qt.AlignCenter)
        title_lbl.setStyleSheet(f"color: {color}; font-size: {'10px' if compact else '12px'}; font-weight: bold;")
        
        value_lbl = QLabel(value)
        value_lbl.setAlignment(Qt.AlignCenter)
        value_lbl.setStyleSheet(f"color: {TEXT_COLOR}; font-size: {'16px' if compact else '20px'}; font-weight: bold;")
        
        layout.addWidget(title_lbl)
        layout.addWidget(value_lbl)
        frame.setLayout(layout)
        
        # Store value label as attribute
        frame.value_label = value_lbl
        return frame
    
    def create_mini_log_frame(self):
        """Create mini log display frame"""
        frame = QGroupBox("System Log")
        frame.setStyleSheet(f"""
            QGroupBox {{
                border: 2px solid {ISC_LIGHT_GREEN};
                border-radius: 5px;
                margin-top: 5px;
                background: {ISC_GREEN};
                color: {ISC_TEXT};
            }}
            QGroupBox::title {{
                color: {ISC_LIGHT_GREEN};
                font-weight: bold;
                font-size: 10px;
            }}
        """)
        
        layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(80)
        self.log_text.setStyleSheet(f"background: {PLOT_BG}; color: {TEXT_COLOR}; font-family: monospace; font-size: 9px; border: 1px solid {ISC_LIGHT_GREEN};")
        layout.addWidget(self.log_text)
        
        frame.setLayout(layout)
        return frame
    
    def refresh_ports(self):
        """Refresh available serial ports"""
        self.combo_port.clear()
        ports = rtt.list_serial_ports()
        for port, desc in ports:
            self.combo_port.addItem(f"{port} - {desc}", port)
        if ports:
            self.append_log(f"Found {len(ports)} port(s)")
    
    def start_reception(self):
        """Start data reception thread"""
        if self.is_receiving:
            return
        
        piloto = self.input_pilot.text()
        circuito = self.input_circuit.text()
        port = self.combo_port.currentData()
        use_influx = self.chk_influx.isChecked()
        debug = self.chk_debug.isChecked()
        
        try:
            baud = int(self.input_baud.text())
        except ValueError:
            QMessageBox.warning(self, "Error", "Invalid baudrate")
            return
        
        if not port:
            QMessageBox.warning(self, "Error", "No port selected")
            return
        
        bucket_id = rtt.create_bucket(piloto, circuito, use_influx=use_influx)
        self.append_log(f"Starting: {port} @ {baud}")
        self.append_log(f"Bucket: {bucket_id}")
        
        self.is_receiving = True
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)
        
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
                signaler.log_message.emit(f"ERROR: {e}")
            finally:
                self.is_receiving = False
        
        self.rx_thread = threading.Thread(target=rx_worker, daemon=True)
        self.rx_thread.start()
    
    def stop_reception(self):
        """Stop data reception"""
        if not self.is_receiving:
            return
        
        self.append_log("Stopping reception...")
        rtt.new_data_flag = -1
        self.is_receiving = False
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)
    
    def update_displays(self):
        """Update all displays with latest data"""
        data = rtt.get_latest_data()
        
        # Update status badge (smaller)
        status_info = data.get("__STATUS__", {})
        badge = status_info.get("badge", "STALE")
        
        if badge == "LIVE":
            self.status_label.setText(f"🟢 LIVE")
            self.status_label.setStyleSheet(f"""
                background: {ISC_GREEN};
                color: {ISC_LIGHT_GREEN};
                font-size: 12px;
                font-weight: bold;
                padding: 8px;
                border: 2px solid {ISC_LIGHT_GREEN};
                border-radius: 5px;
            """)
        elif badge == "STALE":
            self.status_label.setText(f"🟡 STALE")
            self.status_label.setStyleSheet(f"""
                background: {ISC_GREEN};
                color: {WARNING_COLOR};
                font-size: 12px;
                font-weight: bold;
                padding: 8px;
                border: 2px solid {WARNING_COLOR};
                border-radius: 5px;
            """)
        else:
            self.status_label.setText(f"🔴 BAD")
            self.status_label.setStyleSheet(f"""
                background: {ISC_GREEN};
                color: {ERROR_COLOR};
                font-size: 12px;
                font-weight: bold;
                padding: 8px;
                border: 2px solid {ERROR_COLOR};
                border-radius: 5px;
            """)
        
        # Update metrics from ID 0x600
        data_600 = data.get("0x600", {})
        rpm = data_600.get('rpm', 0)
        dc_bus = data_600.get('dc_bus_voltage', 0)
        
        self.lbl_dc_bus.value_label.setText(f"{dc_bus:.1f} V")
        self.lbl_rpm.value_label.setText(f"{rpm:.0f}")
        self.lbl_torque.value_label.setText(f"{data_600.get('torque_total', 0):.1f} Nm")
        self.lbl_min_cell.value_label.setText(f"{data_600.get('cell_min_v', 0):.0f} mV")
        
        data_630 = data.get("0x630", {})
        self.lbl_throttle.value_label.setText(f"{data_630.get('throttle', 0):.1f} %")
        
        data_610 = data.get("0x610", {})
        self.lbl_current.value_label.setText(f"{data_610.get('i_actual', 0):.1f} A")
        
        # Update AMS data
        ams_summary = data.get("ams_summary", {})
        stack_mv = ams_summary.get('stack_mv', 0)
        self.lbl_stack.value_label.setText(f"{stack_mv / 1000:.1f} V")
        
        ams_temp = data.get("ams_temp_summary", {})
        max_temp = ams_temp.get('max_temp_c', 0)
        self.lbl_max_temp.value_label.setText(f"{max_temp:.0f} °C")
        
        # Update mini metrics in logo section
        speed_kmh = rpm * 0.05  # Placeholder formula
        self.mini_speed.setText(f"Speed: {speed_kmh:.1f} km/h")
        self.mini_voltage.setText(f"Voltage: {dc_bus:.1f} V")
        self.mini_temp.setText(f"Temp: {max_temp:.0f} °C")
        
        # Update global AMS stats
        self.lbl_global_min.setText(f"Global Min: {rtt.ams_global_min_mv} mV")
        self.lbl_global_max.setText(f"Global Max: {rtt.ams_global_max_mv} mV")
        self.lbl_stack_total.setText(f"Stack Total: {rtt.ams_stack_total_mv / 1000:.1f} V")
        self.lbl_ams_current.setText(f"Current: {rtt.ams_current_dA / 10:.1f} A")
        
        # Update module cards
        now = time.time()
        for i, card in enumerate(self.module_cards):
            mod = rtt.get_ams_module_data(i)
            if mod:
                card.lbl_min_v.setText(f"Min: {mod.min_cell_mv} mV")
                card.lbl_max_v.setText(f"Max: {mod.max_cell_mv} mV")
                card.lbl_min_t.setText(f"Min T: {mod.min_temp_c:.0f} °C")
                card.lbl_max_t.setText(f"Max T: {mod.max_temp_c:.0f} °C")
                age = now - mod.last_update_ts
                card.lbl_age.setText(f"Age: {age:.1f}s")
        
        # Update plots
        self.plot_rpm.update_plot(rpm)
        self.plot_voltage.update_plot(data_600.get('cell_min_v', 0))
        self.plot_temp.update_plot(max_temp)
    
    def append_log(self, msg: str):
        """Append message to log window (keep last 10 lines)"""
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{ts}] {msg}")
        
        # Keep only last 10 lines
        doc = self.log_text.document()
        while doc.blockCount() > 10:
            cursor = self.log_text.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.select(cursor.BlockUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()
    
    def open_motor_window(self):
        """Open motor/inverter data window"""
        if self.motor_window is None or not self.motor_window.isVisible():
            self.motor_window = MotorInverterWindow()
            self.motor_window.show()
    
    def open_accu_window(self):
        """Open accumulator window with heatmaps"""
        if self.accu_window is None or not self.accu_window.isVisible():
            self.accu_window = AccumulatorWindow()
            self.accu_window.show()
    
    def open_driver_window(self):
        """Open driver readings window"""
        if self.driver_window is None or not self.driver_window.isVisible():
            self.driver_window = DriverWindow()
            self.driver_window.show()
    
    def open_log_viewer(self):
        """Open full log viewer window"""
        if self.log_viewer is None or not self.log_viewer.isVisible():
            self.log_viewer = LogViewerWindow()
            self.log_viewer.show()
    
    def open_session_viewer(self):
        """Open Excel session viewer window"""
        if self.session_viewer is None or not self.session_viewer.isVisible():
            self.session_viewer = SessionViewerWindow()
            self.session_viewer.show()
    
    def export_current_session(self):
        """Export current session data"""
        QMessageBox.information(self, "Export", "Current session export feature coming soon!")
    
    def closeEvent(self, event):
        """Handle window close"""
        if self.is_receiving:
            self.stop_reception()
            time.sleep(0.5)
        event.accept()

# Due to length constraints, continuing with remaining classes in next section...

# ============== SESSION VIEWER WINDOW ==============
class SessionViewerWindow(QWidget):
    """Window to view and analyze past Excel sessions"""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Session Viewer & Analyzer")
        self.setGeometry(150, 150, 1400, 800)
        self.current_session_data = {}
        self.init_ui()
    
    def init_ui(self):
        layout = QHBoxLayout()
        
        # Left panel: Session list
        left_panel = QWidget()
        left_layout = QVBoxLayout()
        
        title = QLabel("📊 Sessions")
        title.setStyleSheet(f"font-size: 16px; font-weight: bold; color: {ISC_LIGHT_GREEN}; padding: 10px;")
        left_layout.addWidget(title)
        
        self.session_list = QListWidget()
        self.session_list.setStyleSheet(f"background: {WIDGET_BG}; color: {TEXT_COLOR}; border: 1px solid {ISC_LIGHT_GREEN};")
        self.session_list.itemClicked.connect(self.load_session)
        left_layout.addWidget(self.session_list)
        
        btn_refresh = QPushButton("🔄 Refresh")
        btn_refresh.setStyleSheet(f"background: {ISC_GREEN}; color: {TEXT_COLOR}; border: 1px solid {ISC_LIGHT_GREEN};")
        btn_refresh.clicked.connect(self.refresh_session_list)
        left_layout.addWidget(btn_refresh)
        
        left_panel.setLayout(left_layout)
        left_panel.setMaximumWidth(300)
        
        # Right panel: Session data viewer
        right_panel = QWidget()
        right_layout = QVBoxLayout()
        
        self.session_info_label = QLabel("Select a session")
        self.session_info_label.setStyleSheet(f"font-size: 14px; color: {ISC_LIGHT_GREEN}; padding: 10px;")
        right_layout.addWidget(self.session_info_label)
        
        # Tabs for different data views
        self.data_tabs = QTabWidget()
        self.data_tabs.setStyleSheet(f"""
            QTabWidget::pane {{
                border: 1px solid {ISC_LIGHT_GREEN};
                background: {WIDGET_BG};
            }}
            QTabBar::tab {{
                background: {ISC_GREEN};
                color: {TEXT_COLOR};
                padding: 8px 16px;
            }}
            QTabBar::tab:selected {{
                background: {ISC_LIGHT_GREEN};
                color: {ISC_DARK_BG};
            }}
        """)
        
        right_layout.addWidget(self.data_tabs)
        right_panel.setLayout(right_layout)
        
        # Splitter
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)
        splitter.setStretchFactor(1, 3)
        
        layout.addWidget(splitter)
        self.setLayout(layout)
        
        # Initial refresh
        self.refresh_session_list()
    
    def refresh_session_list(self):
        """Refresh the list of available sessions"""
        self.session_list.clear()
        sessions = rtt.list_excel_sessions()
        
        for session_file in sessions:
            mod_time = datetime.fromtimestamp(session_file.stat().st_mtime)
            display_text = f"{session_file.stem}\n  {mod_time.strftime('%Y-%m-%d %H:%M:%S')}"
            item = QtWidgets.QListWidgetItem(display_text)
            item.setData(Qt.UserRole, session_file)
            self.session_list.addItem(item)
    
    def load_session(self, item):
        """Load and display selected session"""
        session_file = item.data(Qt.UserRole)
        
        try:
            self.session_info_label.setText(f"Loading: {session_file.name}...")
            self.current_session_data = rtt.load_excel_session(session_file)
            
            if not self.current_session_data:
                self.session_info_label.setText(f"❌ Error loading: {session_file.name}")
                return
            
            # Update info label
            if 'Metadata' in self.current_session_data:
                meta = self.current_session_data['Metadata']
                info_text = f"✅ {session_file.name}\n"
                if len(meta) > 0:
                    info_text += f"Piloto: {meta['Piloto'].iloc[0]}, "
                    info_text += f"Circuit: {meta['Circuito'].iloc[0]}, "
                    info_text += f"Duration: {meta['Duration (min)'].iloc[0]:.1f} min"
                self.session_info_label.setText(info_text)
            else:
                self.session_info_label.setText(f"✅ {session_file.name}")
            
            # Clear existing tabs
            self.data_tabs.clear()
            
            # Create tabs for each sheet
            for sheet_name, df in self.current_session_data.items():
                if sheet_name == 'Metadata':
                    continue
                
                tab = self.create_data_tab(sheet_name, df)
                self.data_tabs.addTab(tab, sheet_name)
        
        except Exception as e:
            self.session_info_label.setText(f"❌ Error: {str(e)}")
    
    def create_data_tab(self, sheet_name, df):
        """Create a tab with plots for a specific data sheet"""
        widget = QWidget()
        layout = QVBoxLayout()
        
        info = QLabel(f"{sheet_name}: {len(df)} records")
        info.setStyleSheet(f"color: {TEXT_COLOR}; padding: 5px;")
        layout.addWidget(info)
        
        if 'timestamp' in df.columns:
            df['timestamp'] = pd.to_datetime(df['timestamp'])
            
            if sheet_name == 'Main':
                plot_layout = QGridLayout()
                
                if 'rpm' in df.columns:
                    canvas1 = self.create_timeseries_plot(df, 'rpm', 'RPM')
                    plot_layout.addWidget(canvas1, 0, 0)
                
                if 'dc_bus_voltage' in df.columns:
                    canvas2 = self.create_timeseries_plot(df, 'dc_bus_voltage', 'DC Bus (V)')
                    plot_layout.addWidget(canvas2, 0, 1)
                
                if 'torque_total' in df.columns:
                    canvas3 = self.create_timeseries_plot(df, 'torque_total', 'Torque (Nm)')
                    plot_layout.addWidget(canvas3, 1, 0)
                
                if 'cell_min_v' in df.columns:
                    canvas4 = self.create_timeseries_plot(df, 'cell_min_v', 'Min Cell (mV)')
                    plot_layout.addWidget(canvas4, 1, 1)
                
                layout.addLayout(plot_layout)
            
            elif sheet_name == 'Motor_Inverter':
                plot_layout = QGridLayout()
                
                if 'motor_temp' in df.columns:
                    canvas1 = self.create_timeseries_plot(df, 'motor_temp', 'Motor Temp (°C)')
                    plot_layout.addWidget(canvas1, 0, 0)
                
                if 'i_actual' in df.columns:
                    canvas2 = self.create_timeseries_plot(df, 'i_actual', 'Current (A)')
                    plot_layout.addWidget(canvas2, 0, 1)
                
                layout.addLayout(plot_layout)
            
            elif sheet_name == 'AMS_Summary':
                plot_layout = QGridLayout()
                
                if 'stack_mv' in df.columns:
                    canvas1 = self.create_timeseries_plot(df, 'stack_mv', 'Stack (mV)')
                    plot_layout.addWidget(canvas1, 0, 0)
                
                if 'max_temp_c' in df.columns:
                    canvas2 = self.create_timeseries_plot(df, 'max_temp_c', 'Max Temp (°C)')
                    plot_layout.addWidget(canvas2, 0, 1)
                
                if 'current_A' in df.columns:
                    canvas3 = self.create_timeseries_plot(df, 'current_A', 'Current (A)')
                    plot_layout.addWidget(canvas3, 1, 0)
                
                layout.addLayout(plot_layout)
        
        else:
            text = QTextEdit()
            text.setReadOnly(True)
            text.setText(df.head(50).to_string())
            text.setStyleSheet(f"background: {PLOT_BG}; color: {TEXT_COLOR}; font-family: monospace;")
            layout.addWidget(text)
        
        widget.setLayout(layout)
        return widget
    
    def create_timeseries_plot(self, df, column, title):
        """Create a time series plot"""
        fig = Figure(figsize=(6, 3), facecolor=PLOT_BG)
        ax = fig.add_subplot(111)
        ax.set_facecolor(PLOT_BG)
        ax.set_title(title, color=ISC_LIGHT_GREEN, fontweight='bold')
        ax.tick_params(colors=TEXT_COLOR)
        
        ax.plot(df['timestamp'], df[column], color=ISC_LIGHT_GREEN, linewidth=1.5)
        ax.grid(True, alpha=0.3, color=TEXT_COLOR)
        ax.tick_params(axis='x', rotation=45)
        fig.tight_layout()
        
        canvas = FigureCanvas(fig)
        return canvas

# ============== OTHER WINDOWS (LogViewer, MplCanvas, Motor, Accu, Driver, Heatmap) ==============
# Keeping same implementations but with ISC color scheme...

class LogViewerWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("System Logs")
        self.setGeometry(200, 200, 900, 600)
        self.init_ui()
        signaler.log_message.connect(self.append_log)
    
    def init_ui(self):
        layout = QVBoxLayout()
        
        title = QLabel("📄 System Logs")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(f"font-size: 18px; font-weight: bold; color: {ISC_LIGHT_GREEN}; padding: 10px;")
        layout.addWidget(title)
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setStyleSheet(f"background: {PLOT_BG}; color: {TEXT_COLOR}; font-family: monospace; border: 1px solid {ISC_LIGHT_GREEN};")
        layout.addWidget(self.log_text)
        
        btn_layout = QHBoxLayout()
        
        btn_clear = QPushButton("🗑️ Clear")
        btn_clear.setStyleSheet(f"background: {ISC_GREEN}; color: {TEXT_COLOR}; border: 1px solid {ISC_LIGHT_GREEN};")
        btn_clear.clicked.connect(self.log_text.clear)
        btn_layout.addWidget(btn_clear)
        
        btn_save = QPushButton("💾 Save")
        btn_save.setStyleSheet(f"background: {ISC_GREEN}; color: {TEXT_COLOR}; border: 1px solid {ISC_LIGHT_GREEN};")
        btn_save.clicked.connect(self.save_logs)
        btn_layout.addWidget(btn_save)
        
        layout.addLayout(btn_layout)
        self.setLayout(layout)
    
    def append_log(self, msg: str):
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.log_text.append(f"[{ts}] {msg}")
    
    def save_logs(self):
        filename, _ = QFileDialog.getSaveFileName(
            self, "Save Logs", f"log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt",
            "Text Files (*.txt)"
        )
        if filename:
            try:
                with open(filename, 'w') as f:
                    f.write(self.log_text.toPlainText())
                QMessageBox.information(self, "Success", f"Logs saved")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed: {e}")

class MplCanvas(FigureCanvas):
    def __init__(self, title="Plot", max_points=100, compact=False):
        figsize = (4, 2) if compact else (5, 3)
        self.fig = Figure(figsize=figsize, facecolor=PLOT_BG)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor(PLOT_BG)
        self.ax.set_title(title, color=ISC_LIGHT_GREEN, fontweight='bold', fontsize=10 if compact else 12)
        self.ax.tick_params(colors=TEXT_COLOR, labelsize=8 if compact else 10)
        for spine in self.ax.spines.values():
            spine.set_color(ISC_LIGHT_GREEN)
        
        super().__init__(self.fig)
        
        self.data = []
        self.max_points = max_points
        self.line, = self.ax.plot([], [], color=ISC_LIGHT_GREEN, linewidth=1.5 if compact else 2)
        self.ax.grid(True, alpha=0.3, color=TEXT_COLOR)
        self.fig.tight_layout()
    
    def update_plot(self, value):
        self.data.append(value)
        if len(self.data) > self.max_points:
            self.data.pop(0)
        
        self.line.set_data(range(len(self.data)), self.data)
        self.ax.relim()
        self.ax.autoscale_view()
        self.draw()

# Motor, Accumulator, Driver, and Heatmap windows follow same pattern with ISC colors
# (Implementations similar to previous version but with color scheme updates)

class MotorInverterWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Motor & Inverter")
        self.setGeometry(150, 150, 900, 600)
        self.init_ui()
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(100)
    
    def init_ui(self):
        layout = QVBoxLayout()
        title = QLabel("🔧 Motor & Inverter")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(f"font-size: 18px; font-weight: bold; color: {ISC_LIGHT_GREEN}; padding: 10px;")
        layout.addWidget(title)
        
        grid = QGridLayout()
        self.lbl_rpm = QLabel("RPM: ---")
        self.lbl_rpm.setStyleSheet(f"color: {TEXT_COLOR};")
        self.lbl_torque_req = QLabel("Torque Req: ---")
        self.lbl_torque_req.setStyleSheet(f"color: {TEXT_COLOR};")
        self.lbl_i_actual = QLabel("Current: ---")
        self.lbl_i_actual.setStyleSheet(f"color: {TEXT_COLOR};")
        self.lbl_motor_temp = QLabel("Motor Temp: ---")
        self.lbl_motor_temp.setStyleSheet(f"color: {TEXT_COLOR};")
        
        grid.addWidget(self.lbl_rpm, 0, 0)
        grid.addWidget(self.lbl_torque_req, 0, 1)
        grid.addWidget(self.lbl_i_actual, 1, 0)
        grid.addWidget(self.lbl_motor_temp, 1, 1)
        layout.addLayout(grid)
        
        plot_layout = QHBoxLayout()
        self.plot_torque = MplCanvas(title="Torque")
        self.plot_current = MplCanvas(title="Current")
        plot_layout.addWidget(self.plot_torque)
        plot_layout.addWidget(self.plot_current)
        layout.addLayout(plot_layout)
        self.setLayout(layout)
    
    def update_data(self):
        data = rtt.get_latest_data()
        data_600 = data.get("0x600", {})
        data_610 = data.get("0x610", {})
        data_630 = data.get("0x630", {})
        
        self.lbl_rpm.setText(f"RPM: {data_600.get('rpm', 0):.0f}")
        self.lbl_torque_req.setText(f"Torque: {data_630.get('torque_req', 0):.1f} Nm")
        self.lbl_i_actual.setText(f"Current: {data_610.get('i_actual', 0):.1f} A")
        self.lbl_motor_temp.setText(f"Motor Temp: {data_610.get('motor_temp', 0):.0f} °C")
        
        self.plot_torque.update_plot(data_600.get('torque_total', 0))
        self.plot_current.update_plot(data_610.get('i_actual', 0))

class AccumulatorWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Accumulator")
        self.setGeometry(150, 150, 1400, 800)
        self.init_ui()
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(500)
    
    def init_ui(self):
        layout = QVBoxLayout()
        title = QLabel("🔋 Accumulator")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(f"font-size: 18px; font-weight: bold; color: {ISC_LIGHT_GREEN}; padding: 10px;")
        layout.addWidget(title)
        
        stats_layout = QHBoxLayout()
        self.lbl_stack_v = QLabel("Stack: ---")
        self.lbl_stack_v.setStyleSheet(f"color: {TEXT_COLOR};")
        self.lbl_current = QLabel("Current: ---")
        self.lbl_current.setStyleSheet(f"color: {TEXT_COLOR};")
        stats_layout.addWidget(self.lbl_stack_v)
        stats_layout.addWidget(self.lbl_current)
        layout.addLayout(stats_layout)
        
        heatmap_layout = QGridLayout()
        self.heatmaps = []
        for i in range(NUM_MODULES):
            heatmap = HeatmapCanvas(title=f"Module {i}")
            heatmap_layout.addWidget(heatmap, i // 3, i % 3)
            self.heatmaps.append(heatmap)
        layout.addLayout(heatmap_layout)
        self.setLayout(layout)
    
    def update_data(self):
        self.lbl_stack_v.setText(f"Stack: {rtt.ams_stack_total_mv / 1000:.1f} V")
        self.lbl_current.setText(f"Current: {rtt.ams_current_dA / 10:.1f} A")
        for i, heatmap in enumerate(self.heatmaps):
            mod = rtt.get_ams_module_data(i)
            if mod:
                heatmap.update_heatmap(mod.temps_c)

class HeatmapCanvas(FigureCanvas):
    def __init__(self, title="Heatmap"):
        self.fig = Figure(figsize=(4, 3), facecolor=PLOT_BG)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor(PLOT_BG)
        self.ax.set_title(title, color=ISC_LIGHT_GREEN, fontsize=10, fontweight='bold')
        super().__init__(self.fig)
        self.grid_data = np.zeros((6, 7))
        self.im = self.ax.imshow(self.grid_data, cmap='hot', vmin=20, vmax=60, aspect='auto')
        self.fig.colorbar(self.im, ax=self.ax, label='°C')
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.fig.tight_layout()
    
    def update_heatmap(self, temps):
        temps_array = np.array(temps[:TEMPS_PER_MODULE])
        temps_array = np.nan_to_num(temps_array, nan=0.0)
        padded = np.pad(temps_array, (0, 42 - len(temps_array)), constant_values=0)
        grid = padded.reshape((6, 7))
        self.im.set_data(grid)
        self.im.set_clim(vmin=max(20, grid.min()), vmax=min(60, grid.max()))
        self.draw()

class DriverWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Driver Data")
        self.setGeometry(150, 150, 800, 500)
        self.init_ui()
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(100)
    
    def init_ui(self):
        layout = QVBoxLayout()
        title = QLabel("🏎️ Driver")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(f"font-size: 18px; font-weight: bold; color: {ISC_LIGHT_GREEN}; padding: 10px;")
        layout.addWidget(title)
        
        grid = QGridLayout()
        self.lbl_throttle_pct = QLabel("Throttle: ---")
        self.lbl_throttle_pct.setStyleSheet(f"color: {TEXT_COLOR};")
        self.lbl_brake_pct = QLabel("Brake: ---")
        self.lbl_brake_pct.setStyleSheet(f"color: {TEXT_COLOR};")
        grid.addWidget(self.lbl_throttle_pct, 0, 0)
        grid.addWidget(self.lbl_brake_pct, 0, 1)
        layout.addLayout(grid)
        
        plot_layout = QHBoxLayout()
        self.plot_throttle = MplCanvas(title="Throttle %")
        self.plot_brake = MplCanvas(title="Brake %")
        plot_layout.addWidget(self.plot_throttle)
        plot_layout.addWidget(self.plot_brake)
        layout.addLayout(plot_layout)
        self.setLayout(layout)
    
    def update_data(self):
        data = rtt.get_latest_data()
        data_630 = data.get("0x630", {})
        throttle = data_630.get('throttle', 0)
        brake = data_630.get('brake', 0)
        self.lbl_throttle_pct.setText(f"Throttle: {throttle:.1f} %")
        self.lbl_brake_pct.setText(f"Brake: {brake:.1f} %")
        self.plot_throttle.update_plot(throttle)
        self.plot_brake.update_plot(brake)

def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
