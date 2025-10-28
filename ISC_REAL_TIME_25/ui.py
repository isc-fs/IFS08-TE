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

# ============== MATRIX GREEN COLOR SCHEME ==============
MATRIX_GREEN = '#00FF41'  # Bright Matrix green for text/graphs
ISC_DARK_GREEN = '#002D0C'  # RGB(0, 45, 12) - Dark ISC green for backgrounds
BLACK_BG = '#0a0a0a'  # Pure black background
DARK_GREY = '#1a1a1a'  # Dark grey for widgets
TEXT_COLOR = MATRIX_GREEN  # Matrix green for all text
ACCENT_COLOR = MATRIX_GREEN  # Matrix green for accents
WARNING_COLOR = '#ffff00'  # Bright yellow for warnings
ERROR_COLOR = '#ff0000'  # Bright red for errors

PLOT_BG = BLACK_BG
WIDGET_BG = DARK_GREY

# ============== SIGNAL EMITTER ==============
class Signaler(QObject):
    """Thread-safe signal emitter"""
    new_data = pyqtSignal()
    log_message = pyqtSignal(str)

signaler = Signaler()

# ============== MATPLOTLIB MATRIX STYLE ==============
plt.style.use('dark_background')

# ============== MAIN WINDOW ==============
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ISCmetrics - Formula Student Telemetry")
        self.setGeometry(100, 100, 1800, 950)
        
        # Set window icon
        icon_path = Path("isc_logo.ico")
        if icon_path.exists():
            self.setWindowIcon(QIcon(str(icon_path)))
        
        # Apply Matrix theme
        self.apply_matrix_theme()
        
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
    
    def apply_matrix_theme(self):
        """Apply Matrix green on black theme"""
        palette = QPalette()
        palette.setColor(QPalette.Window, QColor(BLACK_BG))
        palette.setColor(QPalette.WindowText, QColor(MATRIX_GREEN))
        palette.setColor(QPalette.Base, QColor(DARK_GREY))
        palette.setColor(QPalette.AlternateBase, QColor(20, 20, 20))
        palette.setColor(QPalette.ToolTipBase, QColor(MATRIX_GREEN))
        palette.setColor(QPalette.ToolTipText, QColor(BLACK_BG))
        palette.setColor(QPalette.Text, QColor(MATRIX_GREEN))
        palette.setColor(QPalette.Button, QColor(0, 45, 12))
        palette.setColor(QPalette.ButtonText, QColor(MATRIX_GREEN))
        palette.setColor(QPalette.BrightText, QColor(MATRIX_GREEN))
        palette.setColor(QPalette.Highlight, QColor(MATRIX_GREEN))
        palette.setColor(QPalette.HighlightedText, QColor(BLACK_BG))
        self.setPalette(palette)
    
    def init_ui(self):
        """Initialize main UI layout"""
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setSpacing(3)
        main_layout.setContentsMargins(5, 5, 5, 5)
        
        # === COMPACT TOP SECTION: Logo, ISCmetrics, Status, and Controls in ONE ROW ===
        top_section = self.create_compact_top_section()
        main_layout.addWidget(top_section)
        
        # === MIDDLE: Tab widget (MORE SPACE) ===
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet(f"""
            QTabWidget::pane {{
                border: 1px solid {MATRIX_GREEN};
                background: {BLACK_BG};
            }}
            QTabBar::tab {{
                background: {ISC_DARK_GREEN};
                color: {MATRIX_GREEN};
                padding: 5px 12px;
                margin-right: 1px;
                border: 1px solid {MATRIX_GREEN};
            }}
            QTabBar::tab:selected {{
                background: {MATRIX_GREEN};
                color: {BLACK_BG};
                font-weight: bold;
            }}
        """)
        
        # Tab 1: Overview Dashboard
        self.overview_tab = self.create_overview_tab()
        self.tabs.addTab(self.overview_tab, "📊 Overview")
        
        # Tab 2: AMS Module Details
        self.ams_tab = self.create_ams_tab()
        self.tabs.addTab(self.ams_tab, "🔋 AMS Modules")
        
        main_layout.addWidget(self.tabs, stretch=8)
        
        # === BOTTOM: Mini Log Window ===
        log_frame = self.create_mini_log_frame()
        main_layout.addWidget(log_frame, stretch=1)
        
        # === ATTRIBUTION ===
        attribution = QLabel("Andrés Sánchez de Ágreda © 2025/2026 - ICAI Racing Formula Student")
        attribution.setAlignment(Qt.AlignCenter)
        attribution.setStyleSheet(f"color: {MATRIX_GREEN}; font-size: 8px; padding: 2px;")
        main_layout.addWidget(attribution)
    
    def create_compact_top_section(self):
        """Create super compact top section with everything in one row"""
        container = QFrame()
        container.setStyleSheet(f"""
            QFrame {{
                background: {ISC_DARK_GREEN};
                border: 1px solid {MATRIX_GREEN};
                border-radius: 3px;
            }}
        """)
        container.setMaximumHeight(100)
        
        main_layout = QHBoxLayout(container)
        main_layout.setSpacing(8)
        main_layout.setContentsMargins(5, 5, 5, 5)
        
        # 1. Logo + ISCmetrics (very compact)
        logo_section = self.create_logo_section_compact()
        main_layout.addWidget(logo_section)
        
        # 2. Status indicator (minimal)
        self.status_label = QLabel("⚪ IDLE")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet(f"""
            background: {BLACK_BG};
            color: {MATRIX_GREEN};
            font-size: 10px;
            font-weight: bold;
            padding: 4px 12px;
            border: 1px solid {MATRIX_GREEN};
            border-radius: 3px;
        """)
        self.status_label.setMaximumWidth(80)
        main_layout.addWidget(self.status_label)
        
        # 3. Config controls (super compact grid)
        config_section = self.create_config_section_compact()
        main_layout.addWidget(config_section, stretch=2)
        
        # 4. Action buttons (vertical compact)
        buttons_section = self.create_buttons_section_compact()
        main_layout.addWidget(buttons_section)
        
        return container
    
    def create_logo_section_compact(self):
        """Minimal logo section"""
        frame = QFrame()
        frame.setMaximumWidth(90)
        frame.setStyleSheet(f"""
            QFrame {{
                background: {BLACK_BG};
                border: 1px solid {MATRIX_GREEN};
                border-radius: 3px;
            }}
        """)
        
        layout = QVBoxLayout()
        layout.setSpacing(1)
        layout.setContentsMargins(3, 3, 3, 3)
        
        # Logo
        logo_label = QLabel()
        logo_path = Path("isc_logo.ico")
        
        if logo_path.exists():
            pixmap = QPixmap(str(logo_path))
            scaled_pixmap = pixmap.scaled(40, 40, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            logo_label.setPixmap(scaled_pixmap)
        else:
            logo_label.setText("ISC")
            logo_label.setStyleSheet(f"color: {MATRIX_GREEN}; font-size: 14px; font-weight: bold;")
        
        logo_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(logo_label)
        
        # ISCmetrics text
        app_name = QLabel("ISCmetrics")
        app_name.setAlignment(Qt.AlignCenter)
        app_name.setStyleSheet(f"""
            color: {MATRIX_GREEN};
            font-size: 9px;
            font-weight: bold;
        """)
        layout.addWidget(app_name)
        
        # Mini metrics
        self.mini_speed = QLabel("--- km/h")
        self.mini_voltage = QLabel("--- V")
        self.mini_temp = QLabel("--- °C")
        
        for lbl in [self.mini_speed, self.mini_voltage, self.mini_temp]:
            lbl.setStyleSheet(f"color: {MATRIX_GREEN}; font-size: 7px;")
            lbl.setAlignment(Qt.AlignCenter)
            layout.addWidget(lbl)
        
        frame.setLayout(layout)
        return frame
    
    def create_config_section_compact(self):
        """Compact config section"""
        frame = QFrame()
        layout = QGridLayout()
        layout.setSpacing(3)
        layout.setContentsMargins(3, 3, 3, 3)
        
        # Style
        label_style = f"color: {MATRIX_GREEN}; font-size: 8px;"
        input_style = f"background: {BLACK_BG}; color: {MATRIX_GREEN}; border: 1px solid {MATRIX_GREEN}; font-size: 8px; padding: 2px;"
        
        # Row 1
        lbl_pilot = QLabel("Piloto:")
        lbl_pilot.setStyleSheet(label_style)
        layout.addWidget(lbl_pilot, 0, 0)
        
        self.input_pilot = QLineEdit("Piloto_Test")
        self.input_pilot.setMaximumWidth(90)
        self.input_pilot.setStyleSheet(input_style)
        layout.addWidget(self.input_pilot, 0, 1)
        
        lbl_circuit = QLabel("Circuito:")
        lbl_circuit.setStyleSheet(label_style)
        layout.addWidget(lbl_circuit, 0, 2)
        
        self.input_circuit = QLineEdit("Circuito_Test")
        self.input_circuit.setMaximumWidth(90)
        self.input_circuit.setStyleSheet(input_style)
        layout.addWidget(self.input_circuit, 0, 3)
        
        lbl_port = QLabel("Puerto:")
        lbl_port.setStyleSheet(label_style)
        layout.addWidget(lbl_port, 0, 4)
        
        self.combo_port = QComboBox()
        self.combo_port.setMaximumWidth(140)
        self.combo_port.setStyleSheet(input_style)
        self.refresh_ports()
        layout.addWidget(self.combo_port, 0, 5)
        
        # Row 2
        lbl_baud = QLabel("Baud:")
        lbl_baud.setStyleSheet(label_style)
        layout.addWidget(lbl_baud, 1, 0)
        
        self.input_baud = QLineEdit("115200")
        self.input_baud.setMaximumWidth(60)
        self.input_baud.setStyleSheet(input_style)
        layout.addWidget(self.input_baud, 1, 1)
        
        self.chk_influx = QCheckBox("InfluxDB")
        self.chk_influx.setStyleSheet(f"color: {MATRIX_GREEN}; font-size: 8px;")
        self.chk_influx.setChecked(False)
        layout.addWidget(self.chk_influx, 1, 2)
        
        self.chk_debug = QCheckBox("Debug")
        self.chk_debug.setStyleSheet(f"color: {MATRIX_GREEN}; font-size: 8px;")
        self.chk_debug.setChecked(False)
        layout.addWidget(self.chk_debug, 1, 3)
        
        frame.setLayout(layout)
        return frame
    
    def create_buttons_section_compact(self):
        """Compact buttons section"""
        frame = QFrame()
        layout = QGridLayout()
        layout.setSpacing(2)
        layout.setContentsMargins(2, 2, 2, 2)
        
        btn_style = f"""
            QPushButton {{
                background: {ISC_DARK_GREEN};
                color: {MATRIX_GREEN};
                border: 1px solid {MATRIX_GREEN};
                border-radius: 2px;
                padding: 3px 6px;
                font-size: 8px;
                min-width: 55px;
            }}
            QPushButton:hover {{
                background: {MATRIX_GREEN};
                color: {BLACK_BG};
            }}
        """
        
        btn_accent_style = f"""
            QPushButton {{
                background: {MATRIX_GREEN};
                color: {BLACK_BG};
                border: none;
                border-radius: 2px;
                padding: 3px 6px;
                font-size: 8px;
                font-weight: bold;
                min-width: 55px;
            }}
            QPushButton:hover {{
                background: #00dd38;
            }}
        """
        
        self.btn_refresh = QPushButton("🔄")
        self.btn_refresh.setMaximumHeight(22)
        self.btn_refresh.setStyleSheet(btn_style)
        self.btn_refresh.clicked.connect(self.refresh_ports)
        layout.addWidget(self.btn_refresh, 0, 0)
        
        self.btn_start = QPushButton("▶ Start")
        self.btn_start.setMaximumHeight(22)
        self.btn_start.setStyleSheet(btn_accent_style)
        self.btn_start.clicked.connect(self.start_reception)
        layout.addWidget(self.btn_start, 0, 1)
        
        self.btn_stop = QPushButton("⏹ Stop")
        self.btn_stop.setMaximumHeight(22)
        self.btn_stop.setStyleSheet(btn_style)
        self.btn_stop.setEnabled(False)
        self.btn_stop.clicked.connect(self.stop_reception)
        layout.addWidget(self.btn_stop, 0, 2)
        
        btn_motor = QPushButton("🔧 Motor")
        btn_motor.setMaximumHeight(22)
        btn_motor.setStyleSheet(btn_style)
        btn_motor.clicked.connect(self.open_motor_window)
        layout.addWidget(btn_motor, 1, 0)
        
        btn_accu = QPushButton("🔋 Accu")
        btn_accu.setMaximumHeight(22)
        btn_accu.setStyleSheet(btn_style)
        btn_accu.clicked.connect(self.open_accu_window)
        layout.addWidget(btn_accu, 1, 1)
        
        btn_driver = QPushButton("🏎️ Driver")
        btn_driver.setMaximumHeight(22)
        btn_driver.setStyleSheet(btn_style)
        btn_driver.clicked.connect(self.open_driver_window)
        layout.addWidget(btn_driver, 1, 2)
        
        btn_logs = QPushButton("📄 Logs")
        btn_logs.setMaximumHeight(22)
        btn_logs.setStyleSheet(btn_style)
        btn_logs.clicked.connect(self.open_log_viewer)
        layout.addWidget(btn_logs, 2, 0)
        
        btn_sessions = QPushButton("📊 Sessions")
        btn_sessions.setMaximumHeight(22)
        btn_sessions.setStyleSheet(btn_accent_style)
        btn_sessions.clicked.connect(self.open_session_viewer)
        layout.addWidget(btn_sessions, 2, 1)
        
        btn_export = QPushButton("💾 Export")
        btn_export.setMaximumHeight(22)
        btn_export.setStyleSheet(btn_style)
        btn_export.clicked.connect(self.export_current_session)
        layout.addWidget(btn_export, 2, 2)
        
        frame.setLayout(layout)
        return frame
    
    def create_overview_tab(self):
        """Create overview dashboard"""
        widget = QWidget()
        layout = QVBoxLayout()
        
        # Metrics grid
        metrics_grid = QGridLayout()
        metrics_grid.setSpacing(4)
        
        # Row 1
        self.lbl_dc_bus = self.create_metric_label("DC Bus", "--- V", MATRIX_GREEN, compact=True)
        metrics_grid.addWidget(self.lbl_dc_bus, 0, 0)
        
        self.lbl_rpm = self.create_metric_label("RPM", "---", MATRIX_GREEN, compact=True)
        metrics_grid.addWidget(self.lbl_rpm, 0, 1)
        
        self.lbl_torque = self.create_metric_label("Torque", "--- Nm", MATRIX_GREEN, compact=True)
        metrics_grid.addWidget(self.lbl_torque, 0, 2)
        
        self.lbl_current = self.create_metric_label("Current", "--- A", MATRIX_GREEN, compact=True)
        metrics_grid.addWidget(self.lbl_current, 0, 3)
        
        # Row 2
        self.lbl_min_cell = self.create_metric_label("Min Cell", "--- mV", WARNING_COLOR, compact=True)
        metrics_grid.addWidget(self.lbl_min_cell, 1, 0)
        
        self.lbl_stack = self.create_metric_label("Stack", "--- V", MATRIX_GREEN, compact=True)
        metrics_grid.addWidget(self.lbl_stack, 1, 1)
        
        self.lbl_max_temp = self.create_metric_label("Max Temp", "--- °C", WARNING_COLOR, compact=True)
        metrics_grid.addWidget(self.lbl_max_temp, 1, 2)
        
        self.lbl_throttle = self.create_metric_label("Throttle", "--- %", MATRIX_GREEN, compact=True)
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
        summary_label.setStyleSheet(f"font-size: 12px; font-weight: bold; color: {MATRIX_GREEN};")
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
                border: 1px solid {MATRIX_GREEN};
                border-radius: 3px;
                background: {ISC_DARK_GREEN};
                color: {MATRIX_GREEN};
                font-size: 10px;
            }}
            QGroupBox::title {{
                color: {MATRIX_GREEN};
            }}
        """)
        global_layout = QGridLayout()
        
        self.lbl_global_min = QLabel("Global Min: --- mV")
        self.lbl_global_min.setStyleSheet(f"color: {MATRIX_GREEN}; font-size: 10px;")
        self.lbl_global_max = QLabel("Global Max: --- mV")
        self.lbl_global_max.setStyleSheet(f"color: {MATRIX_GREEN}; font-size: 10px;")
        self.lbl_stack_total = QLabel("Stack Total: --- V")
        self.lbl_stack_total.setStyleSheet(f"color: {MATRIX_GREEN}; font-size: 10px;")
        self.lbl_ams_current = QLabel("Current: --- A")
        self.lbl_ams_current.setStyleSheet(f"color: {MATRIX_GREEN}; font-size: 10px;")
        
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
                border: 1px solid {MATRIX_GREEN};
                border-radius: 3px;
                margin-top: 8px;
                background: {ISC_DARK_GREEN};
                color: {MATRIX_GREEN};
                font-size: 9px;
            }}
            QGroupBox::title {{
                color: {MATRIX_GREEN};
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
            lbl.setStyleSheet(f"font-size: 9px; color: {MATRIX_GREEN};")
        
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
                background: {ISC_DARK_GREEN};
                border: 1px solid {color};
                border-radius: 3px;
                padding: {'4px' if compact else '8px'};
            }}
        """)
        
        layout = QVBoxLayout()
        layout.setSpacing(1)
        
        title_lbl = QLabel(title)
        title_lbl.setAlignment(Qt.AlignCenter)
        title_lbl.setStyleSheet(f"color: {color}; font-size: {'9px' if compact else '11px'}; font-weight: bold;")
        
        value_lbl = QLabel(value)
        value_lbl.setAlignment(Qt.AlignCenter)
        value_lbl.setStyleSheet(f"color: {MATRIX_GREEN}; font-size: {'14px' if compact else '18px'}; font-weight: bold;")
        
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
                border: 1px solid {MATRIX_GREEN};
                border-radius: 3px;
                margin-top: 3px;
                background: {ISC_DARK_GREEN};
                color: {MATRIX_GREEN};
                font-size: 9px;
            }}
            QGroupBox::title {{
                color: {MATRIX_GREEN};
                font-weight: bold;
                font-size: 9px;
            }}
        """)
        
        layout = QVBoxLayout()
        layout.setContentsMargins(2, 2, 2, 2)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(70)
        self.log_text.setStyleSheet(f"background: {BLACK_BG}; color: {MATRIX_GREEN}; font-family: 'Courier New'; font-size: 8px; border: 1px solid {MATRIX_GREEN};")
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
        
        # Update status badge
        status_info = data.get("__STATUS__", {})
        badge = status_info.get("badge", "STALE")
        
        if badge == "LIVE":
            self.status_label.setText(f"🟢 LIVE")
            self.status_label.setStyleSheet(f"""
                background: {BLACK_BG};
                color: {MATRIX_GREEN};
                font-size: 10px;
                font-weight: bold;
                padding: 4px 12px;
                border: 1px solid {MATRIX_GREEN};
                border-radius: 3px;
            """)
        elif badge == "STALE":
            self.status_label.setText(f"🟡 STALE")
            self.status_label.setStyleSheet(f"""
                background: {BLACK_BG};
                color: {WARNING_COLOR};
                font-size: 10px;
                font-weight: bold;
                padding: 4px 12px;
                border: 1px solid {WARNING_COLOR};
                border-radius: 3px;
            """)
        else:
            self.status_label.setText(f"🔴 BAD")
            self.status_label.setStyleSheet(f"""
                background: {BLACK_BG};
                color: {ERROR_COLOR};
                font-size: 10px;
                font-weight: bold;
                padding: 4px 12px;
                border: 1px solid {ERROR_COLOR};
                border-radius: 3px;
            """)
        
        # Update metrics
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
        
        # Update mini metrics
        speed_kmh = rpm * 0.05
        self.mini_speed.setText(f"{speed_kmh:.1f} km/h")
        self.mini_voltage.setText(f"{dc_bus:.1f} V")
        self.mini_temp.setText(f"{max_temp:.0f} °C")
        
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
        """Append message to log window"""
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{ts}] {msg}")
        
        # Keep only last 8 lines
        doc = self.log_text.document()
        while doc.blockCount() > 8:
            cursor = self.log_text.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.select(cursor.BlockUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()
    
    def open_motor_window(self):
        if self.motor_window is None or not self.motor_window.isVisible():
            self.motor_window = MotorInverterWindow()
            self.motor_window.show()
    
    def open_accu_window(self):
        if self.accu_window is None or not self.accu_window.isVisible():
            self.accu_window = AccumulatorWindow()
            self.accu_window.show()
    
    def open_driver_window(self):
        if self.driver_window is None or not self.driver_window.isVisible():
            self.driver_window = DriverWindow()
            self.driver_window.show()
    
    def open_log_viewer(self):
        if self.log_viewer is None or not self.log_viewer.isVisible():
            self.log_viewer = LogViewerWindow()
            self.log_viewer.show()
    
    def open_session_viewer(self):
        if self.session_viewer is None or not self.session_viewer.isVisible():
            self.session_viewer = SessionViewerWindow()
            self.session_viewer.show()
    
    def export_current_session(self):
        QMessageBox.information(self, "Export", "Todavía no está, pero se vienen cositas 😉")
    
    def closeEvent(self, event):
        if self.is_receiving:
            self.stop_reception()
            time.sleep(0.5)
        event.accept()

# ============== SESSION VIEWER WINDOW ==============
class SessionViewerWindow(QWidget):
    """Window to view and analyze past Excel sessions"""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Session Viewer")
        self.setGeometry(150, 150, 1400, 800)
        self.current_session_data = {}
        
        # Set window icon
        icon_path = Path("isc_logo.png")
        if icon_path.exists():
            self.setWindowIcon(QIcon(str(icon_path)))
        
        self.init_ui()
    
    def init_ui(self):
        layout = QHBoxLayout()
        
        # Left panel
        left_panel = QWidget()
        left_layout = QVBoxLayout()
        
        title = QLabel("📊 Sessions")
        title.setStyleSheet(f"font-size: 14px; font-weight: bold; color: {MATRIX_GREEN}; padding: 8px;")
        left_layout.addWidget(title)
        
        self.session_list = QListWidget()
        self.session_list.setStyleSheet(f"background: {BLACK_BG}; color: {MATRIX_GREEN}; border: 1px solid {MATRIX_GREEN}; font-size: 9px;")
        self.session_list.itemClicked.connect(self.load_session)
        left_layout.addWidget(self.session_list)
        
        btn_refresh = QPushButton("🔄 Refresh")
        btn_refresh.setStyleSheet(f"background: {ISC_DARK_GREEN}; color: {MATRIX_GREEN}; border: 1px solid {MATRIX_GREEN}; padding: 4px;")
        btn_refresh.clicked.connect(self.refresh_session_list)
        left_layout.addWidget(btn_refresh)
        
        left_panel.setLayout(left_layout)
        left_panel.setMaximumWidth(280)
        
        # Right panel
        right_panel = QWidget()
        right_layout = QVBoxLayout()
        
        self.session_info_label = QLabel("Select a session")
        self.session_info_label.setStyleSheet(f"font-size: 12px; color: {MATRIX_GREEN}; padding: 8px;")
        right_layout.addWidget(self.session_info_label)
        
        self.data_tabs = QTabWidget()
        self.data_tabs.setStyleSheet(f"""
            QTabWidget::pane {{
                border: 1px solid {MATRIX_GREEN};
                background: {BLACK_BG};
            }}
            QTabBar::tab {{
                background: {ISC_DARK_GREEN};
                color: {MATRIX_GREEN};
                padding: 6px 12px;
            }}
            QTabBar::tab:selected {{
                background: {MATRIX_GREEN};
                color: {BLACK_BG};
            }}
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
                self.session_info_label.setText(f"❌ Error loading")
                return
            
            if 'Metadata' in self.current_session_data:
                meta = self.current_session_data['Metadata']
                if len(meta) > 0:
                    info_text = f"✅ {meta['Piloto'].iloc[0]} @ {meta['Circuito'].iloc[0]} ({meta['Duration (min)'].iloc[0]:.1f} min)"
                    self.session_info_label.setText(info_text)
            
            self.data_tabs.clear()
            
            for sheet_name, df in self.current_session_data.items():
                if sheet_name == 'Metadata':
                    continue
                tab = self.create_data_tab(sheet_name, df)
                self.data_tabs.addTab(tab, sheet_name)
        
        except Exception as e:
            self.session_info_label.setText(f"❌ Error: {str(e)}")
    
    def create_data_tab(self, sheet_name, df):
        widget = QWidget()
        layout = QVBoxLayout()
        
        info = QLabel(f"{sheet_name}: {len(df)} records")
        info.setStyleSheet(f"color: {MATRIX_GREEN}; font-size: 9px; padding: 3px;")
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
                
                layout.addLayout(plot_layout)
        
        else:
            text = QTextEdit()
            text.setReadOnly(True)
            text.setText(df.head(50).to_string())
            text.setStyleSheet(f"background: {BLACK_BG}; color: {MATRIX_GREEN}; font-family: monospace; font-size: 8px;")
            layout.addWidget(text)
        
        widget.setLayout(layout)
        return widget
    
    def create_timeseries_plot(self, df, column, title):
        fig = Figure(figsize=(6, 3), facecolor=PLOT_BG)
        ax = fig.add_subplot(111)
        ax.set_facecolor(PLOT_BG)
        ax.set_title(title, color=MATRIX_GREEN, fontweight='bold', fontsize=10)
        ax.tick_params(colors=MATRIX_GREEN, labelsize=8)
        
        ax.plot(df['timestamp'], df[column], color=MATRIX_GREEN, linewidth=1.5)
        ax.grid(True, alpha=0.3, color=MATRIX_GREEN)
        ax.tick_params(axis='x', rotation=45)
        for spine in ax.spines.values():
            spine.set_color(MATRIX_GREEN)
        fig.tight_layout()
        
        canvas = FigureCanvas(fig)
        return canvas

# ============== LOG VIEWER WINDOW ==============
class LogViewerWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("System Logs")
        self.setGeometry(200, 200, 900, 600)
        
        # Set window icon
        icon_path = Path("isc_logo.ico")
        if icon_path.exists():
            self.setWindowIcon(QIcon(str(icon_path)))
        
        self.init_ui()
        signaler.log_message.connect(self.append_log)
    
    def init_ui(self):
        layout = QVBoxLayout()
        
        title = QLabel("📄 System Logs")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(f"font-size: 16px; font-weight: bold; color: {MATRIX_GREEN}; padding: 8px;")
        layout.addWidget(title)
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setStyleSheet(f"background: {BLACK_BG}; color: {MATRIX_GREEN}; font-family: 'Courier New'; font-size: 9px; border: 1px solid {MATRIX_GREEN};")
        layout.addWidget(self.log_text)
        
        btn_layout = QHBoxLayout()
        
        btn_clear = QPushButton("🗑️ Clear")
        btn_clear.setStyleSheet(f"background: {ISC_DARK_GREEN}; color: {MATRIX_GREEN}; border: 1px solid {MATRIX_GREEN}; padding: 4px;")
        btn_clear.clicked.connect(self.log_text.clear)
        btn_layout.addWidget(btn_clear)
        
        btn_save = QPushButton("💾 Save")
        btn_save.setStyleSheet(f"background: {ISC_DARK_GREEN}; color: {MATRIX_GREEN}; border: 1px solid {MATRIX_GREEN}; padding: 4px;")
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
                QMessageBox.information(self, "Success", "Logs saved")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed: {e}")

# ============== MATPLOTLIB CANVAS ==============
class MplCanvas(FigureCanvas):
    def __init__(self, title="Plot", max_points=100, compact=False):
        figsize = (4, 2.2) if compact else (5, 3)
        self.fig = Figure(figsize=figsize, facecolor=PLOT_BG)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor(PLOT_BG)
        self.ax.set_title(title, color=MATRIX_GREEN, fontweight='bold', fontsize=9 if compact else 11)
        self.ax.tick_params(colors=MATRIX_GREEN, labelsize=7 if compact else 9)
        for spine in self.ax.spines.values():
            spine.set_color(MATRIX_GREEN)
        
        super().__init__(self.fig)
        
        self.data = []
        self.max_points = max_points
        self.line, = self.ax.plot([], [], color=MATRIX_GREEN, linewidth=1.5)
        self.ax.grid(True, alpha=0.2, color=MATRIX_GREEN)
        self.fig.tight_layout()
    
    def update_plot(self, value):
        self.data.append(value)
        if len(self.data) > self.max_points:
            self.data.pop(0)
        
        self.line.set_data(range(len(self.data)), self.data)
        self.ax.relim()
        self.ax.autoscale_view()
        self.draw()

# ============== MOTOR WINDOW ==============
class MotorInverterWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Motor & Inverter")
        self.setGeometry(150, 150, 900, 600)
        
        # Set window icon
        icon_path = Path("isc_logo.ico")
        if icon_path.exists():
            self.setWindowIcon(QIcon(str(icon_path)))
        
        self.init_ui()
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(100)
    
    def init_ui(self):
        layout = QVBoxLayout()
        title = QLabel("🔧 Motor & Inverter")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(f"font-size: 16px; font-weight: bold; color: {MATRIX_GREEN}; padding: 8px;")
        layout.addWidget(title)
        
        grid = QGridLayout()
        self.lbl_rpm = QLabel("RPM: ---")
        self.lbl_rpm.setStyleSheet(f"color: {MATRIX_GREEN}; font-size: 11px;")
        self.lbl_torque_req = QLabel("Torque Req: ---")
        self.lbl_torque_req.setStyleSheet(f"color: {MATRIX_GREEN}; font-size: 11px;")
        self.lbl_i_actual = QLabel("Current: ---")
        self.lbl_i_actual.setStyleSheet(f"color: {MATRIX_GREEN}; font-size: 11px;")
        self.lbl_motor_temp = QLabel("Motor Temp: ---")
        self.lbl_motor_temp.setStyleSheet(f"color: {MATRIX_GREEN}; font-size: 11px;")
        
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

# ============== ACCUMULATOR WINDOW ==============
class AccumulatorWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Accumulator")
        self.setGeometry(150, 150, 1400, 800)
        
        # Set window icon
        icon_path = Path("isc_logo.ico")
        if icon_path.exists():
            self.setWindowIcon(QIcon(str(icon_path)))
        
        self.init_ui()
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(500)
    
    def init_ui(self):
        layout = QVBoxLayout()
        title = QLabel("🔋 Accumulator")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(f"font-size: 16px; font-weight: bold; color: {MATRIX_GREEN}; padding: 8px;")
        layout.addWidget(title)
        
        stats_layout = QHBoxLayout()
        self.lbl_stack_v = QLabel("Stack: ---")
        self.lbl_stack_v.setStyleSheet(f"color: {MATRIX_GREEN}; font-size: 11px;")
        self.lbl_current = QLabel("Current: ---")
        self.lbl_current.setStyleSheet(f"color: {MATRIX_GREEN}; font-size: 11px;")
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

# ============== HEATMAP CANVAS ==============
class HeatmapCanvas(FigureCanvas):
    def __init__(self, title="Heatmap"):
        self.fig = Figure(figsize=(4, 3), facecolor=PLOT_BG)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor(PLOT_BG)
        self.ax.set_title(title, color=MATRIX_GREEN, fontsize=10, fontweight='bold')
        super().__init__(self.fig)
        self.grid_data = np.zeros((6, 7))
        self.im = self.ax.imshow(self.grid_data, cmap='hot', vmin=20, vmax=60, aspect='auto')
        cbar = self.fig.colorbar(self.im, ax=self.ax)
        cbar.set_label('°C', color=MATRIX_GREEN)
        cbar.ax.yaxis.set_tick_params(color=MATRIX_GREEN)
        plt.setp(plt.getp(cbar.ax.axes, 'yticklabels'), color=MATRIX_GREEN)
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

# ============== DRIVER WINDOW ==============
class DriverWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Driver Data")
        self.setGeometry(150, 150, 800, 500)
        
        # Set window icon
        icon_path = Path("isc_logo.ico")
        if icon_path.exists():
            self.setWindowIcon(QIcon(str(icon_path)))
        
        self.init_ui()
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(100)
    
    def init_ui(self):
        layout = QVBoxLayout()
        title = QLabel("🏎️ Driver")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(f"font-size: 16px; font-weight: bold; color: {MATRIX_GREEN}; padding: 8px;")
        layout.addWidget(title)
        
        grid = QGridLayout()
        self.lbl_throttle_pct = QLabel("Throttle: ---")
        self.lbl_throttle_pct.setStyleSheet(f"color: {MATRIX_GREEN}; font-size: 11px;")
        self.lbl_brake_pct = QLabel("Brake: ---")
        self.lbl_brake_pct.setStyleSheet(f"color: {MATRIX_GREEN}; font-size: 11px;")
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

# ============== MAIN ==============
def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
