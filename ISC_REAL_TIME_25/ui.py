"""
ISC RTT Telemetry UI - Enhanced Multi-Window Application
Features:
- Main dashboard with 5-module AMS support
- Dedicated Motor/Inverter window
- Accumulator window with heatmaps for all 5 modules
- Driver readings window
- Modern UI with maintained aesthetic
- Attribution footer
"""

from __future__ import annotations
import sys
import os
import threading
import time
from datetime import datetime
from typing import Optional

import numpy as np
import matplotlib
matplotlib.use("Qt5Agg")

from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QObject
from PyQt5.QtGui import QFont, QPalette, QColor
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QGridLayout,
    QWidget, QLabel, QPushButton, QLineEdit, QComboBox, QTextEdit,
    QMessageBox, QTabWidget, QFrame, QGroupBox, QSizePolicy
)

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

import ISC_RTT_serial as rtt

# ============== CONSTANTS ==============
NUM_MODULES = 5
TEMPS_PER_MODULE = 38
CELLS_PER_MODULE = 19

# ============== SIGNAL EMITTER ==============
class Signaler(QObject):
    """Thread-safe signal emitter"""
    new_data = pyqtSignal()
    log_message = pyqtSignal(str)

signaler = Signaler()

# ============== MATPLOTLIB DARK STYLE ==============
plt.style.use('dark_background')
PLOT_BG = '#1e1e1e'
WIDGET_BG = '#2b2b2b'
TEXT_COLOR = '#e0e0e0'
ACCENT_COLOR = '#00d4aa'
WARNING_COLOR = '#ff6b35'
ERROR_COLOR = '#e63946'

# ============== MAIN WINDOW ==============
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ISC RTT Telemetry - Enhanced Dashboard")
        self.setGeometry(100, 100, 1600, 900)
        
        # Apply dark theme
        self.apply_dark_theme()
        
        # Data receiving thread
        self.rx_thread: Optional[threading.Thread] = None
        self.is_receiving = False
        
        # Sub-windows
        self.motor_window: Optional[MotorInverterWindow] = None
        self.accu_window: Optional[AccumulatorWindow] = None
        self.driver_window: Optional[DriverWindow] = None
        
        # Build UI
        self.init_ui()
        
        # Update timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_displays)
        self.timer.start(100)  # 10 Hz refresh
        
        # Connect signals
        signaler.log_message.connect(self.append_log)
    
    def apply_dark_theme(self):
        """Apply modern dark theme"""
        palette = QPalette()
        palette.setColor(QPalette.Window, QColor(30, 30, 30))
        palette.setColor(QPalette.WindowText, QColor(224, 224, 224))
        palette.setColor(QPalette.Base, QColor(43, 43, 43))
        palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
        palette.setColor(QPalette.ToolTipBase, QColor(224, 224, 224))
        palette.setColor(QPalette.ToolTipText, QColor(224, 224, 224))
        palette.setColor(QPalette.Text, QColor(224, 224, 224))
        palette.setColor(QPalette.Button, QColor(53, 53, 53))
        palette.setColor(QPalette.ButtonText, QColor(224, 224, 224))
        palette.setColor(QPalette.BrightText, QColor(255, 0, 0))
        palette.setColor(QPalette.Highlight, QColor(0, 212, 170))
        palette.setColor(QPalette.HighlightedText, QColor(0, 0, 0))
        self.setPalette(palette)
    
    def init_ui(self):
        """Initialize main UI layout"""
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setSpacing(10)
        
        # === TOP: Control Panel ===
        control_panel = self.create_control_panel()
        main_layout.addWidget(control_panel)
        
        # === MIDDLE: Tab widget for different views ===
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet(f"""
            QTabWidget::pane {{
                border: 1px solid {ACCENT_COLOR};
                background: {WIDGET_BG};
            }}
            QTabBar::tab {{
                background: {WIDGET_BG};
                color: {TEXT_COLOR};
                padding: 8px 16px;
                margin-right: 2px;
            }}
            QTabBar::tab:selected {{
                background: {ACCENT_COLOR};
                color: black;
            }}
        """)
        
        # Tab 1: Overview Dashboard
        self.overview_tab = self.create_overview_tab()
        self.tabs.addTab(self.overview_tab, "📊 Overview")
        
        # Tab 2: AMS Module Details
        self.ams_tab = self.create_ams_tab()
        self.tabs.addTab(self.ams_tab, "🔋 AMS Modules")
        
        main_layout.addWidget(self.tabs, stretch=3)
        
        # === BOTTOM: Log Window ===
        log_frame = self.create_log_frame()
        main_layout.addWidget(log_frame, stretch=1)
        
        # === ATTRIBUTION ===
        attribution = QLabel("Andrés Sánchez de Ágreda © 2025/2026 - ISC Formula Student Telemetry System")
        attribution.setAlignment(Qt.AlignCenter)
        attribution.setStyleSheet(f"color: {ACCENT_COLOR}; font-size: 10px; padding: 5px;")
        main_layout.addWidget(attribution)
    
    def create_control_panel(self):
        """Create top control panel with config and buttons"""
        panel = QGroupBox("Control Panel")
        panel.setStyleSheet(f"""
            QGroupBox {{
                border: 2px solid {ACCENT_COLOR};
                border-radius: 5px;
                margin-top: 10px;
                font-weight: bold;
                background: {WIDGET_BG};
            }}
            QGroupBox::title {{
                color: {ACCENT_COLOR};
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }}
        """)
        
        layout = QHBoxLayout()
        
        # Config inputs
        config_layout = QGridLayout()
        config_layout.addWidget(QLabel("Piloto:"), 0, 0)
        self.input_pilot = QLineEdit("Piloto_Test")
        config_layout.addWidget(self.input_pilot, 0, 1)
        
        config_layout.addWidget(QLabel("Circuito:"), 1, 0)
        self.input_circuit = QLineEdit("Circuito_Test")
        config_layout.addWidget(self.input_circuit, 1, 1)
        
        config_layout.addWidget(QLabel("Puerto:"), 0, 2)
        self.combo_port = QComboBox()
        self.refresh_ports()
        config_layout.addWidget(self.combo_port, 0, 3)
        
        config_layout.addWidget(QLabel("Baudrate:"), 1, 2)
        self.input_baud = QLineEdit("115200")
        config_layout.addWidget(self.input_baud, 1, 3)
        
        layout.addLayout(config_layout)
        
        # Buttons
        btn_layout = QVBoxLayout()
        
        self.btn_refresh = QPushButton("🔄 Refresh Ports")
        self.btn_refresh.clicked.connect(self.refresh_ports)
        btn_layout.addWidget(self.btn_refresh)
        
        self.btn_start = QPushButton("▶ Start Reception")
        self.btn_start.setStyleSheet(f"background-color: {ACCENT_COLOR}; color: black; font-weight: bold;")
        self.btn_start.clicked.connect(self.start_reception)
        btn_layout.addWidget(self.btn_start)
        
        self.btn_stop = QPushButton("⏹ Stop Reception")
        self.btn_stop.setEnabled(False)
        self.btn_stop.clicked.connect(self.stop_reception)
        btn_layout.addWidget(self.btn_stop)
        
        # Sub-window buttons
        btn_motor = QPushButton("🔧 Motor/Inverter")
        btn_motor.clicked.connect(self.open_motor_window)
        btn_layout.addWidget(btn_motor)
        
        btn_accu = QPushButton("🔋 Accumulator")
        btn_accu.clicked.connect(self.open_accu_window)
        btn_layout.addWidget(btn_accu)
        
        btn_driver = QPushButton("🏎️ Driver")
        btn_driver.clicked.connect(self.open_driver_window)
        btn_layout.addWidget(btn_driver)
        
        layout.addLayout(btn_layout)
        panel.setLayout(layout)
        return panel
    
    def create_overview_tab(self):
        """Create overview dashboard with key metrics"""
        widget = QWidget()
        layout = QVBoxLayout()
        
        # Status banner
        self.status_label = QLabel("⚪ IDLE - Waiting for data...")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet(f"""
            background: {WIDGET_BG};
            color: {TEXT_COLOR};
            font-size: 16px;
            font-weight: bold;
            padding: 10px;
            border: 2px solid gray;
            border-radius: 5px;
        """)
        layout.addWidget(self.status_label)
        
        # Key metrics grid
        metrics_grid = QGridLayout()
        
        # DC Bus Voltage
        self.lbl_dc_bus = self.create_metric_label("DC Bus", "--- V", ACCENT_COLOR)
        metrics_grid.addWidget(self.lbl_dc_bus, 0, 0)
        
        # RPM
        self.lbl_rpm = self.create_metric_label("RPM", "---", ACCENT_COLOR)
        metrics_grid.addWidget(self.lbl_rpm, 0, 1)
        
        # Torque
        self.lbl_torque = self.create_metric_label("Torque", "--- Nm", ACCENT_COLOR)
        metrics_grid.addWidget(self.lbl_torque, 0, 2)
        
        # Current
        self.lbl_current = self.create_metric_label("Current", "--- A", ACCENT_COLOR)
        metrics_grid.addWidget(self.lbl_current, 0, 3)
        
        # Min Cell Voltage
        self.lbl_min_cell = self.create_metric_label("Min Cell", "--- mV", WARNING_COLOR)
        metrics_grid.addWidget(self.lbl_min_cell, 1, 0)
        
        # Stack Voltage
        self.lbl_stack = self.create_metric_label("Stack", "--- V", ACCENT_COLOR)
        metrics_grid.addWidget(self.lbl_stack, 1, 1)
        
        # Max Temp
        self.lbl_max_temp = self.create_metric_label("Max Temp", "--- °C", WARNING_COLOR)
        metrics_grid.addWidget(self.lbl_max_temp, 1, 2)
        
        # Throttle
        self.lbl_throttle = self.create_metric_label("Throttle", "--- %", ACCENT_COLOR)
        metrics_grid.addWidget(self.lbl_throttle, 1, 3)
        
        layout.addLayout(metrics_grid)
        
        # Plots
        plot_layout = QHBoxLayout()
        
        self.plot_rpm = MplCanvas(title="RPM History")
        plot_layout.addWidget(self.plot_rpm)
        
        self.plot_voltage = MplCanvas(title="Min Cell Voltage")
        plot_layout.addWidget(self.plot_voltage)
        
        layout.addLayout(plot_layout)
        
        widget.setLayout(layout)
        return widget
    
    def create_ams_tab(self):
        """Create AMS module overview tab"""
        widget = QWidget()
        layout = QVBoxLayout()
        
        # Module summary
        summary_label = QLabel("AMS Module Summary")
        summary_label.setStyleSheet(f"font-size: 14px; font-weight: bold; color: {ACCENT_COLOR};")
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
        global_layout = QGridLayout()
        
        self.lbl_global_min = QLabel("Global Min: --- mV")
        self.lbl_global_max = QLabel("Global Max: --- mV")
        self.lbl_stack_total = QLabel("Stack Total: --- V")
        self.lbl_ams_current = QLabel("Current: --- A")
        
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
                border: 2px solid {ACCENT_COLOR};
                border-radius: 5px;
                margin-top: 10px;
                background: {WIDGET_BG};
            }}
            QGroupBox::title {{
                color: {ACCENT_COLOR};
                font-weight: bold;
            }}
        """)
        
        layout = QVBoxLayout()
        
        lbl_min_v = QLabel("Min: --- mV")
        lbl_max_v = QLabel("Max: --- mV")
        lbl_min_t = QLabel("Min T: --- °C")
        lbl_max_t = QLabel("Max T: --- °C")
        lbl_age = QLabel("Age: ---")
        
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
    
    def create_metric_label(self, title, value, color):
        """Create a styled metric display label"""
        frame = QFrame()
        frame.setStyleSheet(f"""
            QFrame {{
                background: {WIDGET_BG};
                border: 2px solid {color};
                border-radius: 8px;
                padding: 10px;
            }}
        """)
        
        layout = QVBoxLayout()
        
        title_lbl = QLabel(title)
        title_lbl.setAlignment(Qt.AlignCenter)
        title_lbl.setStyleSheet(f"color: {color}; font-size: 12px; font-weight: bold;")
        
        value_lbl = QLabel(value)
        value_lbl.setAlignment(Qt.AlignCenter)
        value_lbl.setStyleSheet(f"color: {TEXT_COLOR}; font-size: 20px; font-weight: bold;")
        
        layout.addWidget(title_lbl)
        layout.addWidget(value_lbl)
        frame.setLayout(layout)
        
        # Store value label as attribute
        frame.value_label = value_lbl
        return frame
    
    def create_log_frame(self):
        """Create log display frame"""
        frame = QGroupBox("System Log")
        frame.setStyleSheet(f"""
            QGroupBox {{
                border: 2px solid {ACCENT_COLOR};
                border-radius: 5px;
                margin-top: 10px;
                background: {WIDGET_BG};
            }}
            QGroupBox::title {{
                color: {ACCENT_COLOR};
                font-weight: bold;
            }}
        """)
        
        layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setStyleSheet(f"background: {PLOT_BG}; color: {TEXT_COLOR}; font-family: monospace;")
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
        
        try:
            baud = int(self.input_baud.text())
        except ValueError:
            QMessageBox.warning(self, "Error", "Invalid baudrate")
            return
        
        if not port:
            QMessageBox.warning(self, "Error", "No port selected")
            return
        
        bucket_id = rtt.create_bucket(piloto, circuito, use_influx=False)
        self.append_log(f"Starting reception: {port} @ {baud} bps")
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
                    use_influx=False,
                    debug=False
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
        reason = status_info.get("reason", "")
        
        if badge == "LIVE":
            self.status_label.setText(f"🟢 LIVE - {reason}")
            self.status_label.setStyleSheet(f"""
                background: {WIDGET_BG};
                color: {ACCENT_COLOR};
                font-size: 16px;
                font-weight: bold;
                padding: 10px;
                border: 2px solid {ACCENT_COLOR};
                border-radius: 5px;
            """)
        elif badge == "STALE":
            self.status_label.setText(f"🟡 STALE - {reason}")
            self.status_label.setStyleSheet(f"""
                background: {WIDGET_BG};
                color: {WARNING_COLOR};
                font-size: 16px;
                font-weight: bold;
                padding: 10px;
                border: 2px solid {WARNING_COLOR};
                border-radius: 5px;
            """)
        else:
            self.status_label.setText(f"🔴 BAD - {reason}")
            self.status_label.setStyleSheet(f"""
                background: {WIDGET_BG};
                color: {ERROR_COLOR};
                font-size: 16px;
                font-weight: bold;
                padding: 10px;
                border: 2px solid {ERROR_COLOR};
                border-radius: 5px;
            """)
        
        # Update metrics from ID 0x600
        data_600 = data.get("0x600", {})
        self.lbl_dc_bus.value_label.setText(f"{data_600.get('dc_bus_voltage', 0):.1f} V")
        self.lbl_rpm.value_label.setText(f"{data_600.get('rpm', 0):.0f}")
        self.lbl_torque.value_label.setText(f"{data_600.get('torque_total', 0):.1f} Nm")
        self.lbl_min_cell.value_label.setText(f"{data_600.get('cell_min_v', 0):.0f} mV")
        
        data_630 = data.get("0x630", {})
        self.lbl_throttle.value_label.setText(f"{data_630.get('throttle', 0):.1f} %")
        
        data_610 = data.get("0x610", {})
        self.lbl_current.value_label.setText(f"{data_610.get('i_actual', 0):.1f} A")
        
        # Update AMS data
        ams_summary = data.get("ams_summary", {})
        self.lbl_stack.value_label.setText(f"{ams_summary.get('stack_mv', 0) / 1000:.1f} V")
        
        ams_temp = data.get("ams_temp_summary", {})
        self.lbl_max_temp.value_label.setText(f"{ams_temp.get('max_temp_c', 0):.0f} °C")
        
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
        self.plot_rpm.update_plot(data_600.get('rpm', 0))
        self.plot_voltage.update_plot(data_600.get('cell_min_v', 0))
    
    def append_log(self, msg: str):
        """Append message to log window"""
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{ts}] {msg}")
    
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
    
    def closeEvent(self, event):
        """Handle window close"""
        if self.is_receiving:
            self.stop_reception()
            time.sleep(0.5)
        event.accept()

# ============== MATPLOTLIB CANVAS ==============
class MplCanvas(FigureCanvas):
    """Matplotlib canvas for plotting"""
    def __init__(self, title="Plot", max_points=100):
        self.fig = Figure(figsize=(5, 3), facecolor=PLOT_BG)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor(PLOT_BG)
        self.ax.set_title(title, color=ACCENT_COLOR, fontweight='bold')
        self.ax.tick_params(colors=TEXT_COLOR)
        self.ax.spines['bottom'].set_color(TEXT_COLOR)
        self.ax.spines['top'].set_color(TEXT_COLOR)
        self.ax.spines['left'].set_color(TEXT_COLOR)
        self.ax.spines['right'].set_color(TEXT_COLOR)
        
        super().__init__(self.fig)
        
        self.data = []
        self.max_points = max_points
        self.line, = self.ax.plot([], [], color=ACCENT_COLOR, linewidth=2)
        self.ax.grid(True, alpha=0.3, color=TEXT_COLOR)
    
    def update_plot(self, value):
        """Update plot with new value"""
        self.data.append(value)
        if len(self.data) > self.max_points:
            self.data.pop(0)
        
        self.line.set_data(range(len(self.data)), self.data)
        self.ax.relim()
        self.ax.autoscale_view()
        self.draw()

# ============== MOTOR/INVERTER WINDOW ==============
class MotorInverterWindow(QWidget):
    """Dedicated window for motor and inverter data"""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Motor & Inverter Data")
        self.setGeometry(150, 150, 900, 600)
        self.init_ui()
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(100)
    
    def init_ui(self):
        layout = QVBoxLayout()
        
        # Title
        title = QLabel("🔧 Motor & Inverter Monitoring")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(f"font-size: 18px; font-weight: bold; color: {ACCENT_COLOR}; padding: 10px;")
        layout.addWidget(title)
        
        # Metrics grid
        grid = QGridLayout()
        
        self.lbl_rpm = QLabel("RPM: ---")
        self.lbl_torque_req = QLabel("Torque Req: --- Nm")
        self.lbl_torque_est = QLabel("Torque Est: --- Nm")
        self.lbl_i_actual = QLabel("Current: --- A")
        self.lbl_n_actual = QLabel("N Actual: ---")
        self.lbl_motor_temp = QLabel("Motor Temp: --- °C")
        self.lbl_pwrstg_temp = QLabel("Power Stage: --- °C")
        self.lbl_air_temp = QLabel("Air Temp: --- °C")
        
        grid.addWidget(self.lbl_rpm, 0, 0)
        grid.addWidget(self.lbl_torque_req, 0, 1)
        grid.addWidget(self.lbl_torque_est, 1, 0)
        grid.addWidget(self.lbl_i_actual, 1, 1)
        grid.addWidget(self.lbl_n_actual, 2, 0)
        grid.addWidget(self.lbl_motor_temp, 2, 1)
        grid.addWidget(self.lbl_pwrstg_temp, 3, 0)
        grid.addWidget(self.lbl_air_temp, 3, 1)
        
        layout.addLayout(grid)
        
        # Plots
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
        
        rpm = data_600.get('rpm', 0)
        torque_total = data_600.get('torque_total', 0)
        torque_req = data_630.get('torque_req', 0)
        torque_est = data_630.get('torque_est', 0)
        i_actual = data_610.get('i_actual', 0)
        n_actual = data_610.get('n_actual', 0)
        motor_temp = data_610.get('motor_temp', 0)
        pwrstg_temp = data_610.get('pwrstg_temp', 0)
        air_temp = data_610.get('air_temp', 0)
        
        self.lbl_rpm.setText(f"RPM: {rpm:.0f}")
        self.lbl_torque_req.setText(f"Torque Req: {torque_req:.1f} Nm")
        self.lbl_torque_est.setText(f"Torque Est: {torque_est:.1f} Nm")
        self.lbl_i_actual.setText(f"Current: {i_actual:.1f} A")
        self.lbl_n_actual.setText(f"N Actual: {n_actual:.0f}")
        self.lbl_motor_temp.setText(f"Motor Temp: {motor_temp:.0f} °C")
        self.lbl_pwrstg_temp.setText(f"Power Stage: {pwrstg_temp:.0f} °C")
        self.lbl_air_temp.setText(f"Air Temp: {air_temp:.0f} °C")
        
        self.plot_torque.update_plot(torque_total)
        self.plot_current.update_plot(i_actual)

# ============== ACCUMULATOR WINDOW WITH HEATMAPS ==============
class AccumulatorWindow(QWidget):
    """Accumulator window with temperature heatmaps for all 5 modules"""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Accumulator Data & Heatmaps")
        self.setGeometry(150, 150, 1400, 800)
        self.init_ui()
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(500)  # Slower update for heatmaps
    
    def init_ui(self):
        layout = QVBoxLayout()
        
        # Title
        title = QLabel("🔋 Accumulator Monitoring")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(f"font-size: 18px; font-weight: bold; color: {ACCENT_COLOR}; padding: 10px;")
        layout.addWidget(title)
        
        # Summary stats
        stats_layout = QHBoxLayout()
        self.lbl_stack_v = QLabel("Stack: --- V")
        self.lbl_current = QLabel("Current: --- A")
        self.lbl_min_cell = QLabel("Min Cell: --- mV")
        self.lbl_max_cell = QLabel("Max Cell: --- mV")
        
        stats_layout.addWidget(self.lbl_stack_v)
        stats_layout.addWidget(self.lbl_current)
        stats_layout.addWidget(self.lbl_min_cell)
        stats_layout.addWidget(self.lbl_max_cell)
        layout.addLayout(stats_layout)
        
        # Heatmaps for 5 modules
        heatmap_layout = QGridLayout()
        self.heatmaps = []
        
        for i in range(NUM_MODULES):
            heatmap = HeatmapCanvas(title=f"Module {i} Temperature Map")
            heatmap_layout.addWidget(heatmap, i // 3, i % 3)
            self.heatmaps.append(heatmap)
        
        layout.addLayout(heatmap_layout)
        self.setLayout(layout)
    
    def update_data(self):
        self.lbl_stack_v.setText(f"Stack: {rtt.ams_stack_total_mv / 1000:.1f} V")
        self.lbl_current.setText(f"Current: {rtt.ams_current_dA / 10:.1f} A")
        self.lbl_min_cell.setText(f"Min Cell: {rtt.ams_global_min_mv} mV")
        self.lbl_max_cell.setText(f"Max Cell: {rtt.ams_global_max_mv} mV")
        
        # Update heatmaps
        for i, heatmap in enumerate(self.heatmaps):
            mod = rtt.get_ams_module_data(i)
            if mod:
                heatmap.update_heatmap(mod.temps_c)

# ============== HEATMAP CANVAS ==============
class HeatmapCanvas(FigureCanvas):
    """Canvas for temperature heatmap"""
    def __init__(self, title="Heatmap"):
        self.fig = Figure(figsize=(4, 3), facecolor=PLOT_BG)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor(PLOT_BG)
        self.ax.set_title(title, color=ACCENT_COLOR, fontsize=10, fontweight='bold')
        
        super().__init__(self.fig)
        
        # Create grid for 38 sensors (6x7 with padding)
        self.grid_data = np.zeros((6, 7))
        self.im = self.ax.imshow(self.grid_data, cmap='hot', vmin=20, vmax=60, aspect='auto')
        self.fig.colorbar(self.im, ax=self.ax, label='°C')
        self.ax.set_xticks([])
        self.ax.set_yticks([])
    
    def update_heatmap(self, temps):
        """Update heatmap with temperature data"""
        # Reshape temps to 6x7 grid (42 positions, 38 sensors + 4 padding)
        temps_array = np.array(temps[:TEMPS_PER_MODULE])
        # Replace NaN with 0 for visualization
        temps_array = np.nan_to_num(temps_array, nan=0.0)
        
        # Pad to 42 elements
        padded = np.pad(temps_array, (0, 42 - len(temps_array)), constant_values=0)
        grid = padded.reshape((6, 7))
        
        self.im.set_data(grid)
        self.im.set_clim(vmin=max(20, grid.min()), vmax=min(60, grid.max()))
        self.draw()

# ============== DRIVER WINDOW ==============
class DriverWindow(QWidget):
    """Driver inputs and sensor readings window"""
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
        
        # Title
        title = QLabel("🏎️ Driver Interface Monitoring")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(f"font-size: 18px; font-weight: bold; color: {ACCENT_COLOR}; padding: 10px;")
        layout.addWidget(title)
        
        # Metrics
        grid = QGridLayout()
        
        self.lbl_throttle_raw1 = QLabel("Throttle Raw 1: ---")
        self.lbl_throttle_raw2 = QLabel("Throttle Raw 2: ---")
        self.lbl_throttle_pct = QLabel("Throttle %: ---")
        self.lbl_brake_raw = QLabel("Brake Raw: ---")
        self.lbl_brake_pct = QLabel("Brake %: ---")
        self.lbl_start_btn = QLabel("Start Button: ---")
        self.lbl_precharge_btn = QLabel("Precharge: ---")
        
        grid.addWidget(self.lbl_throttle_raw1, 0, 0)
        grid.addWidget(self.lbl_throttle_raw2, 0, 1)
        grid.addWidget(self.lbl_throttle_pct, 1, 0)
        grid.addWidget(self.lbl_brake_raw, 1, 1)
        grid.addWidget(self.lbl_brake_pct, 2, 0)
        grid.addWidget(self.lbl_start_btn, 2, 1)
        grid.addWidget(self.lbl_precharge_btn, 3, 0)
        
        layout.addLayout(grid)
        
        # Plots
        plot_layout = QHBoxLayout()
        self.plot_throttle = MplCanvas(title="Throttle %")
        self.plot_brake = MplCanvas(title="Brake %")
        plot_layout.addWidget(self.plot_throttle)
        plot_layout.addWidget(self.plot_brake)
        layout.addLayout(plot_layout)
        
        self.setLayout(layout)
    
    def update_data(self):
        data = rtt.get_latest_data()
        
        data_600 = data.get("0x600", {})
        data_620 = data.get("0x620", {})
        data_630 = data.get("0x630", {})
        
        throttle_raw1 = data_600.get('throttle_raw1', 0)
        throttle_raw2 = data_600.get('throttle_raw2', 0)
        throttle_pct = data_630.get('throttle', 0)
        brake_raw = data_620.get('brake_raw', 0)
        brake_pct = data_630.get('brake', 0)
        start_btn = data_620.get('start_button', 0)
        precharge_btn = data_620.get('precharge_button', 0)
        
        self.lbl_throttle_raw1.setText(f"Throttle Raw 1: {throttle_raw1:.0f}")
        self.lbl_throttle_raw2.setText(f"Throttle Raw 2: {throttle_raw2:.0f}")
        self.lbl_throttle_pct.setText(f"Throttle %: {throttle_pct:.1f}")
        self.lbl_brake_raw.setText(f"Brake Raw: {brake_raw:.0f}")
        self.lbl_brake_pct.setText(f"Brake %: {brake_pct:.1f}")
        self.lbl_start_btn.setText(f"Start Button: {'PRESSED' if start_btn else 'RELEASED'}")
        self.lbl_precharge_btn.setText(f"Precharge: {'ACTIVE' if precharge_btn else 'INACTIVE'}")
        
        self.plot_throttle.update_plot(throttle_pct)
        self.plot_brake.update_plot(brake_pct)

# ============== MAIN ==============
def main():
    print("=== Iniciando ISCmetrics UI ===")
    try:
        app = QApplication(sys.argv)
        app.setStyle('Fusion')
    
        window = MainWindow()
        window.show()

        sys.exit(app.exec_())
    
   
    except Exception as e:
        print(f"FATAL ERROR: {e}") 
        sys.exit(1)
    

if __name__ == "__main__":
    main()
