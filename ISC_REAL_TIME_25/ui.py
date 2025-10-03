# ui.py
"""
ISCmetrics - Real-Time Telemetry UI
- Fullscreen dark UI
- Select COM port & baudrate
- Optional InfluxDB
- Excel logging to ./logs via backend
- Debug toggle that streams backend logs into the UI

NOVEDADES UI:
- Badge LIVE / STALE / BAD en cabecera (con motivo)
- Congelación de widgets cuando STALE/BAD
- Muestra acum temp max (0x640 v3) y, si llega 0x645, muestra sondas DS18B20 (t1..t4, avg)
- Mapea temps de inversor desde 0x610 (motor, IGBT, aire), y rpm/corriente actuales
- 4 gráficas compactas: Acelerador, Freno, DC Bus Voltage y DS Temp (avg)
"""

import os
import sys
import time
import queue
import threading
import subprocess
import tkinter as tk
import tkinter.scrolledtext as st
from tkinter import ttk, messagebox
import logging

# Backend
import ISC_RTT_serial as RTTT

# ---- Matplotlib in Tk ----
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
from collections import deque

# Optional logo
try:
    from PIL import Image, ImageTk
except Exception:
    Image = None
    ImageTk = None

# For Linux headless issues
if sys.platform.startswith("linux") and "DISPLAY" not in os.environ:
    os.environ["DISPLAY"] = ":0"


# -------- Tunables for header logo & spacing --------
LOGO_MAX_HEIGHT = 44
LOGO_VPAD_PX    = 6
ROW0_PADY       = 2
ROW1_PADY       = (2, 0)
ROW2_PADY       = (0, 6)
GRAPHS_PADY     = 4
STATUS_PADY     = 0


# -------- path + platform helpers --------
def resource_path(*parts):
    candidates = []
    try:
        base = getattr(sys, "_MEIPASS", os.path.dirname(os.path.abspath(__file__)))
        candidates.append(os.path.join(base, *parts))
    except Exception:
        pass
    candidates.append(os.path.join("ISC_REAL_TIME_25", *parts))   # legacy
    candidates.append(os.path.join(os.getcwd(), *parts))           # cwd fallback
    for c in candidates:
        if os.path.exists(c):
            return c
    return candidates[0]


def is_windows():
    return sys.platform.startswith("win")


def load_logo_with_padding(png_path, max_h=LOGO_MAX_HEIGHT, vpad_px=LOGO_VPAD_PX):
    if not (Image and ImageTk):
        return None
    img = Image.open(png_path).convert("RGBA")
    w, h = img.size
    if h > max_h:
        new_w = max(1, int(w * (max_h / float(h))))
        try:
            img = img.resize((new_w, max_h), Image.Resampling.LANCZOS)
        except Exception:
            img = img.resize((new_w, max_h), Image.LANCZOS)
    pad_h = img.height + 2 * vpad_px
    padded = Image.new("RGBA", (img.width, pad_h), (0, 0, 0, 0))
    padded.paste(img, (0, vpad_px), img)
    return padded


# -------- logger -> Tk text handler --------
class TkTextHandler(logging.Handler):
    """Send logging records to a Tkinter ScrolledText safely."""
    def __init__(self, text_widget: st.ScrolledText):
        super().__init__()
        self.text_widget = text_widget

    def emit(self, record):
        try:
            msg = self.format(record)
            ts = time.strftime("%H:%M:%S")

            def append():
                self.text_widget.insert(tk.END, f"[{ts}] {msg}\n")
                self.text_widget.see(tk.END)
                # trim lines
                lines = self.text_widget.get("1.0", tk.END).split("\n")
                if len(lines) > 600:
                    self.text_widget.delete("1.0", f"{len(lines)-600}.0")

            self.text_widget.after(0, append)
        except Exception:
            self.handleError(record)


class TelemetryUI:
    def __init__(self):
        # Root FIRST
        self.root = tk.Tk()
        self.root.title("ISCmetrics")
        self.root.attributes("-fullscreen", True)
        self.root.configure(bg="#101010")

        # Early log buffer
        self._early_logs = []

        # Keep references to images
        self.tk_logo = None

        self.setup_data_structures()
        self.setup_ui()
        self._flush_early_logs()
        self.setup_logging_bridge()

    # -------------------- Early logging helpers --------------------
    def _elog(self, message: str):
        try:
            print(message)
        except Exception:
            pass
        self._early_logs.append(message)

    def _flush_early_logs(self):
        if hasattr(self, "telemetry_display"):
            for m in self._early_logs:
                self.log_message(m)
            self._early_logs = []

    # -------------------- Infra de estado --------------------
    def setup_data_structures(self):
        self.data_queue = queue.Queue()

        # Flags/threads
        self.receiving_flag = False
        self.stop_data = False
        self.receiving_thread = None
        self.ui_update_thread = None

        # Historias para plots (compactas)
        self.maxlen_hist = 240  # ~24 s a 10 Hz aprox
        self.throttle_history = deque(maxlen=self.maxlen_hist)
        self.brake_history    = deque(maxlen=self.maxlen_hist)
        self.vdc_history      = deque(maxlen=self.maxlen_hist)
        self.dsavg_history    = deque(maxlen=self.maxlen_hist)
        self.time_history     = deque(maxlen=self.maxlen_hist)

        # Listas demo
        self.pilots_list = ["J. Landa", "N. Huertas", "A. Sanchez", "F. Tobar"]
        self.circuits_list = ["Boadilla", "Jarama", "Montmeló", "Hockenheim"]

        # Tk variables
        self.selected_port = tk.StringVar(self.root, value="")
        self.selected_baud = tk.IntVar(self.root, value=115200)
        self.use_influx_var = tk.BooleanVar(self.root, value=False)
        self.debug_var = tk.BooleanVar(self.root, value=True)
        self.piloto_var = tk.StringVar(self.root, value=self.pilots_list[0])
        self.circuito_var = tk.StringVar(self.root, value=self.circuits_list[0])
        self.status_var = tk.StringVar(self.root, value="Listo.")

        # Estado de badge
        self.link_badge = tk.StringVar(self.root, value="STALE")
        self.link_reason = tk.StringVar(self.root, value="inicio")

        # Congelar UI cuando STALE/BAD
        self.freeze_ui = True

    # -------------------- UI raíz --------------------
    def setup_ui(self):
        # Grid root (da más peso a la fila de gráficas para evitar recortes)
        for i in range(9):
            self.root.grid_columnconfigure(i, weight=1)
        # Rows: 0 header, 1 selects, 2 io, 3-4 panels, 5 graphs, 6 log, 7 status
        self.root.grid_rowconfigure(0, weight=0)
        self.root.grid_rowconfigure(1, weight=0)
        self.root.grid_rowconfigure(2, weight=0)
        self.root.grid_rowconfigure(3, weight=0)
        self.root.grid_rowconfigure(4, weight=0)
        self.root.grid_rowconfigure(5, weight=2)  # más espacio para gráficas
        self.root.grid_rowconfigure(6, weight=1)
        self.root.grid_rowconfigure(7, weight=0)

        self.create_header()
        self.create_controls()
        self.create_data_displays()
        self.create_graphs()
        self.create_statusbar()
        self.setup_bindings()

        # Rellenar combo de puertos al inicio
        self.refresh_ports()

    def setup_bindings(self):
        self.root.bind("<Escape>", self.close_fullscreen)
        self.root.bind("<F11>", self.toggle_fullscreen)
        self.root.bind("<Control-m>", lambda e: self.minimize_window())

    # -------------------- Cabecera --------------------
    def create_header(self):
        header_bar = tk.Frame(self.root, bg="#101010")
        header_bar.grid(row=0, column=0, columnspan=9, sticky="ew", pady=ROW0_PADY, padx=10)
        header_bar.grid_columnconfigure(0, weight=0)
        header_bar.grid_columnconfigure(1, weight=1)
        header_bar.grid_columnconfigure(2, weight=0)

        left_group = tk.Frame(header_bar, bg="#101010")
        left_group.grid(row=0, column=0, sticky="nw")
        right_frame = tk.Frame(header_bar, bg="#101010")
        right_frame.grid(row=0, column=2, sticky="ne")

        # Resolve assets
        ico_path = resource_path("isc_logo.ico")
        png_path = resource_path("isc_logo.png")

        if is_windows() and os.path.exists(ico_path):
            try:
                self.root.iconbitmap(ico_path)
                self._elog(f"[ICON] Using ICO for taskbar: {ico_path}")
            except Exception as e:
                self._elog(f"[ICON] iconbitmap failed: {e}")

        self.tk_logo = None
        if os.path.exists(png_path):
            try:
                if Image and ImageTk:
                    pil_img = load_logo_with_padding(png_path, LOGO_MAX_HEIGHT, LOGO_VPAD_PX)
                    if pil_img is None:
                        self.tk_logo = tk.PhotoImage(file=png_path)
                    else:
                        self.tk_logo = ImageTk.PhotoImage(pil_img)
                else:
                    self.tk_logo = tk.PhotoImage(file=png_path)
            except Exception as e:
                self._elog(f"[ICON] Failed to load PNG logo: {png_path} ({e})")

        if self.tk_logo:
            try:
                self.root.iconphoto(True, self.tk_logo)
            except Exception as e:
                self._elog(f"[ICON] iconphoto failed: {e}")

        if self.tk_logo:
            tk.Label(left_group, image=self.tk_logo, bg="#101010").pack(side="left")
        else:
            tk.Label(left_group, text=" ", bg="#101010").pack(side="left")

        title_lbl = tk.Label(
            left_group,
            text="ISCmetrics",
            font=("Inter", 17, "bold"),
            fg="#FFFFFF",
            bg="#101010",
            padx=8
        )
        title_lbl.pack(side="left")

        # Right side controls (badge + buttons)
        self.badge_label = tk.Label(
            right_frame, textvariable=self.link_badge, font=("Inter", 11, "bold"),
            fg="#000000", bg="#808080", padx=8, pady=3, relief="flat", width=8
        )
        self.badge_label.pack(side="left", padx=(0, 8))

        self.badge_reason_label = tk.Label(
            right_frame, textvariable=self.link_reason, font=("Inter", 9),
            fg="#BBBBBB", bg="#101010", anchor="e", width=24
        )
        self.badge_reason_label.pack(side="left", padx=(0, 8))

        btn_min = tk.Button(
            right_frame, text="—", font=("Inter", 13, "bold"),
            fg="#FFFFFF", bg="#303030", activebackground="#505050",
            width=3, borderwidth=0, command=self.minimize_window
        )
        btn_min.pack(side="left", padx=(0, 6))

        btn_close = tk.Button(
            right_frame, text="×", font=("Inter", 13, "bold"),
            fg="#FFFFFF", bg="#C43131", activebackground="#E04B4B",
            width=3, borderwidth=0, command=self.close_window
        )
        btn_close.pack(side="left")

        self._elog(f"[ICON] CWD: {os.getcwd()}")
        self._elog(f"[ICON] Resolved ICO: {ico_path} (exists={os.path.exists(ico_path)})")
        self._elog(f"[ICON] Resolved PNG: {png_path} (exists={os.path.exists(png_path)})")

    # -------------------- Controles superiores --------------------
    def create_controls(self):
        selects_frame = tk.Frame(self.root, bg="#101010")
        selects_frame.grid(row=1, column=0, columnspan=9, sticky="n", pady=ROW1_PADY)

        form = tk.Frame(selects_frame, bg="#101010")
        form.pack(anchor="w")

        # Piloto
        tk.Label(form, text="Piloto", font=("Inter", 13), fg="#FFFFFF", bg="#101010").grid(
            row=0, column=0, padx=8, pady=(2, 2), sticky="s"
        )
        self.pilot_menu = tk.OptionMenu(form, self.piloto_var, *self.pilots_list)
        self.pilot_menu.config(font=("Inter", 12), fg="#00FF00", bg="#202020", highlightthickness=0, bd=0)
        self.pilot_menu.grid(row=1, column=0, padx=8, pady=(0, 6), sticky="ew")

        # Circuito
        tk.Label(form, text="Circuito", font=("Inter", 13), fg="#FFFFFF", bg="#101010").grid(
            row=0, column=1, padx=8, pady=(2, 2), sticky="s"
        )
        self.circuit_menu = tk.OptionMenu(form, self.circuito_var, *self.circuits_list)
        self.circuit_menu.config(font=("Inter", 12), fg="#00FF00", bg="#202020", highlightthickness=0, bd=0)
        self.circuit_menu.grid(row=1, column=1, padx=8, pady=(0, 6), sticky="ew")

        # Puerto serie + Baud + Influx + Debug
        io_frame = tk.Frame(self.root, bg="#101010")
        io_frame.grid(row=2, column=0, columnspan=9, sticky="n", pady=ROW2_PADY)

        tk.Label(io_frame, text="Puerto", font=("Inter", 12), fg="#FFFFFF", bg="#101010").grid(
            row=0, column=0, padx=(0, 6), pady=2
        )
        self.port_combo = ttk.Combobox(io_frame, textvariable=self.selected_port, width=24, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=(0, 6), pady=2)

        btn_refresh = tk.Button(
            io_frame, text="Actualizar", font=("Inter", 12),
            fg="#FFFFFF", bg="#303030", activebackground="#505050",
            command=self.refresh_ports
        )
        btn_refresh.grid(row=0, column=2, padx=(0, 12), pady=2)

        tk.Label(io_frame, text="Baud", font=("Inter", 12), fg="#FFFFFF", bg="#101010").grid(
            row=0, column=3, padx=(0, 6), pady=2
        )
        self.baud_entry = tk.Entry(io_frame, textvariable=self.selected_baud, width=10, bg="#202020", fg="#00FF00")
        self.baud_entry.grid(row=0, column=4, padx=(0, 12), pady=2)

        self.influx_chk = tk.Checkbutton(
            io_frame, text="Usar InfluxDB", variable=self.use_influx_var,
            onvalue=True, offvalue=False, font=("Inter", 12),
            fg="#FFFFFF", bg="#101010", activebackground="#101010",
            selectcolor="#202020"
        )
        self.influx_chk.grid(row=0, column=5, padx=(0, 12), pady=2, sticky="w")

        self.debug_chk = tk.Checkbutton(
            io_frame, text="Debug", variable=self.debug_var,
            onvalue=True, offvalue=False, font=("Inter", 12),
            fg="#FFFFFF", bg="#101010", activebackground="#101010",
            selectcolor="#202020", command=self._apply_debug_level
        )
        self.debug_chk.grid(row=0, column=6, padx=(0, 12), pady=2, sticky="w")

        self.run_button = tk.Button(
            io_frame, text="INICIAR", font=("Inter", 14, "bold"),
            fg="#FFFFFF", bg="#006400", command=self.start_receiving, relief="raised", bd=2, width=12
        )
        self.run_button.grid(row=0, column=7, padx=6)

        self.stop_button = tk.Button(
            io_frame, text="PARAR", font=("Inter", 14, "bold"),
            fg="#FFFFFF", bg="#404040", command=self.stop_receiving,
            relief="raised", bd=2, width=12, state="disabled"
        )
        self.stop_button.grid(row=0, column=8, padx=6)

        tools_frame = tk.Frame(self.root, bg="#101010")
        tools_frame.grid(row=2, column=0, columnspan=9, sticky="s", pady=(12, 0))
        open_logs_btn = tk.Button(
            tools_frame, text="Abrir carpeta logs", font=("Inter", 11),
            fg="#FFFFFF", bg="#303030", activebackground="#505050",
            command=self.open_logs_folder
        )
        open_logs_btn.pack()

    # -------------------- Cuadros de datos --------------------
    def create_data_displays(self):
        # ACUMULADOR
        accu_frame = tk.LabelFrame(self.root, text="ACUMULADOR",
                                   font=("Inter", 12, "bold"), fg="#00FF00",
                                   bg="#101010", bd=2)
        accu_frame.grid(row=3, column=0, columnspan=2, padx=5, pady=4, sticky="nsew")

        self.accu_voltage_label = tk.Label(accu_frame, text="DC Bus: -- V",
                                           font=("Inter", 14), fg="#FFFFFF", bg="#101010")
        self.accu_voltage_label.pack(pady=2)

        self.accu_current_label = tk.Label(accu_frame, text="Corriente: -- A",
                                           font=("Inter", 14), fg="#FFFFFF", bg="#101010")
        self.accu_current_label.pack(pady=2)

        self.accu_power_label = tk.Label(accu_frame, text="Potencia: -- W",
                                         font=("Inter", 14), fg="#FFFFFF", bg="#101010")
        self.accu_power_label.pack(pady=2)

        # TEMPERATURAS
        temp_frame = tk.LabelFrame(self.root, text="TEMPERATURAS",
                                   font=("Inter", 12, "bold"), fg="#FFA500",
                                   bg="#101010", bd=2)
        temp_frame.grid(row=3, column=2, columnspan=2, padx=5, pady=4, sticky="nsew")

        self.temp_accu_label = tk.Label(temp_frame, text="Accu Max: -- °C",
                                        font=("Inter", 14), fg="#FFFFFF", bg="#101010")
        self.temp_accu_label.pack(pady=2)

        self.temp_motor_label = tk.Label(temp_frame, text="Motor: -- °C",
                                         font=("Inter", 14), fg="#FFFFFF", bg="#101010")
        self.temp_motor_label.pack(pady=2)

        self.temp_inverter_label = tk.Label(temp_frame, text="Inversor: -- °C",
                                            font=("Inter", 14), fg="#FFFFFF", bg="#101010")
        self.temp_inverter_label.pack(pady=2)

        self.temp_air_label = tk.Label(temp_frame, text="Aire: -- °C",
                                       font=("Inter", 12), fg="#AAAAAA", bg="#101010")
        self.temp_air_label.pack(pady=2)

        # DS18B20 detail (if 0x645 present)
        self.ds_box = tk.Frame(temp_frame, bg="#101010")
        self.ds_box.pack(pady=(6, 2), fill="x")
        self.ds_labels = []
        for i in range(4):
            lbl = tk.Label(self.ds_box, text=f"DS{i+1}: -- °C", font=("Inter", 11),
                           fg="#CCCCCC", bg="#101010")
            lbl.grid(row=0, column=i, padx=6)
            self.ds_labels.append(lbl)
        self.ds_summary_label = tk.Label(temp_frame, text="DS avg: -- °C",
                                         font=("Inter", 11), fg="#BBBBBB", bg="#101010")
        self.ds_summary_label.pack(pady=(2, 0))

        # ESTADO INVERSOR
        inverter_frame = tk.LabelFrame(self.root, text="ESTADO INVERSOR",
                                       font=("Inter", 12, "bold"), fg="#FF6B6B",
                                       bg="#101010", bd=2)
        inverter_frame.grid(row=4, column=0, columnspan=2, padx=5, pady=4, sticky="nsew")

        self.inverter_status_label = tk.Label(inverter_frame, text="Estado: DESCONECTADO",
                                              font=("Inter", 14, "bold"), fg="#FF0000", bg="#101010")
        self.inverter_status_label.pack(pady=5)

        self.inverter_errors_label = tk.Label(inverter_frame, text="Errores: --",
                                              font=("Inter", 12), fg="#FFFFFF", bg="#101010")
        self.inverter_errors_label.pack(pady=2)

        self.n_i_label = tk.Label(inverter_frame, text="n_actual: -- rpm | i_actual: -- A",
                                  font=("Inter", 12), fg="#FFFFFF", bg="#101010")
        self.n_i_label.pack(pady=2)

        # TORQUE
        torque_frame = tk.LabelFrame(self.root, text="TORQUE",
                                     font=("Inter", 12, "bold"), fg="#4ECDC4",
                                     bg="#101010", bd=2)
        torque_frame.grid(row=4, column=2, columnspan=2, padx=5, pady=4, sticky="nsew")

        self.torque_req_label = tk.Label(torque_frame, text="Solicitado: -- Nm",
                                         font=("Inter", 14), fg="#FFFFFF", bg="#101010")
        self.torque_req_label.pack(pady=2)

        self.torque_est_label = tk.Label(torque_frame, text="Estimado: -- Nm",
                                         font=("Inter", 14), fg="#FFFFFF", bg="#101010")
        self.torque_est_label.pack(pady=2)

        # ACELERADOR (raw/escalado/clamped)
        accel_frame = tk.LabelFrame(self.root, text="ACELERADOR",
                                    font=("Inter", 12, "bold"), fg="#00BFFF",
                                    bg="#101010", bd=2)
        accel_frame.grid(row=4, column=4, columnspan=2, padx=5, pady=4, sticky="nsew")

        self.accel_raw1_label = tk.Label(accel_frame, text="Raw1: --", font=("Inter", 13), fg="#FFFFFF", bg="#101010")
        self.accel_raw1_label.pack(pady=2)
        self.accel_raw2_label = tk.Label(accel_frame, text="Raw2: --", font=("Inter", 13), fg="#FFFFFF", bg="#101010")
        self.accel_raw2_label.pack(pady=2)
        self.accel_scaled_label = tk.Label(accel_frame, text="Escalado: -- %", font=("Inter", 13), fg="#FFFFFF", bg="#101010")
        self.accel_scaled_label.pack(pady=2)
        self.accel_clamped_label = tk.Label(accel_frame, text="Clamped: -- %", font=("Inter", 13, "bold"), fg="#FFFFFF", bg="#101010")
        self.accel_clamped_label.pack(pady=2)

        # LOG
        log_frame = tk.LabelFrame(self.root, text="LOG",
                                  font=("Inter", 12, "bold"), fg="#FFFFFF",
                                  bg="#101010", bd=2)
        log_frame.grid(row=6, column=0, columnspan=9, padx=5, pady=4, sticky="nsew")

        self.telemetry_display = st.ScrolledText(
            log_frame, width=100, height=10, font=("Consolas", 10),
            bg="#1a1a1a", fg="#00FF00"
        )
        self.telemetry_display.pack(fill="both", expand=True, padx=5, pady=5)

    # -------------------- Gráficos (4 compactos) --------------------
    def create_graphs(self):
        graphs_frame = tk.Frame(self.root, bg="#101010")
        graphs_frame.grid(row=5, column=0, columnspan=9, padx=5, pady=GRAPHS_PADY, sticky="nsew")

        plt.style.use("dark_background")
        # Figura más chata + layout automático para evitar recortes
        self.fig = Figure(figsize=(12, 3.2), facecolor="#101010", constrained_layout=True)

        gs = self.fig.add_gridspec(2, 2)
        self.ax_throttle = self.fig.add_subplot(gs[0, 0])
        self.ax_brake    = self.fig.add_subplot(gs[0, 1])
        self.ax_vdc      = self.fig.add_subplot(gs[1, 0])
        self.ax_dsavg    = self.fig.add_subplot(gs[1, 1])

        # Config común
        for ax in (self.ax_throttle, self.ax_brake, self.ax_vdc, self.ax_dsavg):
            ax.set_facecolor("#1a1a1a")
            ax.grid(True, alpha=0.3)

        # Límites/títulos compactos
        self.ax_throttle.set_title("ACELERADOR (%)", color="white", fontsize=10, fontweight="bold")
        self.ax_throttle.set_ylim(0, 100)

        self.ax_brake.set_title("FRENO (%)", color="white", fontsize=10, fontweight="bold")
        self.ax_brake.set_ylim(0, 100)

        self.ax_vdc.set_title("DC BUS (V)", color="white", fontsize=10, fontweight="bold")
        # Limite inicial razonable (auto-ajuste si se sale)
        self.ax_vdc.set_ylim(0, 420)

        self.ax_dsavg.set_title("DS TEMP AVG (°C)", color="white", fontsize=10, fontweight="bold")
        self.ax_dsavg.set_ylim(0, 90)

        self.canvas = FigureCanvasTkAgg(self.fig, master=graphs_frame)
        self.canvas.draw()
        widget = self.canvas.get_tk_widget()
        widget.pack(fill="both", expand=True)

    # -------------------- Statusbar --------------------
    def create_statusbar(self):
        sb = tk.Frame(self.root, bg="#151515")
        sb.grid(row=7, column=0, columnspan=9, sticky="ew", pady=STATUS_PADY)
        for i in range(9):
            sb.grid_columnconfigure(i, weight=1)

        self.status_label = tk.Label(
            sb, textvariable=self.status_var, anchor="w",
            font=("Inter", 11), fg="#DDDDDD", bg="#151515", padx=8, pady=4
        )
        self.status_label.grid(row=0, column=0, columnspan=9, sticky="ew")

    # -------------------- Logging bridge --------------------
    def setup_logging_bridge(self):
        self.tk_log_handler = TkTextHandler(self.telemetry_display)
        formatter = logging.Formatter("%(levelname)s - %(name)s - %(message)s")
        self.tk_log_handler.setFormatter(formatter)

        self.backend_logger = logging.getLogger("ISC_RTT_USB")
        self.backend_logger.addHandler(self.tk_log_handler)
        self._apply_debug_level()

        self._flush_early_logs()

    def _apply_debug_level(self):
        self.backend_logger.setLevel(logging.DEBUG if self.debug_var.get() else logging.INFO)

    # -------------------- Acciones de ventana --------------------
    def minimize_window(self):
        if self.root.attributes("-fullscreen"):
            self.root.attributes("-fullscreen", False)
        self.root.iconify()

    def close_window(self):
        self.exit_program()

    def close_fullscreen(self, event=None):
        self.root.attributes("-fullscreen", False)

    def toggle_fullscreen(self, event=None):
        self.root.attributes("-fullscreen", not self.root.attributes("-fullscreen"))

    # -------------------- Puerto serie helpers --------------------
    def refresh_ports(self):
        ports = RTTT.list_serial_ports()
        self.port_combo["values"] = [dev for dev, _ in ports]
        if not self.selected_port.get():
            autodet = self._auto_pick_port_from_list(ports)
            if autodet:
                self.selected_port.set(autodet)
        self.status_var.set(f"Puertos detectados: {', '.join([p[0] for p in ports]) or 'ninguno'}")

    def _auto_pick_port_from_list(self, ports):
        for dev, desc in ports:
            d = (desc or "").upper()
            if "CH340" in d or "USB-SERIAL" in d:
                return dev
        return ports[0][0] if ports else ""

    # -------------------- Iniciar / Parar --------------------
    def start_receiving(self):
        if self.receiving_flag:
            messagebox.showwarning("Aviso", "La recepción ya está en marcha")
            return

        port = self.selected_port.get().strip()
        if not port:
            messagebox.showwarning("Puerto", "Selecciona un puerto COM antes de iniciar.")
            return

        try:
            baud = int(self.selected_baud.get())
        except ValueError:
            messagebox.showerror("Baud", "Baud inválido.")
            return

        use_influx = bool(self.use_influx_var.get())
        debug_mode = bool(self.debug_var.get())

        try:
            # Reset flags
            self.stop_data = False
            RTTT.new_data_flag = 0
            self.receiving_flag = True

            piloto = self.piloto_var.get()
            circuito = self.circuito_var.get()

            bucket_id = RTTT.create_bucket(piloto, circuito, use_influx=use_influx)

            # Thread RX
            self.receiving_thread = threading.Thread(
                target=RTTT.receive_data,
                args=(bucket_id, piloto, circuito, port, baud, use_influx, debug_mode),
                daemon=True
            )
            self.receiving_thread.start()

            # Thread UI updates
            self.ui_update_thread = threading.Thread(target=self.update_ui_thread, daemon=True)
            self.ui_update_thread.start()

            # Botones
            self.run_button.config(state="disabled", bg="#404040")
            self.stop_button.config(state="normal", bg="#CC0000")

            mode = "con Influx" if use_influx else "sin Influx"
            dbg = "DEBUG ON" if debug_mode else "DEBUG OFF"
            msg = f"Iniciando telemetría ({mode}, {dbg}): {piloto} en {circuito} | {port} @ {baud}"
            self.log_message(msg)
            self.status_var.set(msg)

        except Exception as e:
            messagebox.showerror("Error", f"Error iniciando recepción: {e}")
            self.receiving_flag = False

    def stop_receiving(self):
        if not self.receiving_flag:
            return

        try:
            self.stop_data = True
            RTTT.new_data_flag = -1
            self.receiving_flag = False

            if self.receiving_thread and self.ui_update_thread:
                if self.receiving_thread.is_alive():
                    self.receiving_thread.join(timeout=2.0)
                if self.ui_update_thread.is_alive():
                    self.ui_update_thread.join(timeout=1.0)

            self.run_button.config(state="normal", bg="#006400")
            self.stop_button.config(state="disabled", bg="#404040")

            self.log_message("Recepción de telemetría detenida")
            self.status_var.set("Detenido.")
        except Exception as e:
            messagebox.showerror("Error", f"Error deteniendo recepción: {e}")

    # -------------------- Loop de actualización UI --------------------
    def update_ui_thread(self):
        while not self.stop_data:
            try:
                if RTTT.new_data_flag == 1:
                    latest_data = RTTT.get_latest_data()
                    self.root.after(0, self.update_badge_and_freeze, latest_data.get("__STATUS__", {}))
                    if latest_data:
                        self.root.after(0, self.update_data_displays, latest_data)
                    self.root.after(0, self.log_message, RTTT.data_str)
                    RTTT.new_data_flag = 0
                time.sleep(0.01)
            except Exception as e:
                self.root.after(0, self.log_message, f"Error actualizando UI: {e}")
                break

    # -------------------- Badge + congelación --------------------
    def update_badge_and_freeze(self, status_obj: dict):
        badge = str(status_obj.get("badge", "STALE")).upper()
        reason = str(status_obj.get("reason", ""))
        self.link_badge.set(badge)
        self.link_reason.set(reason)

        color_map = {
            "LIVE": "#00FF00",
            "STALE": "#BFBF00",
            "BAD": "#FF3333",
        }
        bg = color_map.get(badge, "#808080")
        self.badge_label.config(bg=bg, fg="#000000")

        self.freeze_ui = (badge in {"STALE", "BAD"})
        self._set_widgets_dim(self.freeze_ui)

    def _set_widgets_dim(self, dim: bool):
        fg_dim = "#888888"
        fg_norm = "#FFFFFF"

        labels = [
            self.accu_voltage_label, self.accu_current_label, self.accu_power_label,
            self.temp_accu_label, self.temp_motor_label, self.temp_inverter_label, self.temp_air_label,
            self.inverter_status_label, self.inverter_errors_label, self.n_i_label,
            self.torque_req_label, self.torque_est_label,
            self.accel_raw1_label, self.accel_raw2_label,
            self.accel_scaled_label, self.accel_clamped_label,
        ]
        for lb in labels:
            try:
                lb.config(fg=fg_dim if dim else fg_norm)
            except Exception:
                pass

        tcolor = fg_dim if dim else "#FFFFFF"
        try:
            for ax in (self.ax_throttle, self.ax_brake, self.ax_vdc, self.ax_dsavg):
                ax.title.set_color(tcolor)
            self.canvas.draw()
        except Exception:
            pass

    # -------------------- Render de datos + gráficas --------------------
    def update_data_displays(self, data: dict):
        if self.freeze_ui:
            return
        try:
            # Track values for graphs (may be None)
            g_throttle = None
            g_brake = None
            g_vdc = None
            g_dsavg = None

            # 0x640 — Accumulator summary
            if "0x640" in data:
                accu = data["0x640"]
                if "current_sensor" in accu:
                    self.accu_current_label.config(text=f"Corriente: {accu['current_sensor']:.1f} A")
                if "cell_min_v" in accu:
                    # Si tu 0x640 v2 era otra cosa, mantenemos DC Bus desde 0x600 (abajo)
                    self.accu_voltage_label.config(text=f"Voltaje Min: {accu['cell_min_v']:.2f} V")
                if "cell_max_temp" in accu:
                    temp = float(accu["cell_max_temp"])
                    color = "#FF0000" if temp > 50 else "#FFA500" if temp > 40 else "#FFFFFF"
                    self.temp_accu_label.config(text=f"Accu Max: {temp:.1f} °C", fg=color if not self.freeze_ui else "#888888")

            # 0x610 — Inverter temps & currents
            if "0x610" in data:
                inv = data["0x610"]
                if "motor_temp" in inv:
                    self.temp_motor_label.config(text=f"Motor: {inv['motor_temp']:.1f} °C")
                if "pwrstg_temp" in inv:
                    self.temp_inverter_label.config(text=f"Inversor: {inv['pwrstg_temp']:.1f} °C")
                if "air_temp" in inv:
                    self.temp_air_label.config(text=f"Aire: {inv['air_temp']:.1f} °C")
                # n_actual / i_actual
                ntext = f"n_actual: {inv.get('n_actual','--'):.0f} rpm" if isinstance(inv.get('n_actual'), (int,float)) else "n_actual: -- rpm"
                itext = f"i_actual: {inv.get('i_actual','--'):.1f} A" if isinstance(inv.get('i_actual'), (int,float)) else "i_actual: -- A"
                self.n_i_label.config(text=f"{ntext} | {itext}")

            # 0x680 — Inverter status/errors
            if "0x680" in data:
                invs = data["0x680"]
                if "status" in invs:
                    status = int(invs["status"])
                    status_text = "CONECTADO" if status == 1 else "DESCONECTADO"
                    status_color = "#00FF00" if status == 1 else "#FF0000"
                    self.inverter_status_label.config(text=f"Estado: {status_text}",
                                                      fg=status_color if not self.freeze_ui else "#888888")
                if "errors" in invs:
                    errors = int(invs["errors"])
                    error_color = "#FF0000" if errors > 0 else "#FFFFFF"
                    self.inverter_errors_label.config(text=f"Errores: {errors}",
                                                      fg=error_color if not self.freeze_ui else "#888888")

            # 0x630 — Driver inputs summary
            if "0x630" in data:
                drv = data["0x630"]
                if "torque_req" in drv:
                    self.torque_req_label.config(text=f"Solicitado: {drv['torque_req']:.1f} Nm")
                if "torque_est" in drv:
                    self.torque_est_label.config(text=f"Estimado: {drv['torque_est']:.1f} Nm")
                if "throttle" in drv:
                    g_throttle = float(drv["throttle"])
                    self.accel_scaled_label.config(text=f"Escalado: {g_throttle:.2f} %")
                    self.accel_clamped_label.config(text=f"Clamped:  {max(0.0, min(100.0, g_throttle)):.2f} %")
                if "brake" in drv:
                    g_brake = float(drv["brake"])

            # 0x600 — raw throttle + DC Bus
            if "0x600" in data:
                m600 = data["0x600"]
                if "throttle_raw1" in m600:
                    self.accel_raw1_label.config(text=f"Raw1: {m600['throttle_raw1']:.2f}")
                if "throttle_raw2" in m600:
                    self.accel_raw2_label.config(text=f"Raw2: {m600['throttle_raw2']:.2f}")
                if "dc_bus_power" in m600:
                    self.accu_power_label.config(text=f"Potencia: {m600['dc_bus_power']:.1f} W")
                if "dc_bus_voltage" in m600:
                    vdc = float(m600["dc_bus_voltage"])
                    self.accu_voltage_label.config(text=f"DC Bus: {vdc:.1f} V")
                    g_vdc = vdc

            # 0x645 — detalle DS18B20 (avg preferente)
            if "0x645" in data:
                ds = data["0x645"]
                temps = [ds.get("ds_t1"), ds.get("ds_t2"), ds.get("ds_t3"), ds.get("ds_t4")]
                for i, t in enumerate(temps):
                    if isinstance(t, (int, float)):
                        self.ds_labels[i].config(text=f"DS{i+1}: {t:.1f} °C")
                dsavg = ds.get("ds_avg")
                if isinstance(dsavg, (int, float)):
                    self.ds_summary_label.config(text=f"DS avg: {dsavg:.1f} °C")
                    g_dsavg = float(dsavg)
                else:
                    # calcular media de las disponibles si no viene ds_avg
                    vals = [float(t) for t in temps if isinstance(t, (int, float))]
                    if vals:
                        g_dsavg = sum(vals) / len(vals)
                        self.ds_summary_label.config(text=f"DS avg: {g_dsavg:.1f} °C")

            # Actualiza las 4 gráficas compactas
            self.update_graphs(g_throttle, g_brake, g_vdc, g_dsavg)

        except Exception as e:
            self.log_message(f"Error actualizando displays: {e}")

    # -------------------- Gráficas 2×2 compactas --------------------
    def update_graphs(self, throttle=None, brake=None, vdc=None, dsavg=None):
        if self.freeze_ui:
            return
        try:
            t_now = time.time()
            self.time_history.append(t_now)

            # Acumula sólo si hay dato nuevo
            if throttle is not None:
                self.throttle_history.append(float(throttle))
            else:
                self.throttle_history.append(self.throttle_history[-1] if self.throttle_history else 0.0)

            if brake is not None:
                self.brake_history.append(float(brake))
            else:
                self.brake_history.append(self.brake_history[-1] if self.brake_history else 0.0)

            if vdc is not None:
                self.vdc_history.append(float(vdc))
            else:
                self.vdc_history.append(self.vdc_history[-1] if self.vdc_history else 0.0)

            if dsavg is not None:
                self.dsavg_history.append(float(dsavg))
            else:
                self.dsavg_history.append(self.dsavg_history[-1] if self.dsavg_history else 0.0)

            # Eje tiempo relativo
            if len(self.time_history) > 1:
                t0 = self.time_history[0]
                ts = np.array(self.time_history) - t0
            else:
                ts = np.array([0.0]*len(self.time_history))

            # Clear + re-draw (líneas finas para compacto)
            for ax in (self.ax_throttle, self.ax_brake, self.ax_vdc, self.ax_dsavg):
                ax.cla()
                ax.set_facecolor("#1a1a1a")
                ax.grid(True, alpha=0.3)

            # Titles + limits
            self.ax_throttle.set_title("ACELERADOR (%)", color="white", fontsize=10, fontweight="bold")
            self.ax_throttle.set_ylim(0, 100)
            self.ax_brake.set_title("FRENO (%)", color="white", fontsize=10, fontweight="bold")
            self.ax_brake.set_ylim(0, 100)
            self.ax_vdc.set_title("DC BUS (V)", color="white", fontsize=10, fontweight="bold")
            # auto-ajuste si VDC se sale
            vmax = max(self.vdc_history) if self.vdc_history else 100
            self.ax_vdc.set_ylim(0, max(60, ((int(vmax/20)+1)*20)))  # múltiplos de 20
            self.ax_dsavg.set_title("DS TEMP AVG (°C)", color="white", fontsize=10, fontweight="bold")
            self.ax_dsavg.set_ylim(0, 90)

            # Draw series (sin relleno para mantenerlas pequeñas)
            self.ax_throttle.plot(ts, list(self.throttle_history), linewidth=1.5)
            self.ax_brake.plot(ts, list(self.brake_history), linewidth=1.5)
            self.ax_vdc.plot(ts, list(self.vdc_history), linewidth=1.5)
            self.ax_dsavg.plot(ts, list(self.dsavg_history), linewidth=1.5)

            self.canvas.draw()
        except Exception as e:
            self.log_message(f"Error actualizando gráficos: {e}")

    # -------------------- Utilities --------------------
    def open_logs_folder(self):
        path = os.path.abspath("logs")
        os.makedirs(path, exist_ok=True)
        try:
            if sys.platform.startswith("win"):
                os.startfile(path)  # type: ignore
            elif sys.platform == "darwin":
                subprocess.Popen(["open", path])
            else:
                subprocess.Popen(["xdg-open", path])
        except Exception as e:
            messagebox.showerror("Abrir carpeta", f"No se pudo abrir la carpeta de logs:\n{e}")

    # -------------------- Log y salida --------------------
    def log_message(self, message: str):
        try:
            ts = time.strftime("%H:%M:%S")
            self.telemetry_display.insert(tk.END, f"[{ts}] {message}\n")
            self.telemetry_display.see(tk.END)
            # Limitar a 600 líneas
            lines = self.telemetry_display.get("1.0", tk.END).split("\n")
            if len(lines) > 600:
                self.telemetry_display.delete("1.0", f"{len(lines)-600}.0")
        except Exception as e:
            print(f"Error añadiendo mensaje al log: {e}")

    def exit_program(self):
        try:
            if self.receiving_flag:
                self.stop_receiving()
            self.root.quit()
            self.root.destroy()
            sys.exit(0)
        except Exception as e:
            print(f"Error cerrando programa: {e}")
            sys.exit(1)

    def run(self):
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            self.exit_program()


# -------------------- Main --------------------
if __name__ == "__main__":
    print("=== Iniciando ISCmetrics UI ===")
    try:
        app = TelemetryUI()
        app.run()
    except Exception as e:
        print(f"Error fatal en la aplicación: {e}")
        sys.exit(1)
