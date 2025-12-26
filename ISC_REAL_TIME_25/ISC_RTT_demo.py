"""
ISC_RTT_demo.py
Advanced Physics Simulator for ISC Formula Student Telemetry.
UPDATES:
- Dynamic Brake Temperatures (Heating vs Cooling Airflow).
- Battery Voltage: 400V -> 360V range.
- CSV Logging & Marple Upload integrated.
"""

import time
import threading
import random
import math
import csv
import os
from datetime import datetime
from pathlib import Path
from typing import Dict, Any
import isc_marple 

# --- CONFIGURACIÓN ---
LOG_DIR = Path("logs")
LOG_DIR.mkdir(exist_ok=True)

class DemoCSVLogger:
    def __init__(self, piloto, circuito):
        self.timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = LOG_DIR / f"ISC_DEMO_{self.timestamp_str}_{piloto}_{circuito}.csv"
        self.piloto = piloto
        self.circuito = circuito
        
        self.file = open(self.filename, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.record_count = 0
        
        # CABECERAS (Flat format for Marple)
        self.headers = [
            "time", "time_elapsed_s",
            # 0x600 - Main
            "dc_bus_voltage", "rpm", "torque_total", "cell_min_v", "throttle_raw1", "throttle_raw2",
            # 0x610 - Motor
            "motor_temp", "pwrstg_temp", "air_temp", "n_actual", "i_actual",
            # 0x620 - Driver Raw
            "s1_raw", "s2_raw", "brake_raw", "precharge_btn", "start_btn",
            # 0x630 - Driver Proc
            "torque_req", "torque_est", "throttle_pct", "brake_pct",
            # AMS
            "ams_min_cell_mv", "ams_max_cell_mv", "ams_stack_v", "ams_current_a", 
            "ams_max_temp_c", "ams_min_temp_c", "ams_avg_temp_c",
            # Dynamics
            "g_long", "g_lat", "g_total",
            "susp_force_fl", "susp_force_fr", "susp_force_rl", "susp_force_rr",
            "susp_travel_fl", "susp_travel_fr", "susp_travel_rl", "susp_travel_rr",
            "brake_temp_fl", "brake_temp_fr", "brake_temp_rl", "brake_temp_rr"
        ]
        self.writer.writerow(self.headers)
        print(f"[DEMO] CSV creado: {self.filename}")

    def log(self, data: Dict[int, Any], elapsed_time: float):
        ts_now = datetime.now().isoformat()
        
        d600 = data.get(0x600, {})
        d610 = data.get(0x610, {})
        d620 = data.get(0x620, {})
        d630 = data.get(0x630, {})
        d202 = data.get(0x202, {}) 
        d208 = data.get(0x208, {}) 
        d201 = data.get(0x201, {}) 
        d650 = data.get(0x650, {}) 
        d660 = data.get(0x660, {}) 
        d670 = data.get(0x670, {}) 
        
        s_forces = d660.get('susp_forces', [0]*4)
        s_travel = d660.get('susp_travel', [0]*4)
        
        row = [
            ts_now, f"{elapsed_time:.3f}",
            d600.get('dcbusvoltage', 0), d600.get('rpm', 0), d600.get('torquetotal', 0),
            d600.get('cellminv', 0), d600.get('throttleraw1', 0), d600.get('throttleraw2', 0),
            
            d610.get('motortemp', 0), d610.get('pwrstgtemp', 0), d610.get('airtemp', 0),
            d610.get('nactual', 0), d610.get('iactual', 0),
            
            d620.get('s1raw', 0), d620.get('s2raw', 0), d620.get('brakeraw', 0),
            d620.get('prechargebutton', 0), d620.get('startbutton', 0),
            
            d630.get('torquereq', 0), d630.get('torqueest', 0), d630.get('throttle', 0), d630.get('brake', 0),
            
            d202.get('mincellmv', 0), d202.get('maxcellmv', 0), d202.get('stacktotalmv', 0)/1000.0,
            d201.get('currentdA', 0)/10.0, d208.get('maxtempc', 0), d208.get('mintempc', 0), d208.get('avgtempc', 0),
            
            d650.get('g_long', 0), d650.get('g_lat', 0), d650.get('g_total', 0),
            s_forces[0], s_forces[1], s_forces[2], s_forces[3],
            s_travel[0], s_travel[1], s_travel[2], s_travel[3],
            d670.get('brake_temp_fl', 0), d670.get('brake_temp_fr', 0),
            d670.get('brake_temp_rl', 0), d670.get('brake_temp_rr', 0)
        ]
        
        self.writer.writerow(row)
        self.record_count += 1
        if self.record_count % 50 == 0: self.file.flush()

    def close(self):
        if self.file:
            self.file.close()
            print(f"[DEMO] CSV Cerrado. Filas: {self.record_count}")
            return str(self.filename)
        return None

class DemoDataGenerator:
    def __init__(self):
        self.running = False
        self.thread = None
        self.lock = threading.Lock()
        self.logger = None
        self.upload_to_marple = False
        
        # --- CAR PHYSICS CONSTANTS ---
        self.MASS = 280.0       
        self.MAX_TORQUE = 230.0 
        self.MAX_POWER = 80000  
        self.MAX_SPEED = 150.0 / 3.6 
        self.DRAG_COEFF = 0.9   
        self.WHEEL_RADIUS = 0.25 
        self.GEAR_RATIO = 3.5
        
        # --- BATTERY MODEL ---
        self.BATTERY_MAX_V = 400.0
        self.BATTERY_MIN_V = 360.0 
        self.internal_resistance = 0.12 
        
        # --- STATE VARIABLES ---
        self.time_elapsed = 0.0
        self.dt = 0.05 
        
        # Dynamic State
        self.speed_ms = 0.0
        self.dist_m = 0.0
        self.throttle_cmd = 0.0 
        self.brake_cmd = 0.0    
        
        # Electrical State
        self.voltage_open_circuit = 400.0 
        self.voltage_load = 400.0
        self.current = 0.0
        
        # Thermal State
        self.pack_temp = 25.0
        self.inverter_temp = 30.0
        self.motor_temp = 35.0
        self.ambient_temp = 25.0
        
        # Dynamics
        self.g_lat = 0.0
        self.g_long = 0.0
        self.susp_travel = [40.0]*4
        self.brake_temps = [25.0]*4 # Initialize at ambient
        
        self.data = {}

        # --- TRACK PROFILE ---
        self.track_segments = [
            (300, 150, 0),   # Recta principal
            (60,  60,  0),   # Frenada
            (90,  60,  25),  # Curva cerrada
            (150, 110, 0),   # Recta corta
            (140, 90,  40),  # Curva larga
            (50,  120, 0),   # Salida
            (70,  50,  15),  # Horquilla
            (250, 145, 0),   # Recta trasera
        ]
        self.total_track_len = sum(s[0] for s in self.track_segments)

    def start(self, use_marple=False, piloto="Demo", circuito="Track"):
        if self.running: return
        self.running = True
        self.upload_to_marple = use_marple
        
        # Reset State
        self.time_elapsed = 0.0
        self.dist_m = 0.0
        self.voltage_open_circuit = 400.0
        self.pack_temp = 25.0
        self.speed_ms = 0.0
        self.brake_temps = [25.0] * 4 # RESET BRAKES TO COOL
        
        self.logger = DemoCSVLogger(piloto, circuito)
        self.thread = threading.Thread(target=self._physics_loop, daemon=True)
        self.thread.start()
        print(f"[DEMO] Physics Engine Started. V_batt: 400->360V. Active Brake Thermal Model.")

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        
        if self.logger:
            file_path = self.logger.close()
            if self.upload_to_marple and file_path:
                print("[DEMO] Iniciando subida a Marple...")
                meta = {"piloto": self.logger.piloto, "circuito": self.logger.circuito, "type": "DemoSim"}
                isc_marple.upload_session_csv(file_path, meta)
            self.logger = None

    def _get_current_segment(self, distance):
        d_accum = 0
        dist_in_lap = distance % self.total_track_len
        for length, target_kph, radius in self.track_segments:
            if dist_in_lap < d_accum + length:
                segment_progress = (dist_in_lap - d_accum) / length
                return target_kph, radius, segment_progress, length
            d_accum += length
        return 0, 0, 0, 0

    def _physics_loop(self):
        while self.running:
            start_t = time.time()
            
            # 1. Driver Logic
            target_kph, radius, progress, seg_len = self._get_current_segment(self.dist_m)
            target_ms = target_kph / 3.6
            dist_to_end = seg_len * (1.0 - progress)
            speed_error = target_ms - self.speed_ms
            
            brake_zone = False
            if radius == 0 and dist_to_end < 80 and self.speed_ms > 25: 
                brake_zone = True

            if brake_zone:
                self.throttle_cmd = 0.0
                self.brake_cmd = min(100.0, self.brake_cmd + 20) # Frenada más agresiva
            elif speed_error > 2.0:
                self.throttle_cmd = min(100.0, speed_error * 15)
                self.brake_cmd = 0.0
            elif speed_error < -2.0:
                self.throttle_cmd = 0.0
                self.brake_cmd = min(100.0, abs(speed_error) * 8)
            else:
                self.throttle_cmd = 25.0 
                self.brake_cmd = 0.0

            # 2. Dynamics
            traction_force = (self.throttle_cmd / 100.0) * self.MAX_TORQUE * self.GEAR_RATIO / self.WHEEL_RADIUS
            if self.speed_ms > self.MAX_SPEED: traction_force = 0
            braking_force = (self.brake_cmd / 100.0) * 4500.0 
            drag_force = 0.5 * 1.225 * self.DRAG_COEFF * 1.6 * (self.speed_ms ** 2)
            
            net_force = traction_force - braking_force - drag_force
            accel = net_force / self.MASS
            
            self.speed_ms += accel * self.dt
            if self.speed_ms < 0: self.speed_ms = 0
            self.dist_m += self.speed_ms * self.dt
            
            self.g_long = accel / 9.81
            if radius > 0 and self.speed_ms > 1:
                lat_accel = (self.speed_ms ** 2) / radius
                direction = 1 if int(self.dist_m / 100) % 2 == 0 else -1
                self.g_lat = (lat_accel / 9.81) * direction
                self.g_lat = max(-2.8, min(2.8, self.g_lat))
            else:
                self.g_lat = 0.0
            g_total = math.sqrt(self.g_lat**2 + self.g_long**2)

            # 3. Electrical Model
            mech_power = max(0, traction_force * self.speed_ms)
            elec_power = mech_power / 0.90
            
            if self.voltage_load > 0:
                self.current = elec_power / self.voltage_load
            self.current += 4.0 
            
            # Voltaje 400 -> 360 logic
            base_drop = (self.time_elapsed / 2100.0) * 40.0
            usage_factor = (self.current * self.dt) * 0.005 
            self.voltage_open_circuit -= usage_factor * 0.01
            voc = 400.0 - base_drop - (self.voltage_open_circuit - 400.0)
            if voc < 360.0: voc = 360.0
            self.voltage_load = voc - (self.current * self.internal_resistance)

            # 4. Thermal Model - BRAKES & BATTERY
            
            # Battery: I^2 heating vs Air Cooling
            heating_joule = (self.current ** 2) * 0.00015
            cooling_factor = 0.005 + (self.speed_ms * 0.0015) 
            delta_temp = self.pack_temp - self.ambient_temp
            self.pack_temp += (heating_joule - (delta_temp * cooling_factor)) * self.dt
            if self.pack_temp < self.ambient_temp: self.pack_temp = self.ambient_temp

            # BRAKE THERMAL MODEL (Dynamic)
            # Energy input = Brake Force * Velocity (Power dissipated)
            # Cooling = Convection coefficient (Function of velocity)
            
            brake_power_kw = (braking_force * self.speed_ms) / 1000.0 
            
            # Cooling coeff grows significantly with speed (Airflow over rotors)
            # Base cooling (radiation) + Airflow cooling
            brake_cooling_coeff = 0.08 + (self.speed_ms * 0.015) 
            
            for i in range(4):
                # 4 brakes share the load approx equally
                # Heating factor: 0.15 deg per kW per timestep (tuned for demo visual)
                heating = (brake_power_kw / 4.0) * 0.15 
                
                delta_t = self.brake_temps[i] - self.ambient_temp
                cooling = delta_t * brake_cooling_coeff * self.dt
                
                self.brake_temps[i] += heating - cooling
                
                # Floor at ambient
                if self.brake_temps[i] < self.ambient_temp: 
                    self.brake_temps[i] = self.ambient_temp

            # Update Dictionaries
            self._update_data(g_total)
            
            with self.lock:
                if self.logger: self.logger.log(self.data, self.time_elapsed)
            
            self.time_elapsed += self.dt
            time.sleep(max(0, self.dt - (time.time() - start_t)))

    def _update_data(self, g_total):
        rpm = (self.speed_ms / (2 * math.pi * self.WHEEL_RADIUS)) * 60 * self.GEAR_RATIO
        min_cell = (self.voltage_load / 95.0) - 0.005 + random.uniform(0, 0.002)
        max_cell = (self.voltage_load / 95.0) + 0.005 + random.uniform(0, 0.002)
        
        self.data[0x600] = {
            'dcbusvoltage': self.voltage_load,
            'rpm': rpm,
            'torquetotal': (self.throttle_cmd/100)*self.MAX_TORQUE,
            'cellminv': min_cell * 1000,
            'throttleraw1': 1500 + (self.throttle_cmd*5),
            'throttleraw2': 1500 + (self.throttle_cmd*5)
        }
        self.data[0x610] = {
            'motortemp': self.motor_temp + (self.current*0.05),
            'pwrstgtemp': self.inverter_temp + (self.current*0.04),
            'airtemp': 25.0,
            'nactual': rpm,
            'iactual': self.current
        }
        self.data[0x620] = {
            's1raw': 1500 + (self.throttle_cmd*5),
            's2raw': 1500 + (self.throttle_cmd*5),
            'brakeraw': 800 + (self.brake_cmd*5),
            'prechargebutton': 1, 'startbutton': 1
        }
        self.data[0x630] = {
            'torquereq': (self.throttle_cmd/100)*self.MAX_TORQUE,
            'torqueest': (self.throttle_cmd/100)*self.MAX_TORQUE * 0.9,
            'throttle': self.throttle_cmd,
            'brake': self.brake_cmd
        }
        self.data[0x202] = {'mincellmv': min_cell*1000, 'maxcellmv': max_cell*1000, 'stacktotalmv': self.voltage_load*1000}
        self.data[0x201] = {'currentdA': self.current * 10}
        self.data[0x208] = {'maxtempc': self.pack_temp, 'mintempc': self.pack_temp-2, 'avgtempc': self.pack_temp-1}
        
        self.data[0x650] = {'g_long': self.g_long, 'g_lat': self.g_lat, 'g_total': g_total}
        
        travel = [
            40 + (self.g_long * 8) + (self.g_lat * 8) + random.uniform(-0.5,0.5), 
            40 + (self.g_long * 8) - (self.g_lat * 8) + random.uniform(-0.5,0.5), 
            40 - (self.g_long * 8) + (self.g_lat * 8) + random.uniform(-0.5,0.5), 
            40 - (self.g_long * 8) - (self.g_lat * 8) + random.uniform(-0.5,0.5)  
        ]
        forces = [t * 25 for t in travel]
        
        self.data[0x660] = {'susp_forces': forces, 'susp_travel': travel}
        
        # Mapeo correcto de temperaturas de frenos
        self.data[0x670] = {
            'brake_temp_fl': self.brake_temps[0], 'brake_temp_fr': self.brake_temps[1],
            'brake_temp_rl': self.brake_temps[2], 'brake_temp_rr': self.brake_temps[3]
        }
        
        self.data[0x700] = self.data[0x650]
        self.data[0x710] = self.data[0x660]
        self.data[0x720] = self.data[0x670]

    def get_latest_data(self) -> Dict[str, Any]:
        with self.lock: return self.data.copy()

    def get_ams_module_data(self, module_id: int):
        with self.lock:
            base_v = (self.voltage_load / 95.0) * 1000
            return {
                'voltages': [base_v + random.uniform(-10,10) for _ in range(19)],
                'tempsc': [self.pack_temp + random.uniform(-0.5,0.5) for _ in range(38)]
            }

_demo_generator = None

def start_demo(use_marple=False, piloto="Demo", circuito="Track"):
    global _demo_generator
    if _demo_generator is None: _demo_generator = DemoDataGenerator()
    _demo_generator.start(use_marple, piloto, circuito)

def stop_demo():
    global _demo_generator
    if _demo_generator: _demo_generator.stop()

def get_latest_data(): return _demo_generator.get_latest_data() if _demo_generator else {}
def get_ams_module_data(mid): return _demo_generator.get_ams_module_data(mid) if _demo_generator else {}
def is_demo_running(): return _demo_generator.running if _demo_generator else False