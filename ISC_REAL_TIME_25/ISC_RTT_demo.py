"""
ISC_RTT_demo.py
Demo data generator for ISC Formula Student Telemetry System
Simulates realistic CAN bus data for AMS, Inverter, Motor, Driver, and Dynamics/IMU/Suspension
Author: Andrés Sánchez de Ojeda 2025-2026
"""

import time
import threading
import random
import math
from typing import Dict, Any

class DemoDataGenerator:
    def __init__(self):
        self.running = False
        self.thread = None
        self.lock = threading.Lock()
        # Car specs and simulation constants
        self.MAX_POWER_KW = 85.0
        self.MAX_TORQUE_NM = 230.0
        self.MAX_RPM = 12000.0
        self.MAX_SPEED_KPH = 170.0
        self.NUM_CELLS = 95
        self.CELL_MAX_V = 4.2
        self.CELL_MIN_V = 3.6
        self.BATTERY_MAX_V = self.NUM_CELLS * self.CELL_MAX_V  # 399V
        self.BATTERY_MIN_V = 360.0
        self.BATTERY_CAPACITY_AH = 6.0
        self.BATTERY_ENERGY_WH = self.BATTERY_MAX_V * self.BATTERY_CAPACITY_AH  # ~2.4kWh
        self.VEHICLE_MASS_KG = 250.0

        # Physics state
        self.time_elapsed = 0.0
        self.session_start_time = time.time()
        self.lap_time = 90.0  # s
        self.lap_progress = 0.0
        self.total_energy_used_wh = 0.0
        self.state_of_charge = 100.0

        # Telemetry
        self.speed_kmh = 0.0
        self.rpm = 0.0
        self.throttle = 0.0
        self.brake = 0.0
        self.torque = 0.0
        self.current = 0.0

        self.dc_bus_voltage = self.BATTERY_MAX_V
        self.dc_bus_power = 0.0

        # AMS values
        self.ams_current_dA = 0.0
        self.ams_global_max_mv = self.CELL_MAX_V * 1000
        self.ams_global_min_mv = self.CELL_MAX_V * 1000
        self.ams_stack_total_mv = self.BATTERY_MAX_V * 1000
        self.ams_max_temp_c = 25.0
        self.ams_min_temp_c = 25.0
        self.ams_avg_temp_c = 25.0

        # Module/cell/thermal simulation
        self.modules = []
        for mod_id in range(5):
            module = {
                'id': mod_id,
                'cell_voltages': [self.CELL_MAX_V * 1000 + random.uniform(-10, 10) for _ in range(19)],
                'temperatures': [25.0 + random.uniform(-1, 1) for _ in range(38)],
                'base_temp': 25.0
            }
            self.modules.append(module)

        # Motor/Inverter/etc
        self.motor_temp = 40.0
        self.igbt_temp = 35.0
        self.air_temp = 20.0

        self.throttle_raw1 = 1500
        self.throttle_raw2 = 1520
        self.brake_raw = 800
        self.precharge_button = 0
        self.start_button = 0
        self.suspension = [50.0 for _ in range(4)]

        # DYNAMICS: IMU, Suspension, Brake temp
        self.imu_g_lat = 0.0
        self.imu_g_long = 0.0
        self.imu_g_total = 0.0
        self.susp_force = [0.0] * 4
        self.susp_travel = [40.0] * 4
        self.brake_temps = [40.0] * 4

        self.system_status = 2
        self.error_code = 0
        self.data = {}

        # Barcelona Catalunya "circuit profile"
        self.circuit_profile = [
            (0.00, 120,  0.10),   # (progress, target speed kph, turn radius G)
            (0.10, 70,   0.90),   # Heavy right
            (0.13, 90,   0.40),   # Exit right
            (0.23, 132,  0.25),   # Fast left
            (0.36, 60,   0.70),   # Heavy left
            (0.44, 85,   0.25),   # Exit
            (0.53, 120,  0.15),   # Back straight
            (0.65, 65,   0.85),   # Heavy right
            (0.70, 95,   0.27),   # Exit
            (0.77, 130,  0.15),   # Very fast
            (0.87, 80,   0.65),   # Chicane, double turn
            (0.95, 120,  0.40),   # Final turn exit
            (1.00, 135,  0.10)
        ]

    def start(self):
        if self.running:
            return
        self.running = True
        self.session_start_time = time.time()
        self.total_energy_used_wh = 0.0
        self.thread = threading.Thread(target=self._generation_loop, daemon=True)
        self.thread.start()
        print("[DEMO] Started")

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)

    def _generation_loop(self):
        update_rate = 0.05  # 20Hz
        while self.running:
            start_time = time.time()
            self._update_vehicle_dynamics(update_rate)
            with self.lock:
                self._update_can_data()
            time.sleep(max(0, update_rate - (time.time() - start_time)))
            self.time_elapsed += update_rate

    def _get_profile_section(self, position):
        for i in range(len(self.circuit_profile) - 1):
            if self.circuit_profile[i][0] <= position <= self.circuit_profile[i + 1][0]:
                return self.circuit_profile[i], self.circuit_profile[i + 1]
        return self.circuit_profile[-2], self.circuit_profile[-1]

    def _update_vehicle_dynamics(self, dt):
        self.lap_progress = (self.time_elapsed % self.lap_time) / self.lap_time
        # Interpolate speed and G target from circuit profile
        (p1, s1, g1), (p2, s2, g2) = self._get_profile_section(self.lap_progress)
        pratio = (self.lap_progress - p1) / (p2 - p1) if p2 != p1 else 0
        target_speed = s1 + (s2 - s1) * pratio  # [kph]
        target_g_lat = g1 + (g2 - g1) * pratio  # [fraction, 0=straight, >0 sharp turn]
        cornering = target_g_lat > 0.2

        speed_error = target_speed - self.speed_kmh
        # Throttle/Brake logic
        if speed_error > 5:
            self.throttle = min(100.0, max(30.0, speed_error * 1.5))
            self.brake = 0.0
        elif speed_error < -5:
            self.throttle = 0.0
            self.brake = min(100.0, abs(speed_error) * 2.0)
        else:
            self.throttle = 40.0 + random.uniform(-6, 6)
            self.brake = 0.0
        self.throttle = max(0, min(100, self.throttle))
        self.brake = max(0, min(100, self.brake))

        # Torque curve / power limit (flat until base rpm)
        base_rpm = 6000.0
        wheel_diam = 0.51
        final_drive = 3.5
        if self.rpm < base_rpm:
            avail_torque = self.MAX_TORQUE_NM
        else:
            avail_torque = (self.MAX_POWER_KW * 9549.0) / self.rpm if self.rpm > 0 else self.MAX_TORQUE_NM
        self.torque = (self.throttle / 100.0) * avail_torque

        # Wheel & engine speed
        wheel_speed_rps = (self.speed_kmh / 3.6) / (math.pi * wheel_diam)
        self.rpm = wheel_speed_rps * 60.0 * final_drive
        self.rpm = min(max(self.rpm, 0), self.MAX_RPM)

        # Power and current
        power_kw = (self.torque * self.rpm) / 9549.0
        if self.throttle > 0:
            self.current = (power_kw * 1000.0) / self.dc_bus_voltage if self.dc_bus_voltage > 0 else 0
            self.current = max(0, min(320, self.current))
        else:
            self.current = 8.0 + random.uniform(-2, 2)
        # Energy use for SoC
        self.total_energy_used_wh += (power_kw * dt) / 3600.0
        self.state_of_charge = 100.0 - (self.total_energy_used_wh / self.BATTERY_ENERGY_WH) * 100.0
        self.state_of_charge = max(0, self.state_of_charge)
        # Battery voltage
        base_voltage = self.BATTERY_MIN_V + (self.BATTERY_MAX_V - self.BATTERY_MIN_V) * (self.state_of_charge / 100.0)
        voltage_sag = self.current * 0.08
        self.dc_bus_voltage = base_voltage - voltage_sag + random.uniform(-1, 1)
        self.dc_bus_voltage = max(self.BATTERY_MIN_V, min(self.BATTERY_MAX_V, self.dc_bus_voltage))
        self.dc_bus_power = self.dc_bus_voltage * self.current
        self.ams_current_dA = self.current * 10.0

        # Simple kinematics for speed
        if self.brake > 0:
            decel_mps2 = (self.brake / 100.0) * 14.0  # Max 1.4g
            self.speed_kmh -= (decel_mps2 * 3.6 * dt)
        elif self.throttle > 0:
            wheel_force = (self.torque * final_drive) / (wheel_diam / 2.0)
            accel_mps2 = wheel_force / self.VEHICLE_MASS_KG
            accel_mps2 = min(accel_mps2, 9.0)
            drag_force = 0.5 * 1.2 * 0.5 * 1.5 * ((self.speed_kmh / 3.6) ** 2)
            rolling_resistance = self.VEHICLE_MASS_KG * 9.81 * 0.012
            resistance_decel = (drag_force + rolling_resistance) / self.VEHICLE_MASS_KG
            net_accel = max(0, accel_mps2 - resistance_decel)
            self.speed_kmh += (net_accel * 3.6 * dt)
        else:
            drag_force = 0.5 * 1.2 * 0.5 * 1.5 * ((self.speed_kmh / 3.6) ** 2)
            rolling_resistance = self.VEHICLE_MASS_KG * 9.81 * 0.012
            decel_mps2 = (drag_force + rolling_resistance) / self.VEHICLE_MASS_KG
            self.speed_kmh -= (decel_mps2 * 3.6 * dt)
        self.speed_kmh = min(max(self.speed_kmh, 0), self.MAX_SPEED_KPH)

        # -- DYNAMICS simulation --
        # Estimate G-forces: longitudinal from accel, lat from target_g_lat
        g_base = 9.81
        long_accel = 0.0
        if self.throttle > 0:
            long_accel = (self.torque * final_drive) / (self.VEHICLE_MASS_KG * (wheel_diam/2))
        if self.brake > 0:
            long_accel = -((self.brake / 100.0) * 14.0)
        self.imu_g_long = long_accel / g_base
        # Lateral G from section curvature and current speed
        turn_coeff = min(1.0, abs(target_g_lat))
        lat_g = turn_coeff * ((self.speed_kmh / 90.0) ** 1.15) * 1.1  # Up to 1.2G in hard turns
        if not cornering:
            lat_g = 0.0
        lat_g = lat_g * (-1 if ((int(self.lap_progress*10)%2)==0) else 1)  # alternate left/right
        self.imu_g_lat = lat_g
        self.imu_g_total = math.sqrt(self.imu_g_lat**2 + self.imu_g_long**2)

        # Suspension travel: more in hard braking/turn and uneven track
        base_travel = 40.0
        max_travel = 80.0
        delta = abs(self.imu_g_lat)*35 + abs(self.imu_g_long)*30
        noise = [random.uniform(-2,2) for _ in range(4)]
        self.susp_travel = [
            min(max(base_travel + delta*(1 if i < 2 else -1) + n, 25), max_travel)
            for i, n in enumerate(noise)
        ]
        # Suspension force: proportional to dynamic load shift
        sprung_mass = self.VEHICLE_MASS_KG / 4.0
        load_shift = 40 * self.imu_g_long
        self.susp_force = [
            sprung_mass*g_base + load_shift + random.uniform(-8,8)
            for _ in range(4)
        ]

        # Brake disk temperature elevation while braking (no regenerative)
        for i in range(4):
            if self.brake > 1:
                self.brake_temps[i] += (self.brake / 100.0)*1.8 + 0.1*abs(self.imu_g_long)
            else:
                self.brake_temps[i] -= 0.6 + 0.2*random.uniform(0,1)
            if self.brake_temps[i] > 540:
                self.brake_temps[i] -= 1.5  # radiative/convective extra
            self.brake_temps[i] = max(40.0, min(700, self.brake_temps[i]))

        # Motor/inverter temperature increases under load, with cooldown
        self.motor_temp += (self.current*0.012-(self.motor_temp-22.0)*0.019)*dt
        self.motor_temp = min(max(self.motor_temp, 35), 97)
        self.igbt_temp += (self.current*0.0115-(self.igbt_temp-22.0)*0.02)*dt
        self.igbt_temp = min(max(self.igbt_temp, 30), 85)
        self.air_temp = 22.0 + random.uniform(-0.5,0.5)

        # Cell / AMS temperatures
        cell_temp_rise = 0.03*self.current
        for module in self.modules:
            for i in range(len(module['temperatures'])):
                module['temperatures'][i] += cell_temp_rise*dt
                module['temperatures'][i] -= (module['temperatures'][i]-22.0)*0.015*dt
                module['temperatures'][i] += random.uniform(-0.07,0.09)
                module['temperatures'][i] = min(max(module['temperatures'][i],20), 57)
        all_temps = []
        for module in self.modules:
            all_temps.extend(module['temperatures'])
        self.ams_max_temp_c = max(all_temps)
        self.ams_min_temp_c = min(all_temps)
        self.ams_avg_temp_c = sum(all_temps) / len(all_temps)

        # Cell voltages: discharge & slight variance by temp
        avg_cell_v = (self.dc_bus_voltage*1000)/self.NUM_CELLS
        for module in self.modules:
            for i in range(len(module['cell_voltages'])):
                tv = avg_cell_v + random.uniform(-24,26) - (module['temperatures'][i%len(module['temperatures'])]-25.0)*0.45
                tv = max(self.CELL_MIN_V*1000, min(self.CELL_MAX_V*1000, tv))
                module['cell_voltages'][i] = tv
        all_volts = []
        for module in self.modules:
            all_volts.extend(module['cell_voltages'])
        self.ams_global_max_mv = max(all_volts)
        self.ams_global_min_mv = min(all_volts)
        self.ams_stack_total_mv = sum(all_volts)

        # Driver input ADC
        self.throttle_raw1 = int(1500 + self.throttle * 15)
        self.throttle_raw2 = int(1520 + self.throttle * 15.2)
        self.brake_raw = int(800 + self.brake * 10)
        for i in range(4):
            self.suspension[i] = self.susp_travel[i] + random.uniform(-1,1)

    def _update_can_data(self):
        self.data[0x600] = {
            'dcbusvoltage': self.dc_bus_voltage,
            'dcbuspower': self.dc_bus_power,
            'rpm': self.rpm,
            'torquetotal': self.torque,
            'cellminv': self.ams_global_min_mv,
            'throttleraw1': self.throttle_raw1,
            'throttleraw2': self.throttle_raw2
        }
        self.data[0x610] = {
            'motortemp': self.motor_temp,
            'pwrstgtemp': self.igbt_temp,
            'airtemp': self.air_temp,
            'nactual': self.rpm,
            'iactual': self.current
        }
        self.data[0x620] = {
            's1raw': self.throttle_raw1,
            's2raw': self.throttle_raw2,
            'brakeraw': self.brake_raw,
            'prechargebutton': self.precharge_button,
            'startbutton': self.start_button
        }
        self.data[0x630] = {
            'torquereq': self.torque,
            'torqueest': self.torque * 0.98,
            'throttle': self.throttle,
            'brake': self.brake
        }
        self.data[0x640] = {
            'currentsensor': self.current,
            'cellminv': self.ams_global_min_mv,
            'cellmaxtemp': self.ams_max_temp_c
        }
        self.data[0x645] = {
            'dst1': self.suspension[0],
            'dst2': self.suspension[1],
            'dst3': self.suspension[2],
            'dst4': self.suspension[3],
            'dsavg': sum(self.suspension) / 4.0,
            'dsmax': max(self.suspension),
            'dscount': 4
        }
        self.data[0x680] = {
            'status': self.system_status,
            'errors': self.error_code
        }
        self.data[0x201] = {
            'currentdA': self.ams_current_dA
        }
        self.data[0x202] = {
            'maxcellmv': self.ams_global_max_mv,
            'mincellmv': self.ams_global_min_mv,
            'stacktotalmv': self.ams_stack_total_mv
        }
        self.data[0x208] = {
            'maxtempc': self.ams_max_temp_c,
            'mintempc': self.ams_min_temp_c,
            'avgtempc': self.ams_avg_temp_c,
            'validcount': 190
        }

        # Dynamics panel: IMU (G-forces)
        self.data[0x700] = {
            'g_long': self.imu_g_long, 'g_lat': self.imu_g_lat, 'g_total': self.imu_g_total
        }
        # Suspension
        self.data[0x710] = {
            'susp_forces': self.susp_force, 'susp_travel': self.susp_travel
        }
        # Brake temp
        self.data[0x720] = {
            'brake_temp_fl': self.brake_temps[0],
            'brake_temp_fr': self.brake_temps[1],
            'brake_temp_rl': self.brake_temps[2],
            'brake_temp_rr': self.brake_temps[3]
        }

        # Module-specific data
        for i, module in enumerate(self.modules):
            self.data[f'module{i}_voltages'] = module['cell_voltages']
            self.data[f'module{i}_temps'] = module['temperatures']

    def get_latest_data(self) -> Dict[str, Any]:
        with self.lock:
            return self.data.copy()

    def get_ams_module_data(self, module_id: int) -> Dict[str, Any]:
        with self.lock:
            if 0 <= module_id < len(self.modules):
                return {
                    'voltages': self.modules[module_id]['cell_voltages'].copy(),
                    'tempsc': self.modules[module_id]['temperatures'].copy()
                }
        return {'voltages': [], 'tempsc': []}

# Global instance and API
_demo_generator = None

def start_demo():
    global _demo_generator
    if _demo_generator is None:
        _demo_generator = DemoDataGenerator()
    _demo_generator.start()

def stop_demo():
    global _demo_generator
    if _demo_generator:
        _demo_generator.stop()

def get_latest_data() -> Dict[str, Any]:
    global _demo_generator
    if _demo_generator:
        return _demo_generator.get_latest_data()
    return {}

def get_ams_module_data(module_id: int) -> Dict[str, Any]:
    global _demo_generator
    if _demo_generator:
        return _demo_generator.get_ams_module_data(module_id)
    return {'voltages': [], 'tempsc': []}

def is_demo_running() -> bool:
    global _demo_generator
    return _demo_generator is not None and _demo_generator.running
