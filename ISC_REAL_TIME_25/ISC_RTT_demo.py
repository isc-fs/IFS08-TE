"""
ISC_RTT_Demo.py
Demo data generator for ISC Formula Student Telemetry System
Simulates realistic CAN bus data for AMS, Inverter, Motor, and Driver inputs
Simulates a Formula Student car (85kW, 430V battery) driving Circuit de Barcelona-Catalunya
Author: Andrés Sánchez de Ojeda 2025-2026
"""

import time
import threading
import random
import math
from typing import Dict, Any
from datetime import datetime


class DemoDataGenerator:
    """
    Generates realistic dummy data for all CAN IDs used in the telemetry system.
    Simulates a Formula Student car with 85kW power, 430V battery pack driving
    Circuit de Barcelona-Catalunya with realistic acceleration, braking, and cornering.
    """
    
    def __init__(self):
        self.running = False
        self.thread = None
        self.lock = threading.Lock()
        
        # Vehicle specifications
        self.MAX_POWER_KW = 85.0  # 110 HP
        self.MAX_TORQUE_NM = 230.0
        self.MAX_RPM = 12000.0
        self.MAX_SPEED_KPH = 170.0
        self.WHEEL_DIAMETER_M = 0.51  # 20 inch wheels typical for FS
        self.FINAL_DRIVE_RATIO = 3.5
        self.VEHICLE_MASS_KG = 250.0  # Typical FS car mass with driver
        
        # Battery specifications
        self.NUM_CELLS = 95  # Total cells in series
        self.CELL_NOMINAL_V = 4.0
        self.CELL_MAX_V = 4.2
        self.CELL_MIN_V = 3.6
        self.BATTERY_NOMINAL_V = self.NUM_CELLS * self.CELL_NOMINAL_V  # 380V
        self.BATTERY_MAX_V = self.NUM_CELLS * self.CELL_MAX_V  # 399V
        self.BATTERY_MIN_V = 360.0  # Safety shutdown voltage
        self.BATTERY_CAPACITY_AH = 6.0  # Typical FS battery
        self.BATTERY_ENERGY_WH = self.BATTERY_NOMINAL_V * self.BATTERY_CAPACITY_AH  # ~2.3 kWh
        
        # Simulation state
        self.time_elapsed = 0.0
        self.session_start_time = time.time()
        self.total_energy_used_wh = 0.0
        
        # Vehicle dynamics
        self.speed_kmh = 0.0
        self.rpm = 0.0
        self.throttle = 0.0  # 0-100%
        self.brake = 0.0  # 0-100%
        self.torque = 0.0
        self.current = 0.0
        
        # Battery state
        self.dc_bus_voltage = self.BATTERY_MAX_V
        self.dc_bus_power = 0.0
        self.state_of_charge = 100.0  # Percentage
        
        # AMS data
        self.ams_current_dA = 0.0  # deciAmperes
        self.ams_global_max_mv = self.CELL_MAX_V * 1000
        self.ams_global_min_mv = self.CELL_MAX_V * 1000
        self.ams_stack_total_mv = self.BATTERY_MAX_V * 1000
        self.ams_max_temp_c = 25.0
        self.ams_min_temp_c = 25.0
        self.ams_avg_temp_c = 25.0
        
        # Module-specific data (5 modules, 19 cells each, 38 temps each)
        self.modules = []
        for mod_id in range(5):
            module = {
                'id': mod_id,
                'cell_voltages': [self.CELL_MAX_V * 1000 + random.uniform(-10, 10) for _ in range(19)],
                'temperatures': [25.0 + random.uniform(-1, 1) for _ in range(38)],
                'base_temp': 25.0  # Base temperature for thermal modeling
            }
            self.modules.append(module)
        
        # Motor/Inverter temperatures
        self.motor_temp = 40.0
        self.igbt_temp = 35.0
        self.air_temp = 20.0
        
        # Driver inputs (raw sensors)
        self.throttle_raw1 = 1500
        self.throttle_raw2 = 1520
        self.brake_raw = 800
        self.precharge_button = 0
        self.start_button = 0
        
        # Suspension sensors (4 corners)
        self.suspension = [50.0 for _ in range(4)]
        
        # Circuit simulation
        self.lap_time = 90.0  # Target lap time in seconds for Barcelona FS layout
        self.lap_progress = 0.0  # 0.0 to 1.0
        
        # Barcelona Catalunya circuit profile (normalized 0-1 for lap progress)
        # Format: (position, target_speed_kph, corner_name)
        self.circuit_profile = [
            (0.00, 120, "Start/Finish Straight"),
            (0.12, 60, "Turn 1 - Elf (slow right)"),
            (0.18, 90, "Turn 2 - Exit acceleration"),
            (0.28, 130, "Turn 3 - Renault (fast right)"),
            (0.35, 80, "Turn 4 - Repsol (medium left)"),
            (0.42, 70, "Turn 5 - Seat (slow right)"),
            (0.50, 100, "Turn 7 - Caixa (medium right)"),
            (0.60, 140, "Back straight"),
            (0.68, 75, "Turn 9 - Campsa (fast left)"),
            (0.78, 110, "Turn 10 (medium speed)"),
            (0.88, 90, "Turn 13 - La Caixa (slow)"),
            (0.95, 120, "Final corner exit"),
            (1.00, 130, "Finish line approach")
        ]
        
        # System status
        self.system_status = 2  # 2=Running
        self.error_code = 0
        
        # Latest data packet
        self.data = {}
        
    def start(self):
        """Start the demo data generation thread"""
        if self.running:
            return
        self.running = True
        self.session_start_time = time.time()
        self.total_energy_used_wh = 0.0
        self.thread = threading.Thread(target=self._generation_loop, daemon=True)
        self.thread.start()
        print("[DEMO] Barcelona Circuit simulation started - FS Car (85kW, 430V)")
        
    def stop(self):
        """Stop the demo data generation"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        print("[DEMO] Circuit simulation stopped")
        
    def _generation_loop(self):
        """Main loop that updates dummy data periodically"""
        update_rate = 0.05  # 50ms = 20Hz update rate
        
        while self.running:
            start_time = time.time()
            
            # Update simulation physics
            self._update_vehicle_dynamics(update_rate)
            
            # Update all CAN data packets
            with self.lock:
                self._update_can_data()
            
            # Sleep to maintain update rate
            elapsed = time.time() - start_time
            sleep_time = max(0, update_rate - elapsed)
            time.sleep(sleep_time)
            
            self.time_elapsed += update_rate
    
    def _get_target_speed_at_position(self, position):
        """Get target speed for current position on circuit"""
        # Find the two nearest points in circuit profile
        for i in range(len(self.circuit_profile) - 1):
            if self.circuit_profile[i][0] <= position <= self.circuit_profile[i + 1][0]:
                p1, s1, _ = self.circuit_profile[i]
                p2, s2, _ = self.circuit_profile[i + 1]
                # Linear interpolation
                ratio = (position - p1) / (p2 - p1) if p2 != p1 else 0
                return s1 + (s2 - s1) * ratio
        return self.circuit_profile[0][1]
    
    def _update_vehicle_dynamics(self, dt):
        """Simulate realistic vehicle behavior around Barcelona circuit"""
        # Update lap progress (cycles every lap_time seconds)
        self.lap_progress = (self.time_elapsed % self.lap_time) / self.lap_time
        
        # Get target speed for current position
        target_speed = self._get_target_speed_at_position(self.lap_progress)
        
        # Calculate throttle and brake based on speed difference
        speed_error = target_speed - self.speed_kmh
        
        if speed_error > 5:
            # Need to accelerate
            self.throttle = min(100.0, max(0, speed_error * 2.0))
            self.brake = 0.0
        elif speed_error < -5:
            # Need to brake
            self.throttle = 0.0
            self.brake = min(100.0, abs(speed_error) * 2.5)
        else:
            # Maintain speed
            self.throttle = 40.0 + random.uniform(-5, 5)
            self.brake = 0.0
        
        # Add some driver variation
        self.throttle += random.uniform(-3, 3)
        self.throttle = max(0, min(100, self.throttle))
        
        # Calculate motor torque (electric motor has flat torque curve until base speed)
        base_speed_rpm = 6000.0  # RPM where power becomes limiting
        if self.rpm < base_speed_rpm:
            available_torque = self.MAX_TORQUE_NM
        else:
            # Constant power region
            available_torque = (self.MAX_POWER_KW * 9549.0) / self.rpm if self.rpm > 0 else self.MAX_TORQUE_NM
        
        self.torque = (self.throttle / 100.0) * available_torque
        
        # Calculate wheel speed from vehicle speed
        wheel_speed_rps = (self.speed_kmh / 3.6) / (math.pi * self.WHEEL_DIAMETER_M)
        self.rpm = wheel_speed_rps * 60.0 * self.FINAL_DRIVE_RATIO
        self.rpm = max(0, min(self.MAX_RPM, self.rpm))
        
        # Calculate power demand
        power_kw = (self.torque * self.rpm) / 9549.0
        
        # Calculate current draw (no regenerative braking!)
        if self.throttle > 0:
            self.current = (power_kw * 1000.0) / self.dc_bus_voltage if self.dc_bus_voltage > 0 else 0
            self.current = max(0, min(300, self.current))  # Max 300A current limit
        else:
            self.current = 5.0 + random.uniform(-2, 2)  # Small auxiliary load
        
        # Update energy consumption
        energy_delta_wh = (power_kw * dt) / 3600.0  # Convert to Wh
        self.total_energy_used_wh += energy_delta_wh
        
        # Update state of charge (battery drains over 30 minutes)
        self.state_of_charge = 100.0 - (self.total_energy_used_wh / self.BATTERY_ENERGY_WH) * 100.0
        self.state_of_charge = max(0, self.state_of_charge)
        
        # Update battery voltage based on SoC and load
        # Voltage sags under load and decreases with discharge
        base_voltage = self.BATTERY_MIN_V + (self.BATTERY_MAX_V - self.BATTERY_MIN_V) * (self.state_of_charge / 100.0)
        voltage_sag = self.current * 0.08  # Internal resistance effect
        self.dc_bus_voltage = base_voltage - voltage_sag + random.uniform(-1, 1)
        self.dc_bus_voltage = max(self.BATTERY_MIN_V, min(self.BATTERY_MAX_V, self.dc_bus_voltage))
        
        # DC bus power
        self.dc_bus_power = self.dc_bus_voltage * self.current
        
        # AMS current (positive during discharge, NO regenerative braking)
        self.ams_current_dA = self.current * 10.0
        
        # Calculate acceleration/deceleration
        if self.brake > 0:
            # Braking deceleration (friction brakes only)
            decel_mps2 = (self.brake / 100.0) * 15.0  # Max 1.5G braking
            self.speed_kmh -= (decel_mps2 * 3.6 * dt)
        elif self.throttle > 0:
            # Acceleration
            # F = ma, but limited by motor torque at wheels
            wheel_force = (self.torque * self.FINAL_DRIVE_RATIO) / (self.WHEEL_DIAMETER_M / 2.0)
            accel_mps2 = wheel_force / self.VEHICLE_MASS_KG
            accel_mps2 = min(accel_mps2, 10.0)  # Max ~1G acceleration
            # Subtract drag and rolling resistance
            drag_force = 0.5 * 1.2 * 0.5 * 1.5 * ((self.speed_kmh / 3.6) ** 2)  # Simplified
            rolling_resistance = self.VEHICLE_MASS_KG * 9.81 * 0.012
            resistance_decel = (drag_force + rolling_resistance) / self.VEHICLE_MASS_KG
            net_accel = max(0, accel_mps2 - resistance_decel)
            self.speed_kmh += (net_accel * 3.6 * dt)
        else:
            # Coasting - deceleration from drag and rolling resistance
            drag_force = 0.5 * 1.2 * 0.5 * 1.5 * ((self.speed_kmh / 3.6) ** 2)
            rolling_resistance = self.VEHICLE_MASS_KG * 9.81 * 0.012
            decel_mps2 = (drag_force + rolling_resistance) / self.VEHICLE_MASS_KG
            self.speed_kmh -= (decel_mps2 * 3.6 * dt)
        
        self.speed_kmh = max(0, min(self.MAX_SPEED_KPH, self.speed_kmh))
        
        # Temperature modeling
        # Motor and inverter temperatures increase with power dissipation
        heat_factor = (self.current * 0.015)  # Heat generation
        cooling_factor = (self.motor_temp - 25.0) * 0.02  # Natural cooling
        
        self.motor_temp += (heat_factor - cooling_factor) * dt
        self.motor_temp = max(30, min(95, self.motor_temp))
        
        igbt_heat = (self.current * 0.012)
        igbt_cooling = (self.igbt_temp - 25.0) * 0.025
        self.igbt_temp += (igbt_heat - igbt_cooling) * dt
        self.igbt_temp = max(25, min(85, self.igbt_temp))
        
        self.air_temp = 20.0 + random.uniform(-1, 1)
        
        # Battery cell temperatures
        for module in self.modules:
            module_heat = self.current * 0.008  # Heat from current flow
            for i in range(len(module['temperatures'])):
                temp_diff = module['temperatures'][i] - module['base_temp']
                cooling = temp_diff * 0.015
                module['temperatures'][i] += (module_heat - cooling) * dt
                # Add some cell-to-cell variation
                module['temperatures'][i] += random.uniform(-0.05, 0.05)
                module['temperatures'][i] = max(20, min(60, module['temperatures'][i]))
        
        # Update global AMS temps
        all_temps = []
        for module in self.modules:
            all_temps.extend(module['temperatures'])
        self.ams_max_temp_c = max(all_temps)
        self.ams_min_temp_c = min(all_temps)
        self.ams_avg_temp_c = sum(all_temps) / len(all_temps)
        
        # Update cell voltages based on discharge
        avg_cell_voltage = (self.dc_bus_voltage * 1000.0) / self.NUM_CELLS
        
        for module in self.modules:
            for i in range(len(module['cell_voltages'])):
                # Cells discharge and have slight variations
                module['cell_voltages'][i] = avg_cell_voltage + random.uniform(-30, 30)
                # Temperature affects voltage slightly
                temp_effect = (module['temperatures'][i % len(module['temperatures'])] - 25.0) * 0.2
                module['cell_voltages'][i] -= temp_effect
                module['cell_voltages'][i] = max(self.CELL_MIN_V * 1000, min(self.CELL_MAX_V * 1000, module['cell_voltages'][i]))
        
        # Update global cell voltages
        all_voltages = []
        for module in self.modules:
            all_voltages.extend(module['cell_voltages'])
        self.ams_global_max_mv = max(all_voltages)
        self.ams_global_min_mv = min(all_voltages)
        self.ams_stack_total_mv = sum(all_voltages)
        
        # Raw sensor values (ADC-like)
        self.throttle_raw1 = int(1500 + self.throttle * 15)
        self.throttle_raw2 = int(1520 + self.throttle * 15.2)
        self.brake_raw = int(800 + self.brake * 10)
        
        # Suspension (simulate road bumps and cornering G-forces)
        # More variation during high-speed corners
        bump_intensity = (self.speed_kmh / self.MAX_SPEED_KPH) * 3.0
        for i in range(4):
            self.suspension[i] += random.uniform(-bump_intensity, bump_intensity)
            self.suspension[i] = max(20, min(80, self.suspension[i]))
    
    def _update_can_data(self):
        """Update the data dictionary with all CAN IDs"""
        # 0x600: DC bus, battery info, RPM, torque, throttle
        self.data[0x600] = {
            'dcbusvoltage': self.dc_bus_voltage,
            'dcbuspower': self.dc_bus_power,
            'rpm': self.rpm,
            'torquetotal': self.torque,
            'cellminv': self.ams_global_min_mv,
            'throttleraw1': self.throttle_raw1,
            'throttleraw2': self.throttle_raw2
        }
        
        # 0x610: Inverter state, temperatures
        self.data[0x610] = {
            'motortemp': self.motor_temp,
            'pwrstgtemp': self.igbt_temp,
            'airtemp': self.air_temp,
            'nactual': self.rpm,
            'iactual': self.current
        }
        
        # 0x620: Pedal sensors
        self.data[0x620] = {
            's1raw': self.throttle_raw1,
            's2raw': self.throttle_raw2,
            'brakeraw': self.brake_raw,
            'prechargebutton': self.precharge_button,
            'startbutton': self.start_button
        }
        
        # 0x630: Torque and pedal percentages (processed)
        self.data[0x630] = {
            'torquereq': self.torque,
            'torqueest': self.torque * 0.98,  # Estimated slightly lower
            'throttle': self.throttle,
            'brake': self.brake
        }
        
        # 0x640: Additional sensors
        self.data[0x640] = {
            'currentsensor': self.current,
            'cellminv': self.ams_global_min_mv,
            'cellmaxtemp': self.ams_max_temp_c
        }
        
        # 0x645: Suspension sensors
        self.data[0x645] = {
            'dst1': self.suspension[0],
            'dst2': self.suspension[1],
            'dst3': self.suspension[2],
            'dst4': self.suspension[3],
            'dsavg': sum(self.suspension) / 4.0,
            'dsmax': max(self.suspension),
            'dscount': 4
        }
        
        # 0x680: System status
        self.data[0x680] = {
            'status': self.system_status,
            'errors': self.error_code
        }
        
        # AMS Data - 0x201: Current
        self.data[0x201] = {
            'currentdA': self.ams_current_dA
        }
        
        # 0x202: Voltage summary
        self.data[0x202] = {
            'maxcellmv': self.ams_global_max_mv,
            'mincellmv': self.ams_global_min_mv,
            'stacktotalmv': self.ams_stack_total_mv
        }
        
        # 0x208: Temperature summary
        self.data[0x208] = {
            'maxtempc': self.ams_max_temp_c,
            'mintempc': self.ams_min_temp_c,
            'avgtempc': self.ams_avg_temp_c,
            'validcount': 190  # 5 modules * 38 sensors
        }
        
        # Module-specific data
        for i, module in enumerate(self.modules):
            self.data[f'module{i}_voltages'] = module['cell_voltages']
            self.data[f'module{i}_temps'] = module['temperatures']
    
    def get_latest_data(self) -> Dict[str, Any]:
        """Get the latest generated data (thread-safe)"""
        with self.lock:
            return self.data.copy()
    
    def get_ams_module_data(self, module_id: int) -> Dict[str, Any]:
        """Get specific AMS module data"""
        with self.lock:
            if 0 <= module_id < len(self.modules):
                return {
                    'voltages': self.modules[module_id]['cell_voltages'].copy(),
                    'tempsc': self.modules[module_id]['temperatures'].copy()
                }
        return {'voltages': [], 'tempsc': []}


# Global demo instance
_demo_generator = None


def start_demo():
    """Start the demo data generator"""
    global _demo_generator
    if _demo_generator is None:
        _demo_generator = DemoDataGenerator()
    _demo_generator.start()


def stop_demo():
    """Stop the demo data generator"""
    global _demo_generator
    if _demo_generator:
        _demo_generator.stop()


def get_latest_data() -> Dict[str, Any]:
    """Get latest demo data"""
    global _demo_generator
    if _demo_generator:
        return _demo_generator.get_latest_data()
    return {}


def get_ams_module_data(module_id: int) -> Dict[str, Any]:
    """Get AMS module demo data"""
    global _demo_generator
    if _demo_generator:
        return _demo_generator.get_ams_module_data(module_id)
    return {'voltages': [], 'tempsc': []}


def is_demo_running() -> bool:
    """Check if demo is currently running"""
    global _demo_generator
    return _demo_generator is not None and _demo_generator.running
