![ISC Logo](http://iscracingteam.com/wp-content/uploads/2022/03/Picture5.jpg)

# IFS08-TE — Telemetry & Ground Station (ISCmetrics)

Software and firmware repository for the **Telemetry Department** of the IFS08, ISC Racing Team's Formula Student electric car (Season 2025/2026).

It contains the ground-station real-time telemetry viewer, USB-serial packet reassembler, cloud database integration (Marple Data), and receiver firmware for tracking the car on track and in the pit lane.

---

## 🏎️ Telemetry Architecture at a Glance

```
┌──────────────────────────────────────────────┐
│                  ON THE CAR                  │
│                                              │
│  [ECU (STM32H733)] ──(SPI Bit-Bang)──▶ [nRF24L01+]
│           │                                  │ (2.4 GHz, Ch 76, 1 Mbps)
│    (102-Byte Snapshot)                       ▼
└──────────────────────────────────────────────┼─··· 500m+ Air Link ···
                                               │
┌──────────────────────────────────────────────▼─────────────────────────┐
│                           PIT WALL / GROUND STATION                    │
│                                                                        │
│  [nRF24L01+ / RF-Nano] ──(USB-Serial @ 115200)──▶ [ISC_RTT_serial.py]  │
│                                                          │             │
│                                     ┌────────────────────┴──────────┐  │
│                                     ▼                               ▼  │
│                              [Flat CSV Log]                  [PyQt5 UI]│
│                                     │                         (ui.py)  │
│                                     ▼                                  │
│                              [Marple Data SDK]                         │
│                               (Cloud InfluxDB)                         │
└────────────────────────────────────────────────────────────────────────┘
```

---

## 📁 Repository Structure

```
IFS08-TE/
├── ISC_REAL_TIME_25/             # Primary Ground Station application
│   ├── ui.py                     # PyQt5 real-time telemetry GUI (Dark Grafana theme)
│   ├── ISC_RTT_serial.py         # Serial transport, nRF24 fragment reassembly & CSV logger
│   ├── isc_marple.py             # Marple Data Cloud API SDK integration
│   ├── ISC_RTT_demo.py           # Synthetic telemetry playback for UI development
│   ├── RTT_nano/                 # Arduino / RF-Nano receiver firmware
│   │   ├── RTT_nano.ino          # Standard RF-Nano USB receiver sketch
│   │   └── Nano_HRange/          # High-gain PA/LNA tuned receiver sketch
│   └── AMS_data/                 # Cell mapping & BMS post-processing utilities
├── nrf24/                        # Alternative STM32G431 hardware receiver project
├── NEW FHUB/                     # STM32H523 front hub sensor node project
├── logs/                         # Local storage for recorded .csv telemetry sessions
└── README.md                     # This document
```

---

## ⚡ What is Implemented & Verified in Season 08

### 1. Robust 102-Byte Radio Snapshot Protocol (v2)
- **Zero-loss fragment reassembly**: The car transmits five 24-byte payloads per 200 ms cycle (`magic=0xEC`, `version=0x03`, `kind=0x06`). The receiver validates headers, checksums, and sequence numbers, rebuilding the 102-byte frame without buffer overrun.
- **Full Signal Coverage**: All 45 critical vehicle signals are parsed live (ECU states, APPS/Brake raw ADCs, motor eRPM, inverter temperatures, DC bus voltage, per-module cell voltages & temperatures, and GPS).

### 2. Live GPS Tracking & Vector Map
- **MTK3339 GPS Integration**: Unpacks latitude, longitude, Doppler speed, heading/course, and satellite count from snapshot bytes `[82..95]`.
- **Vector Track Plotter**: Live track visualization in the Dynamics tab with auto-centering, trail rendering, and orientation arrow.
- **Fail-safe Gating**: Speed and position displays are gated strictly by `gps_has_fix` to eliminate stale position jumps when satellite fix drops.

### 3. Predictive Analytics Engine & Strategy Advisor
- **Integrated Wh Counter**: Trapezoidal numerical integration of pack power ($P = V_{bus} \times I_{accu}$) to provide a monotonic, reliable energy meter regardless of BMS SOC estimator availability.
- **Battery Internal Resistance ($R_{int}$) Estimator**: Real-time step-transition detection ($\Delta V / \Delta I$) tracking pack degradation and cell health.
- **Thermal Predictor**: Computes $dT/dt$ (°C/min) and estimates time-to-overtemperature trip (90 °C limit).
- **Adaptive Torque Recommendation**: Recommends torque limit percentages for 22-minute Endurance stints.

### 4. Diagnostics & Calibration Tools
- **Cross-Session Fault Pattern Database**: Scans historical telemetry logs to identify recurrent inverter DEM codes, trip frequencies, and cumulative fault durations.
- **Live Inverter DEM Decoder**: Decodes all 37 NxTech diagnostic codes with SafeState indicators.
- **Pedal Calibration Wizard**: Guided 3-step sampling flow with persistent storage in `settings.json`.
- **Brake Floor Normalization**: Calibrated zero-point offset (`BRK_MIN = 582`) ensuring 0% idle display.

### 5. Cloud Integration (Marple Data)
- Automated direct dataset upload to Marple cloud workspace for post-session telemetry debriefs.

---

## 🚀 Quickstart Guide

### Requirements
- Python 3.10+ (tested on Python 3.11 x64 on Windows/Linux)
- Required packages:
  ```bash
  pip install PyQt5 numpy pandas pyserial matplotlib pyarrow requests marple-data
  ```

### Running the Ground Station
1. Connect the RF-Nano or Arduino USB receiver.
2. Launch the telemetry application:
   ```bash
   cd ISC_REAL_TIME_25
   python ui.py
   ```
3. Open **Settings**, select the detected COM port (`115200` baud), enter driver/circuit metadata, and click **START**.

---

## 📋 End-of-Season Handover & Roadmap

The following items are documented for the incoming team taking over for Season 09:

### 1. 🗄️ MicroSD On-Car Data Extraction & Merging (High Priority)
- **Context**: The ECU and AMS write high-rate binary/CSV logs to on-board microSD cards during runs (full 95-cell individual voltages, 95-cell thermistors, 100 Hz IMU data).
- **What's Needed**:
  - Implement a dedicated **MicroSD Import Wizard** in the UI (or CLI tool in `ISC_REAL_TIME_25/`) to read files directly from the SD card reader.
  - Complete `merge_ams_temps_into_session()` and `merge_gps_into_session()` in `ISC_RTT_serial.py` to synchronize time bases using the RTOS `tick_ms` or wall-clock UTC timestamps.
  - Automatically append granular cell channels (`ams_t_mod0_cell0` .. `ams_t_mod4_cell18`) into the telemetry session CSV for Marple upload.

### 2. 📡 Dual-Receiver Log Stitcher
- **Context**: During long track runs (e.g., Montmeló, Red Bull Ring), track obstacles can cause local radio shadow. Running two receivers (one at the pit wall, one on the far side of the track) captures 100% of packets.
- **What's Needed**:
  - Finalize a built-in multi-file merge utility in the UI that combines two simultaneous receiver logs, using `seq` and `tick_ms` as primary keys, discarding duplicate packets and filling RF dropout gaps.

### 3. ⚡ Inverter AC Power & Phase Current Telemetry Ingest
- **Context**: `IFS08-CE-ECU` now decodes `inv_ac_power_W` (`0x466`) and FOC phase currents (`0x463`).
- **What's Needed**:
  - Once the ECU maps these into the radio snapshot or pit-diag frames, wire them into the Powertrain tab to calculate true mechanical-to-electrical efficiency ($\eta = P_{mech} / P_{elec}$) live.

### 4. ⏱️ Automated Event & Split Timing
- **Context**: Acceleration and Skidpad events require fast turnaround times in the paddock.
- **What's Needed**:
  - Add speed-threshold launch detection (e.g., crossing 5 km/h) and numerical distance integration ($d = \int v \, dt$) to display an instant **75m Acceleration Split (s)**, **0–100 km/h time**, and **peak longitudinal G**.
  - Add GPS bounding-box split triggers for automatic lap timing on circuit loops.

---

## 🔗 Related Repositories

| Repository | Purpose | Communication Contract |
|---|---|---|
| [`IFS08-CE-ECU`](https://github.com/isc-fs/IFS08-CE-ECU) | Vehicle Control Unit Firmware | 102-Byte Radio Snapshot / `0x510-0x521` Dash / `0x700-0x708` Pit-Diag |
| [`IFS08-CE-AMS`](https://github.com/isc-fs/IFS08-CE-AMS) | Accumulator Management System | `0x020` Precharge / `0x131-0x137` Cell Telemetry / `0x4A0` Status |
| [`IFS08-DV-uDV`](https://github.com/isc-fs/IFS08-DV-uDV) | Autonomous Driving System | `0x504-0x511` Autonomous Control & Verification |

---

*ISC Racing Team — Telemetry & Electronics Department (2025/2026)*
