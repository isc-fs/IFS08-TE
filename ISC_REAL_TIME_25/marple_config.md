# Marple Dashboard Reconfiguration Guide
### Migration: TelFrame CSV → Snapshot CSV (ISC RTT v2)

---

## Overview

The radio protocol and CSV schema have been redesigned. This document is a
step-by-step checklist to update every Marple panel, channel, and alert
to the new column names and units.

> [!IMPORTANT]
> Upload a **new test session CSV** with the v2 software before starting.
> You need at least one file with the new headers loaded in Marple so that
> the new channel names are available to select.

---

## Step 1 — Rename Channels (direct 1-to-1 remaps)

For each panel that references the old column, swap it to the new one.
No unit or scale change — just a name update.

| Old channel name | → | New channel name |
|---|---|---|
| `start_btn` | → | `start_button` |
| `throttle_raw1` | → | `apps1_raw` |
| `throttle_raw2` | → | `apps2_raw` |
| `s1_raw` | → | `apps1_raw` |
| `s2_raw` | → | `apps2_raw` |
| `brake_raw` | → | `brake_raw` *(unchanged)* |
| `ams_min_cell_mv` | → | `v_cell_min_mV` |
| `dc_bus_voltage` | → | `inv_dc_bus_V` |
| `rpm` | → | `inv_rpm` |
| `motor_temp` | → | `inv_temp_motor1` |
| `pwrstg_temp` | → | `inv_temp_pwrstg` |
| `n_actual` | → | `inv_speed_actual` |
| `i_actual` | → | `inv_current_actual` |
| `precharge_btn` | → | `ok_precharge` |
| `torque_total` | → | `torque_pct` |

---

## Step 2 — Fix Unit Mismatches

> [!CAUTION]
> These channels have **changed units or scale**. Axis ranges, threshold
> alerts, and formulas that reference them will be wrong until fixed.

### 2.1 — Cell Minimum Voltage

| | Old | New |
|---|---|---|
| Column | `cell_min_v` | `v_cell_min_mV` |
| Unit | **Volts** (float) | **millivolts** (uint16) |
| Scale factor | 1.0 | ×1000 |

**Action:** In every panel showing this channel:
- Replace `cell_min_v` with `v_cell_min_mV`
- Divide axis range by 1000 — e.g. if your Y-axis was `3.0 – 4.2 V`, change it to `3000 – 4200 mV`
- Update any threshold alerts (e.g. `< 3.0` → `< 3000`)

### 2.2 — Pack Current

| | Old | New |
|---|---|---|
| Column | `ams_current_a` | `corriente_accu` |
| Unit | **Amperes** (pre-divided by 10 in Python) | **raw int16** (unit TBD on STM32 side) |

**Action:**
1. Confirm with the STM32 firmware what unit `veh.corriente_accu` is in
   (likely deciamperes, milliamperes, or raw ADC counts).
2. Apply the correct divisor in a **Marple derived channel** (see Step 3).
3. Replace `ams_current_a` with the derived channel in all panels.

---

## Step 3 — Create Derived Channels

The old Python layer pre-computed aggregated values (max, min, avg) from
per-module data before writing the CSV. The new CSV stores **raw per-module
values** instead. Recreate the aggregates as Marple derived channels.

### 3.1 — Pack Maximum Cell Voltage

```
Name:    ams_max_cell_mv
Formula: MAX(vmax_mod0, vmax_mod1, vmax_mod2, vmax_mod3, vmax_mod4)
Unit:    mV
```

### 3.2 — Pack Maximum Temperature

```
Name:    ams_max_temp_c
Formula: MAX(tmax_mod0, tmax_mod1, tmax_mod2, tmax_mod3, tmax_mod4)
Unit:    °C
```

### 3.3 — Pack Minimum Temperature

```
Name:    ams_min_temp_c
Formula: MIN(tmax_mod0, tmax_mod1, tmax_mod2, tmax_mod3, tmax_mod4)
Unit:    °C
Note:    Only module-level maximums are available; true cell-level min
         temperature is no longer transmitted over radio.
```

### 3.4 — Pack Average Temperature (optional)

```
Name:    ams_avg_temp_c
Formula: (tmax_mod0 + tmax_mod1 + tmax_mod2 + tmax_mod3 + tmax_mod4) / 5
Unit:    °C
```

### 3.5 — Pack Current in Amperes (once unit confirmed)

Example assuming STM32 stores in deciamperes (×0.1):
```
Name:    current_A
Formula: corriente_accu * 0.1
Unit:    A
```

---

## Step 4 — Delete Obsolete Channels

> [!WARNING]
> These columns no longer exist in the CSV. Remove them from all panels
> to avoid broken/empty series.

### 4.1 — Channels dropped from the data pipeline

| Channel | Was |
|---|---|
| `air_temp` | Inverter air temperature (not in snapshot) |
| `torque_req` | Requested torque from 0x630 CAN frame |
| `torque_est` | Estimated torque from 0x630 CAN frame |
| `throttle_pct` | Processed throttle % from 0x630 |
| `brake_pct` | Processed brake % from 0x630 |
| `ams_stack_v` | Full stack voltage (not serialised) |
| `ams_avg_temp_c` | Pre-computed average (use derived channel instead) |

### 4.2 — Dynamics placeholders (were always zero)

These were never actually populated with data, so no information is lost:

`g_long` · `g_lat` · `g_total` ·
`susp_force_fl` · `susp_force_fr` · `susp_force_rl` · `susp_force_rr` ·
`susp_travel_fl` · `susp_travel_fr` · `susp_travel_rl` · `susp_travel_rr` ·
`brake_temp_fl` · `brake_temp_fr` · `brake_temp_rl` · `brake_temp_rr`

---

## Step 5 — Add New Channels to Dashboard

These signals are **new** — they were not available before.
Consider adding panels for the most operationally useful ones.

### Recommended additions

| Channel | Type | Description | Suggested panel |
|---|---|---|---|
| `seq` | uint16 | Snapshot sequence counter | Comms health — plot delta to detect dropped snapshots |
| `tick_ms` | uint32 | STM32 RTOS timestamp (ms) | Time sync reference |
| `soc` | uint8 | State of charge (%) | Battery panel |
| `inv_error` | uint8 | Inverter error code | Fault panel — alert on non-zero |
| `inv_state` | uint8 | Inverter state machine | Inverter panel |
| `ctrl_state` | uint8 | Control state machine | Control panel |
| `inv_temp_board` | uint16 | Inverter board temperature | Thermal panel |
| `corriente_dcdc` | int16 | DC-DC converter current | Power panel |
| `temp_dcdc` | int16 | DC-DC converter temperature | Thermal panel |
| `ams_fsm_state` | uint8 | AMS state machine | Battery panel |
| `vmin_mod0..4` | uint16 | Per-module min cell voltage (mV) | Per-module battery panel |
| `vmax_mod0..4` | uint16 | Per-module max cell voltage (mV) | Per-module battery panel |
| `tmax_mod0..4` | int16 | Per-module max temperature (°C) | Per-module thermal panel |

### Dropped-snapshot detector (derived channel)

To monitor radio link quality, create a derived channel that counts
missing sequence numbers:

```
Name:    seq_jump
Formula: seq - PREV(seq)   [or equivalent delta function in Marple]
Alert:   seq_jump > 1  →  "Radio packet loss"
```

---

## Checklist Summary

- [ ] Upload a v2 test CSV to Marple so new channels are available
- [ ] Rename all channels per Step 1
- [ ] Fix `v_cell_min_mV` axis ranges and alerts (Step 2.1)
- [ ] Confirm `corriente_accu` unit with firmware team (Step 2.2)
- [ ] Create derived channels: `ams_max_cell_mv`, `ams_max_temp_c`, `ams_min_temp_c` (Step 3)
- [ ] Create `current_A` derived channel once unit is confirmed (Step 3.5)
- [ ] Remove all channels listed in Step 4
- [ ] Add priority new channels: `soc`, `inv_error`, `seq` (Step 5)
- [ ] Set alert on `inv_error != 0`
- [ ] Set alert on `seq_jump > 1` for radio link monitoring