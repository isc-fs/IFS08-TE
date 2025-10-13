/*
 * ams_can_map.h
 *
 *  Created on: Oct 13, 2025
 *      Author: Andres Sanchez de Agreda
 */

#ifndef INC_AMS_CAN_MAP_H_
#define INC_AMS_CAN_MAP_H_


// ams_can_map.h  (11-bit Standard IDs for CAN2)
#pragma once

// High-level AMS → Main-Board telemetry on CAN2 only
#define CAN2_ID_AMS_STATUS        0x200  // DC bus, AMS state
#define CAN2_ID_AMS_CURRENT       0x201  // Current (deci-amps, int16)
#define CAN2_ID_AMS_VOLT_SUM      0x202  // [MAX_V, MIN_V, STACK_mV]
#define CAN2_ID_AMS_VOLT_BLOCK0   0x203  // cells [0..3] mV (4x uint16 BE)
#define CAN2_ID_AMS_VOLT_BLOCK1   0x204  // cells [4..7]
#define CAN2_ID_AMS_VOLT_BLOCK2   0x205  // cells [8..11]
#define CAN2_ID_AMS_VOLT_BLOCK3   0x206  // cells [12..15]
#define CAN2_ID_AMS_VOLT_BLOCK4   0x207  // cells [16..19] (last bytes may pad 0xFF)

#define CAN2_ID_AMS_TEMP_SUM      0x208  // [MAX_T, MIN_T, AVG_Tx10, VALID_CNT]
#define CAN2_ID_AMS_TEMP_BLOCK0   0x209  // temps [0..7]  (1B each, °C; 0xFF pad)
#define CAN2_ID_AMS_TEMP_BLOCK1   0x20A  // temps [8..15]
#define CAN2_ID_AMS_TEMP_BLOCK2   0x20B  // temps [16..23]
#define CAN2_ID_AMS_TEMP_BLOCK3   0x20C  // temps [24..31]
#define CAN2_ID_AMS_TEMP_BLOCK4   0x20D  // temps [32..37] (pad as needed)

// Optional: current severity notifications (on CAN2)
#define CAN2_ID_AMS_CUR_WARN      0x250  // 80..100% of C_MAX
#define CAN2_ID_AMS_CUR_FAULT     0x251  // >100% of C_MAX
#define CAN2_ID_AMS_CUR_NORMAL    0x252  // back to nominal



#endif /* INC_AMS_CAN_MAP_H_ */
