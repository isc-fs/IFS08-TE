// -----------------------------------------------------------------------------
// Author       :   Luis de la Barba & Javier R. Juliani
// Date         :   17/04/2020
// Adaptation   :   Juan Mata & Jaime Landa
// Date         :   03/2024
// Updated		: 	10/2025
// Author		: 	Andrés Sánchez de Ágreda
// Name         :   module_state_machine.h
// Description  :   This file is for the finite state machine initialization
// -----------------------------------------------------------------------------

#ifndef INC_MODULE_STATE_MACHINE_H_
#define INC_MODULE_STATE_MACHINE_H_

#include "main.h"

// Forward declaration to avoid circular dependencies
class BMS_MOD;

extern TIM_HandleTypeDef htim17;

#define CHARGE_MIN_CURRENT_ABS  100  // mA - adjust to your needs
#define CHARGE_FAIL_TIMEOUT_MS  5000 // ms - adjust to your needs

// CPU states
#define CPU_DISCONNECTED         0
#define CPU_PRECHARGE           1
#define CPU_POWER               2
#define CPU_CHARGING            3
#define CPU_ERROR               4

// CPU error codes
#define CPU_OK                  0
#define CPU_ERROR_COMMUNICATION 1

// Current error codes
#define Current_OK              0
#define Current_ERROR_Comunication  1

// -----------------------------------------------------------------------------
// --- FAN TIMER CONFIG ---------------------------------------------------------
// -----------------------------------------------------------------------------
#define FAN_TIMER_ARR   10559

// -----------------------------------------------------------------------------
// --- BMS ----------------------------------------------------------------------
// -----------------------------------------------------------------------------
#define BMS_ID          0x12C
#define BMS_MAXV        4200    // mV
#define BMS_MINV        2800
#define BMS_MAXT        60      // Celsius
#define BMS_SHUNT       4000    // 3750 mV voltage for balancing
#define NUM_CELLS_PER_MODULE  12    // Number of cells per BMS module

// -----------------------------------------------------------------------------
// --- CAR ----------------------------------------------------------------------
// -----------------------------------------------------------------------------
#define CPU_ID_send     0x20
#define CPU_ID_recv     0x100

// -----------------------------------------------------------------------------
// --- Current Class ------------------------------------------------------------
// -----------------------------------------------------------------------------
#define Current_ID      0x450
#define Current_max     200
#define T_MAX           60
#define Temp_ID         0x400

// -----------------------------------------------------------------------------
// --- Digital Outputs ----------------------------------------------------------
// -----------------------------------------------------------------------------
#define RELAY_AIR_1         36  // 4
#define RELAY_AIR_2         35  // 5
#define RELAY_PRECHARGE     34  // 5 - 34

// -----------------------------------------------------------------------------
// --- Current States available for the Finite State Machine -------------------
// -----------------------------------------------------------------------------
enum STATE {
    start,
    precharge,
    transition,
    run,
    charge,
    error
};

// -----------------------------------------------------------------------------
// --- FUNCTIONS ----------------------------------------------------------------
// -----------------------------------------------------------------------------
void setup_state_machine(void);
void select_state(void);
void parse_state(CANMsg* data);
STATE get_state(void);
HAL_StatusTypeDef module_send_message_CAN1(uint32_t id, uint8_t* data, uint8_t length);

#endif /* INC_MODULE_STATE_MACHINE_H_ */
