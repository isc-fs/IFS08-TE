// -----------------------------------------------------------------------------
// Author       :   Luis de la Barba & Javier R. Juliani
// Date         :   17/04/2020
// Adaptation   :   Juan Mata & Jaime Landa
// Date         :   03/2024
// Updated		: 	10/2025
// Author		: 	Andrés Sánchez de Ágreda
// Name         :   module_state_machine.cpp
// Description  :   This file is for the finite states machine
// -----------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include "main.h"
#include "module_state_machine.h"
#include "class_bms.h"
#include "class_cpu.h"
#include "class_current.h"

extern TIM_HandleTypeDef htim17;

// INITIALIZE VARIABLES

// New BMS - one per module with voltage and temperature readings
BMS_MOD BMS[] = {
    BMS_MOD(BMS_ID + 00, BMS_MAXV, BMS_MINV, BMS_MAXT, NUM_CELLS_PER_MODULE, BMS_SHUNT, 50, 105),
    BMS_MOD(BMS_ID + 30, BMS_MAXV, BMS_MINV, BMS_MAXT, NUM_CELLS_PER_MODULE, BMS_SHUNT, 100, 205),
    BMS_MOD(BMS_ID + 60, BMS_MAXV, BMS_MINV, BMS_MAXT, NUM_CELLS_PER_MODULE, BMS_SHUNT, 150, 305),
    BMS_MOD(BMS_ID + 90, BMS_MAXV, BMS_MINV, BMS_MAXT, NUM_CELLS_PER_MODULE, BMS_SHUNT, 200, 405),
    BMS_MOD(BMS_ID + 120, BMS_MAXV, BMS_MINV, BMS_MAXT, NUM_CELLS_PER_MODULE, BMS_SHUNT, 250, 505),
};

int BMS_N = 5;

int MIN_V = 4200;
int MAX_T = 0;
uint8_t message_MIN_V[2] = {0, 0};  // Here I'll get the minimum voltages for sending them for telemetry
int time_sending_minV = 0;           // For checking the interval I send the messages
uint8_t message_MAX_T[2] = {0, 0};
int time_sending_maxT = 0;

uint16_t fan_speed = 0;

CPU_MOD CPU(CPU_ID_send, CPU_ID_recv, 500);  // Same with CPU, rest of vehicle

int flag_charger = 0;                         // For knowing whether I am charging or in the car
static uint32_t charge_current_error_counter = 0;
int flag_ams_ok = 1;
int flag_first_time = 0;
int flag_start_button = 0;

Current_MOD current(Current_ID, Current_max);  // Class for current measurement

static STATE state = STATE::start;            // Variable on the state machine
char buffer[80];

/*********************************************************************************************************
 ** Function name:           setup_state_machine
 ** Descriptions:            Initializes the state machine library
 *********************************************************************************************************/
void setup_state_machine(void) {
    HAL_GPIO_WritePin(AMS_OK_GPIO_Port, AMS_OK_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_AIR_N_GPIO_Port, RELAY_AIR_N_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_AIR_P_GPIO_Port, RELAY_AIR_P_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_PRECHARGE_GPIO_Port, RELAY_PRECHARGE_Pin, GPIO_PIN_RESET);
}

/*********************************************************************************************************
 ** Function name:           get_state
 ** Descriptions:            get the current state
 *********************************************************************************************************/
STATE get_state(void) {
    return state;
}

/*********************************************************************************************************
 ** Function name:           select_state
 ** Descriptions:            check what should they do on the state machine
 *********************************************************************************************************/
void select_state(void) {
    int state_air_n = 0;        // 0 means open, 1 closed - Turn on SEVCON
    int state_air_p = 0;        // 0 means open, 1 closed - Energize HV relay
    int state_precharge = 0;    // 0 means open, 1 closed
    int flag_cpu = CPU_ERROR_COMMUNICATION;
    int flag_current = Current_ERROR_Comunication;
    int gpio_charge = HAL_GPIO_ReadPin(Charge_Button_GPIO_Port, Charge_Button_Pin);  // pull-up 1 = charge started

    if (flag_first_time == 0) {
        flag_first_time = 1;
        HAL_Delay(1000);
        // printValue(gpio_charge);
    }

    // TIM16 -> APB2 = 264MHz
    // 1 ms interruption = 1ms * 264MHz = 264000 prescalado 264 por ejemplo
    // timer_count = 264000 / 264 = 1000

    uint32_t time = HAL_GetTick();
    int times = HAL_GetTick();
    CPU.voltage_acum = 0;  // For precharge

    MIN_V = 4200;  // I reset the number each cycle cause if the voltages goes up again I wanna has it risen again on telemetry
    MAX_T = 0;

    for (int i = 0; i < BMS_N; i++) {
        BMS[i].voltage_acum = 0;  // For precharge

        if (BMS[i].query_voltage(time, buffer) != BMS_OK) {  // I ask the BMS about voltages and checking their states
            state = STATE::error;
            flag_ams_ok = 0;
        }

        CPU.voltage_acum += BMS[i].voltage_acum;  // For precharge

        if (BMS[i].MIN_V < MIN_V) {
            MIN_V = BMS[i].MIN_V;  // Checking the minimum voltage of cell in the whole battery
        }

        // int currentvalue = readAnalogValue();
        // printValue(currentvalue);
        // if(MIN_V == 0 || currentvalue > 50 || BMS[i].query_voltage(time, buffer) != BMS_OK) {
        //     flag_ams_ok = 0;
        //     state = STATE::error;
        // }
        // printValue(MIN_V);
        // printValue(currentvalue);
        // printValue(BMS[i].query_voltage(time, buffer) != BMS_OK);
        // else {
        //     flag_ams_ok = 1;
        // }

        if (BMS[i].query_temperature(time, buffer) != BMS_OK) {
            state = STATE::error;
            // DESCOMENTA
        }

        if (BMS[i].MAX_T > MAX_T) {
            MAX_T = BMS[i].MAX_T;
        }
    }

    if ((times - time_sending_minV) > 500) {
        message_MIN_V[1] = MIN_V & 0xFF;
        message_MIN_V[0] = (MIN_V >> 8) & 0xFF;
        if (BMS[0].flag_charger != 1) {
            if (module_send_message_CAN1(BMS_ID, message_MIN_V, 2) != HAL_OK) {
                print((char*)"Error al enviar tension minima");  // Sending the message through telemetry each 500 ms
            }
        }
        time_sending_minV = times;
    }

    // No need to assign to BALANCING_V - it's a constant #define
    // for (int i = 0; i < BMS_N; i++) {
    //     BMS[i].BALANCING_V = MIN_V;  // Here I say I wanna balance all the cells in the battery to the minimum
    // }

    flag_cpu = CPU.query(time, buffer);  // Asking the rest of the car how is it
    // flag_cpu = CPU_OK;
    flag_current = current.query(time, buffer);  // asking current how is it

    // print((char*)"state: ");
    // printValue(state);
    // print((char*)"voltage_acu: ");
    // printValue(CPU.voltage_acum);
    // print((char*)"dc_bus: ");
    // printValue(CPU.DC_BUS);
    // printValue(state);

    switch (state) {
        case STATE::start:
            CPU.updateState(CPU_DISCONNECTED);
            fan_speed = 0;
            __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, fan_speed);
            if(gpio_charge == GPIO_PIN_SET) {
                state = STATE::charge;
            }
            if (flag_cpu != CPU_ERROR_COMMUNICATION && flag_start_button == 1) {
                state = STATE::transition;  // If I do communicate with the rest of the car, I go to precharge
            }
            break;

        case STATE::precharge:
            state_air_n = 1;
            state_air_p = 0;
            state_precharge = 1;
            CPU.updateState(CPU_PRECHARGE);
            if (flag_cpu == CPU_OK) {
                state = STATE::transition;
            } else if (flag_cpu == CPU_ERROR_COMMUNICATION) {
                state = STATE::error;
            } else if(flag_current != Current_OK) {
                state = STATE::error;  // I take this out cause in precharge current can be very high, but probably can be uncommented,
            }
            break;

        case STATE::transition:
            state_air_n = 1;
            state_air_p = 0;
            state_precharge = 1;
            CPU.updateState(CPU_PRECHARGE);
            if (CPU.voltage_acum > 0.7 * CPU.DC_BUS) {
                // printValue(CPU.DC_BUS);
                if(flag_start_button == 0 && CPU.DC_BUS == 0) {
                    state = STATE::start;
                }
                if(CPU.DC_BUS > 200 && CPU.DC_BUS < 500) {
                    state = STATE::run;  // If DCBUS voltage is higher than 90% of battery voltage, precharge finish
                }
            } else if(flag_cpu == CPU_ERROR_COMMUNICATION && flag_charger != 1) {
                state = STATE::error;
            } else if(flag_current != Current_OK) {
                state = STATE::error;
            }
            break;

        case STATE::run:
            state_air_n = 1;
            state_air_p = 1;
            state_precharge = 1;
            CPU.updateState(CPU_POWER);
            // print((char*)"run");
            // printValue(CPU.DC_BUS);
            if(CPU.DC_BUS < 60) {
                state = STATE::start;
                flag_start_button = 0;
            }
            fan_speed = FAN_TIMER_ARR * 75 / 100;
            __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, fan_speed);
            if (flag_cpu == CPU_ERROR_COMMUNICATION && flag_charger != 1) {
                state = STATE::error;  // If I disconnect the charger, error
            }
            // print((char*)"CPU: ");
            if (flag_current != Current_OK) {
                state = STATE::error;  // If current is too high, error
            }
            if(HAL_GPIO_ReadPin(DIGITAL1_GPIO_Port, DIGITAL1_Pin) == GPIO_PIN_RESET) {  // SDC_IO
                state = STATE::error;
            }
            // print((char*)"DIGITAL: ");
            break;

        case STATE::charge:
            state_air_n = 1;
            state_air_p = 1;
            state_precharge = 0;
            CPU.updateState(CPU_CHARGING);
            fan_speed = FAN_TIMER_ARR * 40 / 100;
            __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, fan_speed);

            int32_t current_act = current.Current * 1000;  // Actual current in mA to check if it's charging
            if(abs(current_act) < CHARGE_MIN_CURRENT_ABS) {
                if(charge_current_error_counter == 0) {
                    charge_current_error_counter = HAL_GetTick();
                } else if(HAL_GetTick() - charge_current_error_counter > CHARGE_FAIL_TIMEOUT_MS) {
                    state = STATE::error;  // Charge has been interrupted
                }
            } else {
                charge_current_error_counter = 0;
            }

            if (gpio_charge == GPIO_PIN_RESET) {
                state = STATE::start;
                charge_current_error_counter = 0;
            }
            break;

        case STATE::error:
            state_air_n = 0;  // All relays closed
            state_air_p = 0;
            state_precharge = 0;
            CPU.updateState(CPU_ERROR);
            // int currentvalue = readAnalogValue();
            for (int i = 0; i < BMS_N; i++) {
                // if(MIN_V != 0 && currentvalue < 50 && BMS[i].return_voltage_error() == BMS_OK) {
                //     state = STATE::start;
                // }
            }
            fan_speed = 0;
            __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, fan_speed);
            break;
    }

    sprintf(buffer, "");
    print(buffer);
    sprintf(buffer, " - STATE: %i", state);
    print(buffer);
    sprintf(buffer, "");
    print(buffer);

    HAL_GPIO_WritePin(AMS_OK_GPIO_Port, AMS_OK_Pin, flag_ams_ok ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_AIR_N_GPIO_Port, RELAY_AIR_N_Pin, state_air_n ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_AIR_P_GPIO_Port, RELAY_AIR_P_Pin, state_air_p ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_PRECHARGE_GPIO_Port, RELAY_PRECHARGE_Pin, state_precharge ? GPIO_PIN_SET : GPIO_PIN_RESET);

    if(1) {
        // printnl((char*)"State: ");
        // printValue(state);
        // printnl((char*)"State AIR+: ");
        // printValue(state_air_p);
        // printnl((char*)"State AIR-: ");
        // printValue(state_air_n);
        // printnl((char*)"Relee Prec: ");
        // printValue(state_precharge);
    }
}

/*********************************************************************************************************
 ** Function name:           parse_state
 ** Descriptions:            Function for analysing the data from the CAN
 *********************************************************************************************************/
void parse_state(CANMsg* data) {
    if (data->id == 0x600) {
        flag_start_button = data->buf[0];
        // printValue(flag_start_button);
    }

    if (data->id == 0x100) {
        CPU.DC_BUS = (int)(data->buf[1] << 8) + data->buf[0];  // This direction sends the voltage in DCBUS
    }

    uint32_t time = HAL_GetTick();
    bool flag_bms = false;

    for (int i = 0; i < BMS_N; i++) {
        flag_bms = BMS[i].parse(data->id, &data->buf[0], time);  // Checking if the message received is for BMS
        if (flag_bms) {
            i = BMS_N;
        }
    }

    if (!flag_bms) {
        if (CPU.parse(data->id, &data->buf[0], time)) {  // Checking if message is for CPU
        }

        if (data->id == 419385575) {  // If message from this direction received, it is because the charger is connected and the accu is for charging
            for (int i = 0; i < BMS_N; i++) {
                BMS[i].flag_charger = 1;
            }
            current.flag_charger = 1;
            flag_charger = 1;
            // Serial.print("Charger: ");
            // Serial.println(flag_charger);
        } else {
            // Serial.print("ID: ");
            // Serial.println(data->id);
        }
    }
}
