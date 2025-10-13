// -----------------------------------------------------------------------------
// Author       :   Luis de la Barba & Javier R. Juliani
// Date         :   17/04/2020
// Adaptation   :   Juan Mata & Jaime Landa
// Date         :   03/2024
// Update		:	10/2025
// Author		: 	Andrés Sánchez de Ágreda
// Name         :   class_current.h
// Description  :
// * This file is for declaring the functions and variables of the class of the current
// -----------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>

#include "main.h"
#include "class_current.h"
#include "ams_can_map.h"

// ********************************************************************************************************
// **Function name:           Current_MOD
// **Descriptions:            Initialization function of current class
// **********************************************************************************************************
Current_MOD::Current_MOD(uint32_t ID, int _C_MAX)
{
    CANID = ID;
    C_MAX = _C_MAX;
    flag_error_current = 1;
}

// ********************************************************************************************************
// ** Function name:           query
// ** Descriptions:            Function that transforms the voltage measured by the sensor to its equivalent current
// **********************************************************************************************************
int Current_MOD::query(int time, char* buffer) {
    error = Current_OK;

    // 7-sample moving average (as you had)
    int s1 = readAnalogValue();
    int s2 = readAnalogValue();
    int s3 = readAnalogValue();
    int s4 = readAnalogValue();
    int s5 = readAnalogValue();
    int s6 = readAnalogValue();
    int s7 = readAnalogValue();

    Current = (s1+s2+s3+s4+s5+s6+s7)/7;
    // Calibration: raw ADC → amps (keep your linearization)
    Current = (int)(0.22727f * Current - 489.455f + 0.5f);

    // Alerts on CAN2
    if(Current > C_MAX*0.8 && Current < C_MAX) {
        uint8_t msg[1] = { (uint8_t)(Current & 0xFF) };
        (void)module_send_message_CAN2(CAN2_ID_AMS_CUR_WARN, msg, 1);
    }

    if (Current > C_MAX) {
        if (flag_error_current == 1) {
            uint8_t m[2] = { (uint8_t)(Current & 0xFF), (uint8_t)((Current>>8)&0xFF) };
            (void)module_send_message_CAN2(CAN2_ID_AMS_CUR_FAULT, m, 2);
        }
        flag_error_current++;
        if (flag_error_current >= 100) {
            // optional: latch or act in AMS
        }
    } else {
        if (flag_error_current != 0) {
            uint8_t z[2] = {0};
            for (int i = 0; i < 3; i++)
                (void)module_send_message_CAN2(CAN2_ID_AMS_CUR_NORMAL, z, 2);
        }
        flag_error_current = 0;
    }

    // Periodic TX on CAN2 (value in deci-amps, little-endian)
    if (time > time_lim_sended) {
        time_lim_sended += TIME_LIM_SEND;
        int16_t da = (int16_t)(Current * 10);   // 0.1 A per LSB
        uint8_t payload[2] = { (uint8_t)(da & 0xFF), (uint8_t)((da>>8)&0xFF) };
        (void)module_send_message_CAN2(CANID, payload, 2);
    }

    if (TIME_LIM_PLOT > 0 && time > time_lim_plotted) {
        time_lim_plotted += TIME_LIM_PLOT;
        info(buffer);
    }
    return error;
}

// ********************************************************************************************************
// **Function name:           info
// **Descriptions:            Function for printing the class data
// **********************************************************************************************************
void Current_MOD::info(char* buffer) {
    if (getUARTState() == HAL_UART_STATE_READY) { // Send the message just if there is a serial por connected
		sprintf(buffer, "\n***********************\n");
		print(buffer);
		sprintf(buffer, "         Current\n");
		print(buffer);
		sprintf(buffer, "***********************\n");
		print(buffer);
		sprintf(buffer, " - ERROR:     %i\n", error);
		print(buffer);
		sprintf(buffer, " - CAN ID:    0x%lx\n", CANID);
		print(buffer);
		sprintf(buffer, " - LIM C =    %i A\n", C_MAX);
		print(buffer);
		sprintf(buffer, "-----------------------\n");
		print(buffer);
		sprintf(buffer, "Current (A): %i\n", Current);
		print(buffer);

    }
}


