// -----------------------------------------------------------------------------
// Author       :   Luis de la Barba & Javier R. Juliani
// Date         :   17/04/2020
// Adaptation   :   Juan Mata & Jaime Landa
// Date         :   03/2024
// Update		:	10/2025
// Author		: 	Andrés Sánchez de Ágreda
// Name         :   class_cpu.h
// Description  :
// * This file is for defining the class of the connection protocols with putside
// -----------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>

#include "main.h"
#include "class_cpu.h"
#include "ams_can_map.h"



extern HAL_StatusTypeDef module_send_message_CAN2(uint32_t id, uint8_t* data, uint8_t length);
extern HAL_UART_StateTypeDef getUARTState(void);
extern void print(char *s);

// ********************************************************************************************************
// **Function name:           CPU_MOD
// **Descriptions:            Initialization function of teh class
// ********************************************************************************************************
CPU_MOD::CPU_MOD(uint32_t _ID_send, uint32_t _ID_recv, int _LAG) { /////////////////////////Habia un _LAG=0
    CANID_send = _ID_send;
    CANID_recv = _ID_recv;

    time_lim_plotted += _LAG;
    time_lim_sended += _LAG;
    time_lim_received += _LAG;
}

// ********************************************************************************************************
// **Function name:           info
// **Descriptions:            Function for printing the class data
// ********************************************************************************************************
void CPU_MOD::info(char* buffer) {
    if (getUARTState() == HAL_UART_STATE_READY) { // Send the message just if there is a serial por connected
        sprintf(buffer, "\n***********************\n");
        print(buffer);
        sprintf(buffer, "         CPU\n");
        print(buffer);
        sprintf(buffer, "***********************\n");
        print(buffer);
        sprintf(buffer, " - ERROR:       %i\n", error);
        print(buffer);
        sprintf(buffer, " - CAN ID_send: 0x%lx\n", CANID_send);
        print(buffer);
        sprintf(buffer, " - CAN ID_recv: 0x%lx\n", CANID_recv);
        print(buffer);
        sprintf(buffer, " - DC BUS:      %i V\n", DC_BUS);
        print(buffer);
        sprintf(buffer, " - BAT BUS:	 %i V\n", voltage_acum/1000);
        print(buffer);
        sprintf(buffer, " - DC STATE:    %i\n", current_state);
        print(buffer);
        sprintf(buffer, "-----------------------\n");
        print(buffer);
    }
}

// ********************************************************************************************************
// **Function name:           parse
// **Descriptions:            Function for parsing the received data via CAN protocl
// ********************************************************************************************************
bool CPU_MOD::parse(uint32_t id, uint8_t* buf, uint32_t t) {
    if (id == 0x100) {
        error = CPU_OK;
        time_lim_received = t + TIME_LIM_RECV;
        DC_BUS = (int)((buf[1]<<8)|buf[0]);
        if (DC_BUS > 280) {
            error = CPU_BUS_LINE_OK;
            // <<<<<< was CAN1; now CAN2 only
            (void)module_send_message_CAN2(CANID_send, currentState, 1);
        }
        return true;
    }
    return false;
}

// ********************************************************************************************************
// **Function name:           return_error
// **Descriptions:            Function for returning the state of the BMS
// ********************************************************************************************************
int CPU_MOD::return_error() {
    return error;
}

// ********************************************************************************************************
// ** Function name:           query
// ** Descriptions:            Function to check if i need to send a mesage new mesage and the received mesajes interval are within the limits
// ********************************************************************************************************
int CPU_MOD::query(uint32_t time, char* buffer) {
    if (time > time_lim_sended) {
        time_lim_sended += TIME_LIM_SEND;
    }
    if (TIME_LIM_PLOT > 0 && time > time_lim_plotted) {
        time_lim_plotted += TIME_LIM_PLOT;
        // info(buffer);
    }
    return error;
}

void CPU_MOD::updateState(int s) {
    currentState[0] = s & 0xFF;
    current_state = s;
}

