// -----------------------------------------------------------------------------
// Author       :   Luis de la Barba & Javier R. Juliani
// Date         :   17/04/2020
// Adaptation   :   Juan Mata & Jaime Landa
// Date         :   03/2024
// Update		:	10/2025
// Author		: 	Andrés Sánchez de Ágreda
// Name         :   class_bms.h
// Description  :
// * This file is for defining the BMS class
// -----------------------------------------------------------------------------

#include <class_bms.h>
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "class_bms.h"
#include "ams_can_map.h"

// externs from your project
extern HAL_StatusTypeDef module_send_message_CAN2(uint32_t id, uint8_t* data, uint8_t length);
extern HAL_UART_StateTypeDef getUARTState(void);
extern void print(char *s);
extern void printnl(char *s);

// --- helpers
static inline void put_u16_be(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v>>8); p[1] = (uint8_t)(v); }


///////// Dont ever touch this function if BMS are still ZEVA

/*********************************************************************************************************
 ** Function name:           BMS_MOD
 ** Descriptions:            Initialization function of teh class
 *********************************************************************************************************/

BMS_MOD::BMS_MOD(uint32_t _ID, int _MAXV, int _MINV, int _MAXT,
		uint8_t _NUMCELLS, unsigned int _SHUNT, int _LAG_V, int _LAG_T) {
	CANID = _ID;
	LIMIT_MAX_V = _MAXV;
	LIMIT_MIN_V = _MINV;
	LIMIT_MAX_T = _MAXT;
	NUM_CELLS = _NUMCELLS;

	uint32_t now = HAL_GetTick();

	time_lim_plotted_volts = now + _LAG_V;
	time_lim_sent_volts = now + _LAG_V;
	time_lim_received_volts = now + TIME_LIM_RECV_VOLTS + _LAG_V;

	time_lim_plotted_temps = now + _LAG_T;
	time_lim_sent_temps = now + _LAG_T;
	time_lim_received_temps = now + TIME_LIM_RECV_TEMPS + _LAG_T;
}

/*********************************************************************************************************
 ** Function name:           info
 ** Descriptions:            Function for printing the class data
 *********************************************************************************************************/
void BMS_MOD::voltage_info(char *buffer) {

	if (getUARTState() == HAL_UART_STATE_READY) { // Send the message just if there is a serial por connected
		print((char*) "\n***********************");
		print((char*) "         BMS");
		print((char*) "***********************");
		sprintf(buffer, " - ERROR:     %i", error_volt);
		print(buffer);
		sprintf(buffer, " - CAN ID:    0x%lx", CANID);
		print(buffer);
		sprintf(buffer, " - MAX V =    %i mV", MAX_V);
		print(buffer);
		sprintf(buffer, " - MIN V =    %i mV", MIN_V);
		print(buffer);
		print((char*) "-----------------------");
		sprintf(buffer, "VOLTS (mV): [%i", cellVoltagemV[0]);
		printnl(buffer);
		for (int i = 0; i < NUM_CELLS; i++) {
			sprintf(buffer, ", %i", cellVoltagemV[i]);
			printnl(buffer);
		}

		for (int i = 0; i < NUM_CELLS; i++) {
			voltage_acum += cellVoltagemV[i];
		}

		print((char*) "]");
		sprintf(buffer, " - V(max) = %i mV || V(min) = %i", MAX_V, MIN_V);
		print(buffer);
	}
	sprintf(buffer, "- BALANCING V = %i mV", BALANCING_V);
	print(buffer);

	sprintf(buffer, "- STACK VOLTAGE = %i V", voltage_acum / 1000);
	print(buffer);
}

// ********************************************************************************************************
// **Function name:           info
// **Descriptions:            Function for printing the class data
// **********************************************************************************************************
void BMS_MOD::temperature_info(char *buffer) {
	if (getUARTState() == HAL_UART_STATE_READY) //Send the message just if there is a serial por connected
	{
		print((char*) "\n***********************");
		print((char*) "     Temperatures");
		print((char*) "***********************");
		sprintf(buffer, " - ERROR:     %i", error_temp);
		print(buffer);
		sprintf(buffer, " - CAN ID:    0x%lx", CANID + 20);
		print(buffer);
		sprintf(buffer, " - MAX T =    %i ºC", MAX_T);
		print(buffer);
		sprintf(buffer, " - MIN T =    %i ºC", MIN_T);
		print(buffer);
		sprintf(buffer, " - LIM T =    %i ºC", LIMIT_MAX_T);
		print((char*) "-----------------------");
		sprintf(buffer, "Temperatures (ºC): [%i", cellTemperature[0]);
		printnl(buffer);
		for (int i = 1; i < 38; i++) {
			sprintf(buffer, ", %i", cellTemperature[i]);
			printnl(buffer);
		}
		print((char*) "]");

	}
}


/*********************************************************************************************************
 ** Function name:           parse
 ** Descriptions:            Function for parsing the received data via CAN protocol
 *********************************************************************************************************/
bool BMS_MOD::parse(uint32_t id, uint8_t *buf, uint32_t t) {
    if (id > CANID && id < CANID + 30) {
        int m = id % CANID;
        int pos = 0;

        if (m >= 1 && m <= 5) {                   // VOLTAGE packets
            time_lim_received_volts = t + TIME_LIM_RECV_VOLTS;

            for (int i = 0; i < 4; i++) {
                pos = (m - 1) * 4 + i;
                if (pos >= 19) break;
                cellVoltagemV[pos] = (buf[2 * i] << 8) | buf[2 * i + 1];

                if ((cellVoltagemV[pos] > LIMIT_MAX_V) && pos < NUM_CELLS) {
                    flag_error_volt[pos]++;
                }
            }

            MAX_V = cellVoltagemV[0];
            MIN_V = cellVoltagemV[0];
            for (int i = 1; i < 19; i++) {
                if (cellVoltagemV[i] > MAX_V) MAX_V = cellVoltagemV[i];
                else if (cellVoltagemV[i] < MIN_V) MIN_V = cellVoltagemV[i];
            }
            return true;

        } else if (m >= 21 && m <= 25) {          // TEMPERATURE packets
            time_lim_received_temps = t + TIME_LIM_RECV_TEMPS;

            // REMOVED: if(flag_charger==1) mirror to CAN1   <-- NOT ANYMORE

            for (int i = 0; i < 8; i++) {
                pos = (m - 21) * 8 + i;
                if (pos >= 38) break;
                cellTemperature[pos] = buf[i];
            }

            MAX_T = cellTemperature[0];
            MIN_T = cellTemperature[0];
            for (int i = 0; i < 38; i++) {
                if (cellTemperature[i] > MAX_T) MAX_T = cellTemperature[i];
                else if (cellTemperature[i] < MIN_T) MIN_T = cellTemperature[i];
            }
            return true;
        }
    }
    return false;
}

/*********************************************************************************************************
 ** Function name:           return_error
 ** Descriptions:            Function for returning the state of the BMS
 *********************************************************************************************************/
int BMS_MOD::return_error() {
	return error_volt;
}

/*********************************************************************************************************
 ** Function name:           query_voltage
 ** Descriptions:            Function to check if i need to send a message new message and the received messages interval are within the limits
 *********************************************************************************************************/
int BMS_MOD::query_voltage(uint32_t time, char *buffer) {
    // balancing command (unchanged behavior)
    if (get_state() == charge) {
        // leave commented to disable balancing during drive
        // message_balancing[0] = (BALANCING_V >> 8) & 0xFF;
        // message_balancing[1] = BALANCING_V & 0xFF;
    } else {
        message_balancing[0] = 0;
        message_balancing[1] = 0;
    }

    if (time > time_lim_sent_volts) {
        time_lim_sent_volts += TIME_LIM_SEND_VOLTS;
        if (CANID != 0x00) {
            if (module_send_message_CAN2(CANID, message_balancing, 2) != HAL_OK) {
                error_volt = BMS_ERROR_COMMUNICATION;
            }
        }
        // Also publish latest values on CAN2 for the main-board
        bms_publish_voltage_CAN2(this);
    }

    if (time > time_lim_received_volts) error_volt = BMS_ERROR_COMMUNICATION;
    else error_volt = BMS_OK;

    if (TIME_LIM_PLOT_VOLTS > 0 && time > time_lim_plotted_volts) {
        time_lim_plotted_volts += TIME_LIM_PLOT_VOLTS;
        voltage_info(buffer);
    }
    return error_volt;
}


/*********************************************************************************************************
 ** Function name:           query_temperature
 ** Descriptions:            Function to check if i need to send a message new message and the received messages interval are within the limits
 *********************************************************************************************************/

int BMS_MOD::query_temperature(uint32_t time, char *buffer) {
    if (time > time_lim_sent_temps) {
        time_lim_sent_temps += TIME_LIM_SEND_TEMPS;
        (void)module_send_message_CAN2(CANID + 20, message_temperatures, 2);
        // publish latest temps snapshot for main-board
        bms_publish_temperature_CAN2(this);
    }

    if (TIME_LIM_PLOT_TEMPS > 0 && time > time_lim_plotted_temps) {
        time_lim_plotted_temps += TIME_LIM_PLOT_TEMPS;
        temperature_info(buffer);
    }
    return error_temp;
}

// ---- NEW: publishers on CAN2 ------------------------------------------------
static void bms_publish_voltage_CAN2(BMS_MOD *self) {
    // Summary
    uint32_t stack_mV = 0;
    for (int i = 0; i < self->NUM_CELLS; ++i) stack_mV += self->cellVoltagemV[i];

    uint8_t sum[8] = {0};
    put_u16_be(&sum[0], (uint16_t)self->MAX_V);
    put_u16_be(&sum[2], (uint16_t)self->MIN_V);
    sum[4] = (uint8_t)(stack_mV >> 24);
    sum[5] = (uint8_t)(stack_mV >> 16);
    sum[6] = (uint8_t)(stack_mV >> 8);
    sum[7] = (uint8_t)(stack_mV);
    module_send_message_CAN2(CAN2_ID_AMS_VOLT_SUM, sum, 8);

    // Raw cells: 4 per frame
    const uint16_t *cells = (const uint16_t*)self->cellVoltagemV;
    uint8_t frame[8];
    for (int blk = 0; blk < 5; ++blk) {
        int base = blk*4;
        memset(frame, 0xFF, 8);
        for (int k = 0; k < 4; ++k) {
            int idx = base + k;
            if (idx >= self->NUM_CELLS) break;
            put_u16_be(&frame[k*2], (uint16_t)self->cellVoltagemV[idx]);
        }
        module_send_message_CAN2(CAN2_ID_AMS_VOLT_BLOCK0 + blk, frame, 8);
    }
}

static void bms_publish_temperature_CAN2(BMS_MOD *self) {
    // Summary: MAX, MIN, AVG*10 (decicelsius), valid count (ignoring zeros)
    int maxT = -128, minT = 127, sum = 0, valid = 0;
    for (int i = 0; i < 38; ++i) {
        int t = self->cellTemperature[i];
        if (t == 0) continue;    // treat 0 as “missing/defective probe”
        if (t > maxT) maxT = t;
        if (t < minT) minT = t;
        sum += t; valid++;
    }
    if (valid == 0) { maxT = 0; minT = 0; }

    uint8_t sumf[8] = {0};
    sumf[0] = (uint8_t)maxT;
    sumf[1] = (uint8_t)minT;
    int avg10 = (valid ? (sum * 10 / valid) : 0);
    sumf[2] = (uint8_t)(avg10 >> 8);
    sumf[3] = (uint8_t)(avg10);
    sumf[4] = (uint8_t)(valid & 0xFF);
    sumf[5] = sumf[6] = sumf[7] = 0;
    module_send_message_CAN2(CAN2_ID_AMS_TEMP_SUM, sumf, 8);

    // Raw temps: 8 per frame
    uint8_t frame[8];
    for (int blk = 0; blk < 5; ++blk) {
        int base = blk*8;
        memset(frame, 0xFF, 8);
        for (int k = 0; k < 8; ++k) {
            int idx = base + k;
            if (idx >= 38) break;
            frame[k] = self->cellTemperature[idx] ? self->cellTemperature[idx] : 0xFF;
        }
        module_send_message_CAN2(CAN2_ID_AMS_TEMP_BLOCK0 + blk, frame, 8);
    }
}

