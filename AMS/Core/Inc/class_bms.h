// -----------------------------------------------------------------------------
// Author       :   Luis de la Barba & Javier R. Juliani
// Date         :   17/04/2020
// Adaptation   :   Juan Mata & Jaime Landa
// Date         :   03/2024
// Update       :   10/2025
// Author       :   Andrés Sánchez de Ágreda
// Name         :   class_cpu.h
// Description  :
// * This file is for defining the BMS class
// -----------------------------------------------------------------------------

#ifndef INC_CLASS_BMS_H_
#define INC_CLASS_BMS_H_

#include <stdint.h>
#include "main.h"
#include "ams_can_map.h"
#include "module_state_machine.h"
// Forward declaration
//enum STATE;

#define MAX_CELLS          24
#define NUM_TEMPS          38

#define TIME_LIM_RECV_VOLTS   500u
#define TIME_LIM_SEND_VOLTS   200u
#define TIME_LIM_PLOT_VOLTS     0u
#define TIME_LIM_RECV_TEMPS  1000u
#define TIME_LIM_SEND_TEMPS   250u
#define TIME_LIM_PLOT_TEMPS     0u

#define BALANCING_V 0

enum {
    BMS_OK = 0,
    BMS_ERROR_COMMUNICATION = 1
};

#define Temperatures_OK BMS_OK

extern "C" {
HAL_StatusTypeDef module_send_message_CAN2(uint32_t id, uint8_t* data, uint8_t length);
HAL_UART_StateTypeDef getUARTState(void);
void print(char *s);
void printnl(char *s);
}

STATE get_state(void);

class BMS_MOD {
public:
    BMS_MOD(uint32_t _ID, int _MAXV, int _MINV, int _MAXT,
            uint8_t _NUMCELLS, unsigned int _SHUNT, int _LAG_V, int _LAG_T);

    bool parse(uint32_t id, uint8_t *buf, uint32_t t);
    int query_voltage(uint32_t time, char *buffer);
    int query_temperature(uint32_t time, char *buffer);

    void voltage_info(char *buffer);
    void temperature_info(char *buffer);

    int return_error();
    int return_voltage_error() const { return error_volt; }
    int return_temperature_error() const { return error_temp; }

    uint32_t CANID = 0;
    int LIMIT_MAX_V = 0;
    int LIMIT_MIN_V = 0;
    int LIMIT_MAX_T = 0;
    uint8_t NUM_CELLS = 0;

    int MAX_V = 0, MIN_V = 0;
    int MAX_T = 0, MIN_T = 0;

    uint16_t cellVoltagemV[MAX_CELLS] = {0};
    uint8_t  cellTemperature[NUM_TEMPS] = {0};

    uint32_t voltage_acum = 0;  // Moved to public
    int flag_charger = 0;       // Added

private:
    friend void bms_publish_voltage_CAN2(BMS_MOD *self);
    friend void bms_publish_temperature_CAN2(BMS_MOD *self);

    unsigned int SHUNT_mOhm = 0;
    int error_volt = BMS_ERROR_COMMUNICATION;
    int error_temp = BMS_ERROR_COMMUNICATION;
    uint16_t flag_error_volt[MAX_CELLS] = {0};

    uint32_t time_lim_plotted_volts   = 0;
    uint32_t time_lim_sent_volts      = 0;
    uint32_t time_lim_received_volts  = 0;

    uint32_t time_lim_plotted_temps   = 0;
    uint32_t time_lim_sent_temps      = 0;
    uint32_t time_lim_received_temps  = 0;

    uint8_t  message_balancing[2]     = {0, 0};
    uint8_t  message_temperatures[2]  = {0, 0};
};

void bms_publish_voltage_CAN2(BMS_MOD *self);
void bms_publish_temperature_CAN2(BMS_MOD *self);
#endif /* INC_CLASS_BMS_H_ */
