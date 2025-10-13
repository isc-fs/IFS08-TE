// -----------------------------------------------------------------------------
// Author       : Luis de la Barba & Javier R. Juliani
// Date         : 17/04/2020
// Adaptation   : Juan Mata, Jaime Landa & Andrés Sánchez
// Update       : 10/2025 (CAN2-only, raw+summary TX, zero-ignoring stats)
// Name         : class_temperatures.h
// Description  : Temperatures class (AMS <-> temps boards). Requests over CAN2,
//                 parses replies, republishes summary + raw on CAN2.
//                 DS18B20 sensors on temps boards provide whole °C bytes.
//                 38 channels across multiple battery modules.
// -----------------------------------------------------------------------------

#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- External symbols provided elsewhere (e.g., main.c) ---------------------
typedef enum { HAL_UART_STATE_RESET=0x00U, HAL_UART_STATE_READY=0x20U } HAL_UART_StateTypeDef;
typedef enum { HAL_OK=0x00U, HAL_ERROR=0x01U, HAL_BUSY=0x02U, HAL_TIMEOUT=0x03U } HAL_StatusTypeDef;

HAL_UART_StateTypeDef getUARTState(void);
void print(char *uart_buffer);
void printnl(char *uart_buffer);
HAL_StatusTypeDef module_send_message_CAN2(uint32_t id, uint8_t* data, uint8_t length);

#ifdef __cplusplus
}
#endif

// --------------------- Configurable compile-time defaults ---------------------
#ifndef TEMPS_NUM_CHANNELS
#define TEMPS_NUM_CHANNELS 38
#endif

#ifndef TIME_LIM_SEND
#define TIME_LIM_SEND 100u      // ms: request period to temps boards
#endif

#ifndef TIME_LIM_RECV
#define TIME_LIM_RECV 300u      // ms: reception timeout
#endif

#ifndef TIME_LIM_PLOT
#define TIME_LIM_PLOT 1000u     // ms: UART print period; set 0 to disable
#endif

// ----- Error codes (keep legacy names for compatibility) ---------------------
#ifndef TEMPERATURES_ERROR_DEFS
#define TEMPERATURES_ERROR_DEFS
#define Temperatures_OK                    0
#define Temperatures_ERROR_COMMUNICATION   1
#define Temperatures_ERROR_MAXIMUM_T       2
#endif

// ---------------------- CAN2 ID map (override if you like) -------------------
// We republish temperatures on CAN2 at fixed IDs so the main board is simple.
// Summary: MAX, MIN, AVG*10 (decicelsius), VALID_CNT
// Raw: 5 blocks * 8 temps (°C) -> 38 total; pad with 0xFF.
#ifndef CAN2_ID_AMS_TEMP_SUM
#define CAN2_ID_AMS_TEMP_SUM      0x208
#endif
#ifndef CAN2_ID_AMS_TEMP_BLOCK0
#define CAN2_ID_AMS_TEMP_BLOCK0   0x209
#define CAN2_ID_AMS_TEMP_BLOCK1   0x20A
#define CAN2_ID_AMS_TEMP_BLOCK2   0x20B
#define CAN2_ID_AMS_TEMP_BLOCK3   0x20C
#define CAN2_ID_AMS_TEMP_BLOCK4   0x20D
#endif

// -----------------------------------------------------------------------------
// Class
// -----------------------------------------------------------------------------
class Temperatures_MOD {
public:
  // MODULEID: base ID of temps system (we send request at MODULEID+20,
  //           boards reply at MODULEID+21..+25).
  // _T_MAX  : °C limit to flag overtemp (e.g., 65).
  // _LAG    : ms offset to stagger timers vs. other modules.
  Temperatures_MOD(uint32_t MODULEID, int _T_MAX, int _LAG = 0);

  // Feed each received CAN2 frame here.
  bool parse(uint32_t id, uint8_t *buf, uint32_t t_ms);

  // Call periodically from your main loop / tick.
  int  query(uint32_t time_ms, char *buffer_for_printf);

  // Optional UART debug
  void info(char *buffer_for_printf);

  // Accessors
  inline int      get_max()   const { return MAX_T; }
  inline int      get_min()   const { return MIN_T; }
  inline int      get_limit() const { return LIMIT_MAX_T; }
  inline uint32_t get_id()    const { return baseID; }
  inline int      get_error() const { return error; }

private:
  // Config
  uint32_t baseID       = 0;   // request at baseID+20; replies baseID+21..+25
  int      LIMIT_MAX_T  = 65;  // °C threshold

  // State
  int   MAX_T = 0;
  int   MIN_T = 0;
  int   error = Temperatures_OK;

  uint32_t time_lim_plotted  = 0;
  uint32_t time_lim_sended   = 0;
  uint32_t time_lim_received = 0;

  // Data (°C, 0 = missing/defective; we ignore zeros in stats)
  uint8_t cellTemperature[TEMPS_NUM_CHANNELS] = {0};

  // Outgoing request payload (adjust to your temps-board protocol if needed)
  uint8_t message_temperatures[2] = { 0x01, 0x00 }; // cmd + arg (example)

  // Helpers
  void update_min_max_ignore_zero();
  void publish_CAN2(); // summary + raw blocks
};
