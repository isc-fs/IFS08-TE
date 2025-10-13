// -----------------------------------------------------------------------------
// class_temperatures.cpp  (AMS temps over CAN2 only)
// - Requests temps at (baseID+20)
// - Expects replies at (baseID+21 .. baseID+25), each 8 temps (°C)
// - Republishes on CAN2:
//     * Summary: CAN2_ID_AMS_TEMP_SUM
//     * Raw blocks (8 temps each): CAN2_ID_AMS_TEMP_BLOCK0..4
//
// Notes:
// * DS18B20U MSOP sensors live behind temps boards; those boards quantize to
//   whole °C bytes already. We treat 0 as "missing/defective" for stats.
// * Channels come from different battery modules; we just expose a flat array
//   so the main board stays simple. The block framing is fixed (8/8/8/8/6).
// -----------------------------------------------------------------------------

#include "class_temperatures.h"
#include <string.h>
#include <stdio.h>

static inline uint8_t pad_or_val(uint8_t v){ return v ? v : 0xFF; }

Temperatures_MOD::Temperatures_MOD(uint32_t MODULEID, int _T_MAX, int _LAG) {
  baseID = MODULEID;
  LIMIT_MAX_T = _T_MAX;

  time_lim_plotted  = TIME_LIM_PLOT + _LAG;
  time_lim_sended   = 0u + _LAG;
  time_lim_received = TIME_LIM_RECV + _LAG;
}

void Temperatures_MOD::update_min_max_ignore_zero() {
  int maxT = -128, minT = 127, any = 0;
  for (int i=0;i<TEMPS_NUM_CHANNELS;i++){
    int t = cellTemperature[i];
    if (t==0) continue;         // ignore 0 (bad/missing probe)
    if (t>maxT) maxT=t;
    if (t<minT) minT=t;
    any=1;
  }
  MAX_T = any?maxT:0;
  MIN_T = any?minT:0;
  error = (MAX_T > LIMIT_MAX_T) ? Temperatures_ERROR_MAXIMUM_T : Temperatures_OK;
}

void Temperatures_MOD::publish_CAN2() {
  // Summary: MAX, MIN, AVG*10 (decicelsius), VALID_CNT
  int sum=0, valid=0;
  for (int i=0;i<TEMPS_NUM_CHANNELS;i++){
    uint8_t t = cellTemperature[i];
    if (t){ sum += t; valid++; }
  }
  int avg10 = valid ? (sum*10/valid) : 0;

  uint8_t sumf[8] = {
    (uint8_t)MAX_T,
    (uint8_t)MIN_T,
    (uint8_t)((avg10>>8)&0xFF),
    (uint8_t)(avg10 & 0xFF),
    (uint8_t)(valid & 0xFF),
    0,0,0
  };
  (void)module_send_message_CAN2(CAN2_ID_AMS_TEMP_SUM, sumf, 8);

  // Raw temps: 8 per frame => 5 frames cover 38 channels (last frame pads)
  uint8_t frame[8];
  for (int blk=0; blk<5; ++blk){
    int base = blk*8;
    memset(frame, 0xFF, 8);
    for (int k=0;k<8;k++){
      int idx = base + k;
      if (idx >= TEMPS_NUM_CHANNELS) break;
      frame[k] = pad_or_val(cellTemperature[idx]);  // 0xFF = unused/missing
    }
    (void)module_send_message_CAN2((uint32_t)(CAN2_ID_AMS_TEMP_BLOCK0 + blk), frame, 8);
  }
}

bool Temperatures_MOD::parse(uint32_t id, uint8_t *buf, uint32_t t_ms) {
  // Temps boards reply on CAN2 at baseID+21..baseID+25 (5 packets * 8 temps)
  if (id >= baseID+21 && id <= baseID+25) {
    time_lim_received = t_ms + TIME_LIM_RECV;

    const int pkt = (int)(id - (baseID+21)); // 0..4
    for (int i=0;i<8;i++){
      int pos = pkt*8 + i;
      if (pos >= TEMPS_NUM_CHANNELS) break;
      // DS18B20 boards already provide whole °C (uint8)
      cellTemperature[pos] = buf[i];
    }
    update_min_max_ignore_zero();
    return true;
  }
  return false;
}

int Temperatures_MOD::query(uint32_t time_ms, char *buffer) {
  // Periodically request temps from the boards
  if (time_ms > time_lim_sended) {
    time_lim_sended += TIME_LIM_SEND;
    (void)module_send_message_CAN2(baseID+20, message_temperatures, 2); // request

    // Also republish the latest snapshot for the main board on CAN2
    publish_CAN2();
  }

  // Timeout check
  if (time_ms > time_lim_received) {
    error = Temperatures_ERROR_COMMUNICATION;
  }

  // Optional UART dump
  if (TIME_LIM_PLOT > 0 && time_ms > time_lim_plotted) {
    time_lim_plotted += TIME_LIM_PLOT;
    info(buffer);
  }
  return error;
}

void Temperatures_MOD::info(char *buffer) {
  if (getUARTState() != HAL_UART_STATE_READY) return;

  // High-level header
  sprintf(buffer, "\n***********************\n"); print(buffer);
  sprintf(buffer, "     Temperatures\n"); print(buffer);
  sprintf(buffer, "***********************\n"); print(buffer);

  // Stats & IDs
  sprintf(buffer, " - ERR: %d  MAX=%d  MIN=%d  LIM=%d\n", error, MAX_T, MIN_T, LIMIT_MAX_T); print(buffer);
  sprintf(buffer, " - REQ on CAN2 ID: 0x%lX\n", (unsigned long)(baseID+20)); print(buffer);
  sprintf(buffer, " - RSP on CAN2 IDs: 0x%lX..0x%lX\n",
          (unsigned long)(baseID+21), (unsigned long)(baseID+25)); print(buffer);
  sprintf(buffer, " - PUB summary: 0x%03X, raw blocks: 0x%03X..0x%03X\n",
          CAN2_ID_AMS_TEMP_SUM, CAN2_ID_AMS_TEMP_BLOCK0, CAN2_ID_AMS_TEMP_BLOCK4); print(buffer);
  sprintf(buffer, "-----------------------\n"); print(buffer);

  // Print temps compactly (modules flattened)
  sprintf(buffer, "Temps [0..%d]: [", TEMPS_NUM_CHANNELS-1); printnl(buffer);
  for (int i=0;i<TEMPS_NUM_CHANNELS;i++){
    sprintf(buffer, (i==TEMPS_NUM_CHANNELS-1) ? "%d" : "%d,", (int)cellTemperature[i]); printnl(buffer);
  }
  print((char*)"]");
}
