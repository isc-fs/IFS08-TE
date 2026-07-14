/* RTT_nano.ino — RF-Nano / nRF24L01+ receiver for IFS08-CE-ECU telemetry
 *
 * === TRANSMITTER PROTOCOL (feat/1, telemetry_task.cpp) ===
 *
 * ONE packet kind per 200 ms cycle (snapshot):
 *   SNAPSHOT  version=0x03  kind=6  frag_tot=5  — full vehicle state, 102 bytes total
 *
 * Common 32-byte on-air header (bytes 0-7):
 *   [0]     magic      0xEC
 *   [1]     version    0x03
 *   [2]     frag_idx   0 … (frag_tot-1)
 *   [3]     frag_tot   5
 *   [4..5]  seq        uint16_t LE
 *   [6]     kind       6 (snapshot)
 *   [7]     reserved   0x00
 *   [8..31] data       24 bytes of snapshot slice
 *
 * Snapshot wire layout (102 bytes total, split across 5 × 24-byte fragments):
 *   [0..3]   tick_ms         uint32_t LE
 *   [4..5]   seq             uint16_t LE
 *   [6]      start_button    uint8_t
 *   [7..8]   apps1_raw       uint16_t LE
 *   [9..10]  apps2_raw       uint16_t LE
 *   [11..12] brake_raw       uint16_t LE
 *   [13..14] torque_pct      uint16_t LE
 *   [15]     ev_2_3          uint8_t
 *   [16]     t11_8_9         uint8_t
 *   [17]     ctrl_state      uint8_t
 *   [18]     ok_precharge    uint8_t
 *   [19]     ams_fsm_state   uint8_t
 *   [20..21] v_cell_min_mV   uint16_t LE
 *   [22]     soc             uint8_t
 *   [23..32] vmin_modulo[5]  5 × uint16_t LE
 *   [33..42] vmax_modulo[5]  5 × uint16_t LE
 *   [43..44] corriente_accu  int16_t LE
 *   [45..46] corriente_dcdc  int16_t LE
 *   [47..48] temp_dcdc       int16_t LE
 *   [49..58] temp_max_mod[5] 5 × int16_t LE
 *   [59]     inv_state       uint8_t
 *   [60]     vconfig_ready   uint8_t
 *   [61]     inv_error       uint8_t
 *   [62..63] inv_dc_bus_V    uint16_t LE
 *   [64..65] inv_temp_motor1 uint16_t LE
 *   [66..67] inv_temp_pwrstg uint16_t LE
 *   [68..69] inv_temp_board  uint16_t LE
 *   [70..73] inv_rpm         int32_t LE
 *   [74..77] inv_speed_actual int32_t LE
 *   [78..81] inv_current_actual int32_t LE
 *   [82..101] (reserved / zero-padded)
 *
 * Radio config (matches nrf24.c in IFS08-CE-ECU exactly):
 *   Channel  : 76 (0x4C)
 *   Address  : ECU01  {0x45,0x43,0x55,0x30,0x31}
 *   Payload  : 32 bytes fixed
 *   Data rate: 1 Mbps
 *   CRC      : 8-bit  (CONFIG=EN_CRC|PWR_UP, no CRCO bit)
 *   PA level : 0 dBm  (RF_SETUP=0x06, PA_LOW)
 *   AutoAck  : disabled
 *   DynPD    : disabled
 *
 * VERBOSE 0: binary frames only  (use when connected to ISCmetrics host)
 * VERBOSE 1: binary + human log  (use when monitoring with a serial terminal)
 */

#include <SPI.h>
#include <RF24.h>
#include "printf.h"

// ------------- Radio config (must match TX) -------------
static const uint8_t PIN_CE  = 10;
static const uint8_t PIN_CSN = 9;
RF24 radio(PIN_CE, PIN_CSN);

// Pipe address: ECU01 = {'E','C','U','0','1'} = {0x45,0x43,0x55,0x30,0x31}
// RF24 stores addresses LSB-first: uint64_t LE = 0x3130554345ULL
static const uint64_t PIPE_ADDR = 0x3130554345ULL;
static const uint8_t  CHANNEL   = 76;
static const uint8_t  PAYLOAD   = 32;

// ------------- Protocol constants (must match telemetry_task.cpp) ------------
static const uint8_t MAGIC              = 0xECu;

// --- feat/1 (current) protocol ---
static const uint8_t VERSION_SNAPSHOT   = 0x03u;
static const uint8_t KIND_SNAPSHOT      = 6u;    // single snapshot kind
static const uint8_t FRAG_SNAPSHOT      = 5u;    // 5 fragments × 24 bytes = 120 >= 102

// --- Legacy feat/telemetry-port protocol (keep for backward compat) ---
static const uint8_t VERSION_LEGACY     = 0x02u;
static const uint8_t KIND_FAST          = 0x03u; // RF_FAST: 2 fragments
static const uint8_t KIND_SLOW          = 0x04u; // RF_SLOW: 5 fragments
static const uint8_t FRAG_FAST          = 2u;
static const uint8_t FRAG_SLOW          = 5u;

// ------------- Status protocol constants -------------
static const uint8_t KIND_STATUS          = 0x99u;
static const uint8_t STATUS_OK             = 0x00u;
static const uint8_t STATUS_NO_RADIO_CHIP  = 0x01u;
static const uint8_t STATUS_NO_TRANSMITTER = 0x02u;

// ------------- Serial framing to host (serial.py) -------------
static const uint8_t SOF1 = 0xAAu;
static const uint8_t SOF2 = 0x55u;

// ------------- Verbosity -------------
// 0 = binary frames only  (use this when connected to ISCmetrics host / Python UI)
// 1 = binary frames + human-readable log prints on the same serial port
//     (use this when monitoring with Arduino IDE serial monitor / PuTTY)
#define VERBOSE 0

// ------------- Globals -------------
uint8_t buf[PAYLOAD];
bool radio_ok = false;
unsigned long last_rx_time = 0;
unsigned long last_status_time = 0;

// ------------- Helpers -------------
#if VERBOSE
static void dumpHex(const uint8_t* p, uint8_t n) {
    for (uint8_t i = 0; i < n; ++i) {
        if (i) Serial.print(' ');
        if (p[i] < 0x10u) Serial.print('0');
        Serial.print(p[i], HEX);
    }
}
#endif

// Validate the 8-byte fragment header in buf[].
// Returns true if the packet should be forwarded.
// Handles both current (v0x03/kind=6) and legacy (v0x02/kind=0x03|0x04) protocols.
static bool validateHeader() {
    if (buf[0] != MAGIC) return false;

    uint8_t ver  = buf[1];
    uint8_t kind = buf[6];
    uint8_t ftot = buf[3];
    uint8_t fidx = buf[2];

    // --- Current protocol: version 0x03, kind=6, 5 fragments ---
    if (ver == VERSION_SNAPSHOT && kind == KIND_SNAPSHOT) {
        return (ftot == FRAG_SNAPSHOT) && (fidx < FRAG_SNAPSHOT);
    }

    // --- Legacy protocol: version 0x02, kind=FAST(3) or SLOW(4) ---
    if (ver == VERSION_LEGACY) {
        if (kind == KIND_FAST) {
            return (ftot == FRAG_FAST) && (fidx < FRAG_FAST);
        } else if (kind == KIND_SLOW) {
            return (ftot == FRAG_SLOW) && (fidx < FRAG_SLOW);
        }
    }

    return false;
}

// Send status packet over serial
static void sendStatusFrame(uint8_t status_code) {
    uint8_t status_packet[PAYLOAD];
    memset(status_packet, 0, PAYLOAD);
    status_packet[0] = MAGIC;
    status_packet[1] = VERSION_SNAPSHOT;   // always use current version in status frames
    status_packet[2] = 0;
    status_packet[3] = 1;
    status_packet[4] = 0;
    status_packet[5] = 0;
    status_packet[6] = KIND_STATUS;
    status_packet[7] = status_code;

    uint8_t xorv = 0u;
    for (uint8_t i = 0u; i < PAYLOAD; ++i) xorv ^= status_packet[i];

    Serial.write(SOF1);
    Serial.write(SOF2);
    Serial.write(PAYLOAD);
    Serial.write(status_packet, PAYLOAD);
    Serial.write(xorv);
    Serial.flush();

#if VERBOSE
    Serial.print(F("[STATUS] "));
    if      (status_code == STATUS_NO_RADIO_CHIP)  Serial.println(F("NO_RADIO_CHIP  (0x01) — nRF24L01 SPI not responding"));
    else if (status_code == STATUS_NO_TRANSMITTER) Serial.println(F("NO_TRANSMITTER (0x02) — no packets received in >2s"));
    else { Serial.print(F("UNKNOWN code=0x")); Serial.println(status_code, HEX); }
#endif
}

// ------------- Setup ---------------------------------------------------------
void setup() {
    Serial.begin(115200);
#if defined(USBCON) || defined(ARDUINO_AVR_LEONARDO)
    while (!Serial) {}
#endif

    printf_begin();

    radio_ok = radio.begin();
    if (radio_ok) {
        radio.setAddressWidth(5);
        radio.setChannel(CHANNEL);
        radio.setAutoAck(false);
        radio.setDataRate(RF24_1MBPS);
        radio.setCRCLength(RF24_CRC_8);      // TX CONFIG=EN_CRC|PWR_UP, no CRCO -> 8-bit
        radio.setPALevel(RF24_PA_LOW);       // TX RF_SETUP=0x06 -> 0 dBm
        radio.disableDynamicPayloads();
        radio.setPayloadSize(PAYLOAD);

        radio.openReadingPipe(1, PIPE_ADDR);
        radio.startListening();

#if VERBOSE
        Serial.println(F("[RADIO] nRF24L01 OK — ch76, 1Mbps, CRC8, 0dBm, addr=ECU01"));
        Serial.println(F("[PROTO] Accepting v0x03/kind=6 (snapshot) and v0x02/kind=3,4 (legacy)"));
        radio.printDetails();
#endif
    } else {
#if VERBOSE
        Serial.println(F("[RADIO] ERROR — nRF24L01 not found! Check SPI wiring (CE=10, CSN=9)."));
#endif
    }
    last_rx_time = millis();
}

// ------------- Loop ----------------------------------------------------------
void loop() {
    unsigned long now = millis();
    bool chip_connected = radio_ok && radio.isChipConnected();

    // Determine status
    uint8_t current_status = STATUS_OK;
    if (!chip_connected) {
        current_status = STATUS_NO_RADIO_CHIP;
    } else if (now - last_rx_time > 2000) {
        current_status = STATUS_NO_TRANSMITTER;
    }

    // Send status frame every 1000ms if not OK
    if (current_status != STATUS_OK) {
        if (now - last_status_time >= 1000) {
            sendStatusFrame(current_status);
            last_status_time = now;
        }
    }

    // Read packets if available
    if (chip_connected && radio.available()) {
        while (radio.available()) {
            radio.read(buf, PAYLOAD);

            if (!validateHeader()) {
#if VERBOSE
                // Print rejected header for debugging
                Serial.print(F("[REJECT] "));
                dumpHex(buf, 8);
                Serial.println();
#endif
                continue;
            }

            last_rx_time = millis();

            // Send binary frame: AA 55 20 <32B> <XOR>
            uint8_t xorv = 0u;
            for (uint8_t i = 0u; i < PAYLOAD; ++i) xorv ^= buf[i];

            Serial.write(SOF1);
            Serial.write(SOF2);
            Serial.write(PAYLOAD);       // 0x20
            Serial.write(buf, PAYLOAD);  // raw 32-byte fragment
            Serial.write(xorv);          // XOR checksum
            Serial.flush();

#if VERBOSE
            uint16_t seq = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
            uint8_t  ver  = buf[1];
            uint8_t  kind = buf[6];

            if (ver == VERSION_SNAPSHOT && kind == KIND_SNAPSHOT) {
                Serial.print(F("[SNAP]"));
            } else if (kind == KIND_FAST) {
                Serial.print(F("[FAST]"));
            } else {
                Serial.print(F("[SLOW]"));
            }
            Serial.print(F(" seq=")); Serial.print(seq);
            Serial.print(F(" frag=")); Serial.print(buf[2]);
            Serial.print(F("/")); Serial.println(buf[3]);
#endif
        }
    }
}