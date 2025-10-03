/* rtt_nano_rx.ino — RF-Nano / nRF24L01+ receiver for STM32 TelFrame
 *
 * For each 32-byte radio payload, it:
 *  - (optional) prints a readable line with id/seq and 7 floats
 *  - emits a framed binary record on Serial: AA 55 20 <32 bytes> <xor>
 *
 * TelFrame on the TX (little-endian):
 *   uint16_t id;   // bytes 0..1
 *   uint16_t seq;  // bytes 2..3
 *   float    v1..v7 (bytes 4..31)
 */

#include <SPI.h>
#include <RF24.h>
#include "printf.h"

// ------------ Config (must match TX) ------------
static const uint8_t PIN_CE  = 9;   // CE on D9
static const uint8_t PIN_CSN = 10;  // CSN (CS/SS) on D10
RF24 radio(PIN_CE, PIN_CSN);

static const uint64_t PIPE_ADDR = 0xE7E7E7E7E7ULL; // 5-byte address
static const uint8_t  CHANNEL   = 76;              // RF_CH
static const uint8_t  PAYLOAD   = 32;              // fixed payload size

// Serial framing to host tool
static const uint8_t SOF1 = 0xAA;
static const uint8_t SOF2 = 0x55;

// Verbosity & test helpers
#define VERBOSE 1            // 0: only binary frames, 1: also human-readable logs
#define SEND_TEST_PATTERN 0  // MUST be 0 in production
#define SEND_ACCEL_SWEEP  0  // MUST be 0 in production

// ------------ Globals ------------
uint8_t buf[PAYLOAD];

#if VERBOSE
static void dumpHex(const uint8_t* p, uint8_t n) {
  for (uint8_t i = 0; i < n; ++i) {
    if (i) Serial.print(' ');
    if (p[i] < 16) Serial.print('0');
    Serial.print(p[i], HEX);
  }
}
#endif  // VERBOSE

void setup() {
  Serial.begin(115200);
#if defined(USBCON) || defined(ARDUINO_AVR_LEONARDO)
  while (!Serial) {}
#endif

  // Short grace period so we don't spam the bootloader/host on reset
  delay(1500);

  printf_begin();

#if VERBOSE
  Serial.println(F("Init RF-NANO RX..."));
#endif

  if (!radio.begin()) {
#if VERBOSE
    Serial.println(F("ERR: radio.begin() failed (chip not detected)."));
#endif
  }

  // Mirror TX settings
  radio.setAddressWidth(5);
  radio.setChannel(CHANNEL);
  radio.setAutoAck(false);           // NO-ACK
  radio.setDataRate(RF24_1MBPS);
  radio.setCRCLength(RF24_CRC_16);
  radio.setPALevel(RF24_PA_MAX);

  radio.disableDynamicPayloads();    // fixed-size payloads
  radio.setPayloadSize(PAYLOAD);

  // Use pipe 1
  radio.openReadingPipe(1, PIPE_ADDR);
  radio.startListening();

#if VERBOSE
  radio.printDetails();
  Serial.print(F("Chip conectado: "));
  Serial.println(radio.isChipConnected() ? F("SI") : F("NO"));
  Serial.println(F("RF-NANO RX listo."));
#endif
}

void loop() {
  if (!radio.available()) {
    return; // nothing to read
  }

  // Drain RX FIFO to keep up at higher rates
  while (radio.available()) {
    radio.read(buf, PAYLOAD);

#if SEND_ACCEL_SWEEP
    // (kept for reference; MUST stay disabled in production)
    static float f[8] = {0};
    static int dir = +1;
    static unsigned long pt = 0;
    unsigned long t = millis();
    if (t - pt > 20) {
      pt = t;
      f[1] += dir * 0.02f;
      if (f[1] >= 1.0f) { f[1] = 1.0f; dir = -1; }
      if (f[1] <= 0.0f) { f[1] = 0.0f; dir = +1; }
      memcpy(buf, f, 32);
    }
#endif  // SEND_ACCEL_SWEEP

    // ---------- Decode STM32 TelFrame ----------
    // Little-endian on both sides (AVR is little-endian)
    uint16_t id  = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    uint16_t seq = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);

    float v[7];
    memcpy(v, buf + 4, 7 * sizeof(float));

#if VERBOSE
    // (A) HEX dump
    Serial.print(F("[RX] HEX: "));
    dumpHex(buf, PAYLOAD);
    Serial.println();

    // (B) Proper view
    Serial.print(F("[RX] ID=0x")); Serial.print(id, HEX);
    Serial.print(F(" seq="));      Serial.print(seq);
    Serial.print(F(" | v: "));
    for (int i = 0; i < 7; ++i) {
      Serial.print(v[i], 2);
      if (i != 6) Serial.print(F(", "));
    }
    Serial.println();
#endif  // VERBOSE

    // ---------- Binary frame to host: AA 55 20 <32B> <xor> ----------
    uint8_t xorv = 0;
    for (uint8_t i = 0; i < PAYLOAD; ++i) xorv ^= buf[i];

    Serial.write(SOF1);
    Serial.write(SOF2);
    Serial.write(PAYLOAD);      // 0x20
    Serial.write(buf, PAYLOAD); // raw 32 bytes
    Serial.write(xorv);         // XOR checksum
  }
}
