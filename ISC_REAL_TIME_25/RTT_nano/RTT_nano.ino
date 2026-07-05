/* rtt_nano_rx.ino — RF-Nano / nRF24L01+ receiver for STM32 fragmented snapshot
 *
 * On-air packet format (32 bytes, little-endian):
 *   [0]     magic      0xEC
 *   [1]     version    0x03
 *   [2]     frag_idx   0 … (FRAG_TOTAL-1)
 *   [3]     frag_tot   FRAG_TOTAL (5)
 *   [4..5]  seq        uint16_t LE  — snapshot sequence number
 *   [6]     kind       0x06 (kRadioKindSnapshot)
 *   [7]     reserved   0x00
 *   [8..31] data       24 bytes — slice of the 102-byte serialized snapshot
 *
 * For each valid fragment the sketch:
 *   - (optional, VERBOSE=1) prints a human-readable header decode + hex dump
 *   - emits a framed binary record on Serial: AA 55 20 <32 bytes> <xor>
 *
 * Invalid packets (wrong magic / version / kind / frag_tot) are dropped with
 * a VERBOSE warning and never forwarded.
 */

#include <SPI.h>
#include <RF24.h>
#include "printf.h"

// ------------- Radio config (must match TX) -------------
static const uint8_t PIN_CE  = 10;
static const uint8_t PIN_CSN = 9;
RF24 radio(PIN_CE, PIN_CSN);

static const uint64_t PIPE_ADDR = 0xE7E7E7E7E7ULL; // 5-byte address
static const uint8_t  CHANNEL   = 76;
static const uint8_t  PAYLOAD   = 32;               // fixed payload size

// ------------- Fragment protocol constants (must match STM32 TX) -------------
static const uint8_t MAGIC      = 0xECu;
static const uint8_t VERSION    = 0x03u;
static const uint8_t KIND_SNAP  = 0x06u;
static const uint8_t FRAG_TOTAL = 5u;
static const uint8_t HDR_SIZE   = 8u;
static const uint8_t DATA_SIZE  = 24u; // = PAYLOAD - HDR_SIZE

// ------------- Serial framing to host (serial.py) -------------
static const uint8_t SOF1 = 0xAAu;
static const uint8_t SOF2 = 0x55u;

// ------------- Verbosity -------------
#define VERBOSE 1  // 0: binary frames only, 1: also human-readable logs

// ------------- Globals -------------
uint8_t buf[PAYLOAD];

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
static bool validateHeader() {
    if (buf[0] != MAGIC) {
#if VERBOSE
        Serial.print(F("[DROP] bad magic: 0x")); Serial.println(buf[0], HEX);
#endif
        return false;
    }
    if (buf[1] != VERSION) {
#if VERBOSE
        Serial.print(F("[DROP] bad version: ")); Serial.println(buf[1]);
#endif
        return false;
    }
    if (buf[6] != KIND_SNAP) {
#if VERBOSE
        Serial.print(F("[DROP] unknown kind: ")); Serial.println(buf[6]);
#endif
        return false;
    }
    if (buf[3] != FRAG_TOTAL) {
#if VERBOSE
        Serial.print(F("[DROP] unexpected frag_tot: ")); Serial.println(buf[3]);
#endif
        return false;
    }
    if (buf[2] >= FRAG_TOTAL) {
#if VERBOSE
        Serial.print(F("[DROP] frag_idx out of range: ")); Serial.println(buf[2]);
#endif
        return false;
    }
    return true;
}

// ------------- Setup -------------
void setup() {
    Serial.begin(115200);
#if defined(USBCON) || defined(ARDUINO_AVR_LEONARDO)
    while (!Serial) {}
#endif

    printf_begin();

#if VERBOSE
    Serial.println(F("Init RF-NANO RX (fragmented snapshot mode)..."));
#endif

    if (!radio.begin()) {
#if VERBOSE
        Serial.println(F("ERR: radio.begin() failed (chip not detected)."));
#endif
    }

    // Mirror the STM32 TX configuration exactly
    radio.setAddressWidth(5);
    radio.setChannel(CHANNEL);
    radio.setAutoAck(false);
    radio.setDataRate(RF24_1MBPS);
    radio.setCRCLength(RF24_CRC_16);
    radio.setPALevel(RF24_PA_MAX);
    radio.disableDynamicPayloads();
    radio.setPayloadSize(PAYLOAD);

    radio.openReadingPipe(1, PIPE_ADDR);
    radio.startListening();

#if VERBOSE
    radio.printDetails();
    Serial.print(F("Chip conectado: "));
    Serial.println(radio.isChipConnected() ? F("SI") : F("NO"));
    Serial.println(F("RF-NANO RX listo."));
#endif
}

// ------------- Loop -------------
void loop() {
    if (!radio.available()) {
        return;
    }

    // Drain RX FIFO to keep up with burst traffic (5 fragments per snapshot)
    while (radio.available()) {
        radio.read(buf, PAYLOAD);

        if (!validateHeader()) {
            continue; // drop (VERBOSE warn already printed inside)
        }

        // Decode header fields for logging
        const uint8_t  frag_idx = buf[2];
        const uint8_t  frag_tot = buf[3];
        const uint16_t seq      = static_cast<uint16_t>(buf[4])
                                | (static_cast<uint16_t>(buf[5]) << 8);
        const uint8_t  kind     = buf[6];

#if VERBOSE
        // Full hex dump
        Serial.print(F("[RX] HEX: "));
        dumpHex(buf, PAYLOAD);
        Serial.println();

        // Decoded header
        Serial.print(F("[RX] magic=0x")); Serial.print(MAGIC, HEX);
        Serial.print(F(" ver="));         Serial.print(buf[1]);
        Serial.print(F(" frag="));        Serial.print(frag_idx);
        Serial.print('/');                 Serial.print(frag_tot);
        Serial.print(F(" seq="));         Serial.print(seq);
        Serial.print(F(" kind="));        Serial.println(kind);

        // Data slice offset in the original 102-byte snapshot
        Serial.print(F("[RX] data["));
        Serial.print(frag_idx * DATA_SIZE);
        Serial.print(F("..]: "));
        dumpHex(buf + HDR_SIZE, DATA_SIZE);
        Serial.println();
#else
        // Suppress unused-variable warnings when VERBOSE=0
        (void)frag_idx; (void)frag_tot; (void)seq; (void)kind;
#endif

        // ---------- Binary frame to host: AA 55 20 <32B> <XOR> ----------
        uint8_t xorv = 0u;
        for (uint8_t i = 0u; i < PAYLOAD; ++i) xorv ^= buf[i];

        Serial.write(SOF1);
        Serial.write(SOF2);
        Serial.write(PAYLOAD);       // 0x20
        Serial.write(buf, PAYLOAD);  // raw 32-byte fragment
        Serial.write(xorv);          // XOR checksum
    }
}