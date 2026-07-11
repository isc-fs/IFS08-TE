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
 *   - emits a framed binary record on Serial: AA 55 20 <32 bytes> <xor>
 *
 * If the radio chip is disconnected or the connection is lost (no packets for 2s),
 * it sends a structured 1 Hz binary status update frame: AA 55 20 <status payload> <xor>
 * (kind = 0x99, status_code = 0x01 [no radio chip] | 0x02 [no transmitter]).
 * This prevents serial link spamming and high host CPU overhead.
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

// ------------- Status protocol constants -------------
static const uint8_t KIND_STATUS          = 0x99u;
static const uint8_t STATUS_OK             = 0x00u;
static const uint8_t STATUS_NO_RADIO_CHIP  = 0x01u;
static const uint8_t STATUS_NO_TRANSMITTER = 0x02u;

// ------------- Serial framing to host (serial.py) -------------
static const uint8_t SOF1 = 0xAAu;
static const uint8_t SOF2 = 0x55u;

// ------------- Verbosity -------------
#define VERBOSE 0  // 0: binary frames only, 1: human-readable print logs (slow)

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
static bool validateHeader() {
    if (buf[0] != MAGIC) {
        return false;
    }
    if (buf[1] != VERSION) {
        return false;
    }
    if (buf[6] != KIND_SNAP) {
        return false;
    }
    if (buf[3] != FRAG_TOTAL) {
        return false;
    }
    if (buf[2] >= FRAG_TOTAL) {
        return false;
    }
    return true;
}

// Send status packet over serial
static void sendStatusFrame(uint8_t status_code) {
    uint8_t status_packet[PAYLOAD];
    memset(status_packet, 0, PAYLOAD);
    status_packet[0] = MAGIC;
    status_packet[1] = VERSION;
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
}

// ------------- Setup -------------
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
        radio.setCRCLength(RF24_CRC_16);
        radio.setPALevel(RF24_PA_MAX);
        radio.disableDynamicPayloads();
        radio.setPayloadSize(PAYLOAD);

        radio.openReadingPipe(1, PIPE_ADDR);
        radio.startListening();
    }
    last_rx_time = millis();
}

// ------------- Loop -------------
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
                continue; // drop
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
        }
    }
}