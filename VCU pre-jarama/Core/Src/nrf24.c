/*
  nrf24.c  — STM32H7 HAL driver (polling) for nRF24L01(+)
  Based on ControllersTech, cleaned for H723 + SPI2 + USART2 debug.
*/

#include "nrf24.h"
#include <string.h>
#include <stdio.h>

/* ==== Hardware bindings (edit only if your pins change) ==== */
extern SPI_HandleTypeDef  hspi1;
#define NRF24_SPI        (&hspi1)

extern UART_HandleTypeDef huart2;     /* ECU prints go through USART2 */
#define NRF24_UART       (&huart2)

/* CE / CSN lines */
#define NRF24_CE_PORT    GPIOC
#define NRF24_CE_PIN     GPIO_PIN_6
#define NRF24_CSN_PORT   GPIOG
#define NRF24_CSN_PIN    GPIO_PIN_3

/* ==== Local helpers ====================================================== */
static inline void CS_Select(void)   { HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_RESET); }
static inline void CS_UnSelect(void) { HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_SET);   }
static inline void CE_Enable(void)   { HAL_GPIO_WritePin(NRF24_CE_PORT,  NRF24_CE_PIN,  GPIO_PIN_SET);   }
static inline void CE_Disable(void)  { HAL_GPIO_WritePin(NRF24_CE_PORT,  NRF24_CE_PIN,  GPIO_PIN_RESET); }

static void uputs(const char *s){ HAL_UART_Transmit(NRF24_UART, (uint8_t*)s, strlen(s), HAL_MAX_DELAY); }

/* conservative timeouts for H7 @ polling */
#define T_SHORT   100U
#define T_LONG   1000U

/* ==== Low-level SPI access ============================================== */
void nrf24_WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(W_REGISTER | (reg & REGISTER_MASK));
    buf[1] = val;
    CS_Select();
    HAL_SPI_Transmit(NRF24_SPI, buf, 2, T_SHORT);
    CS_UnSelect();
}
void nrf24_WriteRegMulti(uint8_t reg, const uint8_t *data, int size)
{
    uint8_t cmd = (uint8_t)(W_REGISTER | (reg & REGISTER_MASK));
    CS_Select();
    HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, T_SHORT);
    HAL_SPI_Transmit(NRF24_SPI, (uint8_t*)data, size, T_LONG);
    CS_UnSelect();
}

 uint8_t nrf24_ReadReg(uint8_t reg)
{
    uint8_t cmd = (uint8_t)(R_REGISTER | (reg & REGISTER_MASK));
    uint8_t val = 0;
    CS_Select();
    HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, T_SHORT);
    HAL_SPI_Receive (NRF24_SPI, &val, 1, T_SHORT);
    CS_UnSelect();
    return val;
}

static void nrf24_ReadRegMulti(uint8_t reg, uint8_t *data, int size)
{
    uint8_t cmd = (uint8_t)(R_REGISTER | (reg & REGISTER_MASK));
    CS_Select();
    HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, T_SHORT);
    HAL_SPI_Receive (NRF24_SPI, data, size, T_LONG);
    CS_UnSelect();
}

static void nrf24_SendCmd(uint8_t cmd)
{
    CS_Select();
    HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, T_SHORT);
    CS_UnSelect();
}

/* ==== Soft reset (subset) =============================================== */
static void nrf24_reset(uint8_t reg)
{
    if (reg == STATUS) {
        /* clear IRQs (RX_DR | TX_DS | MAX_RT) */
        nrf24_WriteReg(STATUS, 0x70);
    }
    else if (reg == FIFO_STATUS) {
        nrf24_WriteReg(FIFO_STATUS, 0x11);
    }
    else {
        /* sensible defaults for bring-up */
        nrf24_WriteReg(CONFIG,      0x08);  /* CRC off, PWR_DOWN */
        nrf24_WriteReg(EN_AA,       0x00);  /* no Auto-ACK */
        nrf24_WriteReg(EN_RXADDR,   0x03);  /* P0,P1 enabled */
        nrf24_WriteReg(SETUP_AW,    0x03);  /* 5-byte addr */
        nrf24_WriteReg(SETUP_RETR,  0x00);  /* no retries */
        nrf24_WriteReg(RF_CH,       76);    /* ch=76 */
        nrf24_WriteReg(RF_SETUP,    0x06);  /* 1 Mbps, 0 dBm */
        nrf24_WriteReg(FEATURE,     0x00);  /* no dyn payloads */
        nrf24_WriteReg(DYNPD,       0x00);
        nrf24_WriteReg(FIFO_STATUS, 0x11);
        nrf24_WriteReg(STATUS,      0x70);  /* clear IRQs */

        uint8_t def0[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
        uint8_t def1[5] = {0xC2,0xC2,0xC2,0xC2,0xC2};
        nrf24_WriteRegMulti(RX_ADDR_P0, def0, 5);
        nrf24_WriteRegMulti(RX_ADDR_P1, def1, 5);
        nrf24_WriteReg    (RX_ADDR_P2, 0xC3);
        nrf24_WriteReg    (RX_ADDR_P3, 0xC4);
        nrf24_WriteReg    (RX_ADDR_P4, 0xC5);
        nrf24_WriteReg    (RX_ADDR_P5, 0xC6);
        nrf24_WriteRegMulti(TX_ADDR,    def0, 5);

        nrf24_WriteReg(RX_PW_P0, 0);
        nrf24_WriteReg(RX_PW_P1, 0);
        nrf24_WriteReg(RX_PW_P2, 0);
        nrf24_WriteReg(RX_PW_P3, 0);
        nrf24_WriteReg(RX_PW_P4, 0);
        nrf24_WriteReg(RX_PW_P5, 0);
    }
}

/* ==== Public API ========================================================= */

void NRF24_Init(void)
{
	CE_Disable();
	CS_UnSelect();
	HAL_Delay(1);

	// ACTIVATE 0x73 (needed by many BK24xx/nRF24 clones)
	uint8_t act[2] = { ACTIVATE, 0x73 };
	CS_Select(); HAL_SPI_Transmit(NRF24_SPI, act, 2, T_SHORT); CS_UnSelect();
	HAL_Delay(1);

    nrf24_reset(0);

    /* fixed settings for link bring-up */
    nrf24_WriteReg(EN_AA,        0x00);  /* NO ACK */
    nrf24_WriteReg(SETUP_RETR,   0x00);  /* NO retries */
    nrf24_WriteReg(EN_RXADDR,    0x03);  /* P0,P1 */
    nrf24_WriteReg(SETUP_AW,     0x03);  /* 5-byte */
    nrf24_WriteReg(RF_CH,        76);    /* channel 76 */
    nrf24_WriteReg(RF_SETUP,     0x06);  /* 1 Mbps, 0 dBm */
    nrf24_WriteReg(FEATURE,      0x00);
    nrf24_WriteReg(DYNPD,        0x00);
    nrf24_WriteReg(FIFO_STATUS,  0x11);
    nrf24_WriteReg(STATUS,       0x70);  /* clear IRQs */

    //CE_Enable();    /* power state will be set in TxMode/RxMode */
}

void NRF24_TxMode(uint8_t *Address, uint8_t channel)
{
    CE_Disable();

    nrf24_WriteReg(RF_CH, channel);
    nrf24_WriteRegMulti(TX_ADDR,    Address, 5);
    nrf24_WriteRegMulti(RX_ADDR_P0, Address, 5);  /* ACK return path if enabled later */

    /* CONFIG: PWR_UP(1) | EN_CRC(1) | CRCO(1=16bit) | PRIM_RX(0) */
    uint8_t cfg = (1<<1) | (1<<3) | (1<<2);   /* 0x0E */
    nrf24_WriteReg(CONFIG, cfg);
    HAL_Delay(3);

    //CE_Enable();
}


void NRF24_RxMode(uint8_t *Address, uint8_t channel)
{
    CE_Disable();

    nrf24_reset(STATUS);
    nrf24_WriteReg(RF_CH, channel);

    /* enable pipe 2 too (example) */
    uint8_t en = nrf24_ReadReg(EN_RXADDR);
    en |= (1<<2);
    nrf24_WriteReg(EN_RXADDR, en);

    /* Pipe1 carries the MSBytes for P2..P5 */
    nrf24_WriteRegMulti(RX_ADDR_P1, Address, 5);
    nrf24_WriteReg(RX_ADDR_P2, 0xEE);

    nrf24_WriteReg(RX_PW_P2, 32);

    /* CONFIG: PWR_UP | PRIM_RX */
    uint8_t cfg = nrf24_ReadReg(CONFIG);
    cfg |= (1<<1) | (1<<0);
    nrf24_WriteReg(CONFIG, cfg);

    CE_Enable();
}

uint8_t NRF24_IsDataAvailable(int pipenum)
{
    uint8_t st = nrf24_ReadReg(STATUS);
    if ((st & (1<<6)) && (st & (pipenum << 1))) {
        nrf24_WriteReg(STATUS, (1<<6));   /* clear RX_DR */
        return 1;
    }
    return 0;
}

void NRF24_Receive(uint8_t *data)
{
    uint8_t cmd = R_RX_PAYLOAD;

    CS_Select();
    HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, T_SHORT);
    HAL_SPI_Receive (NRF24_SPI, data, 32, T_LONG);
    CS_UnSelect();

    HAL_Delay(1);
    nrf24_SendCmd(FLUSH_RX);
}

/* Read a snapshot of interesting registers into 'data' (38 bytes). */
void NRF24_ReadAll(uint8_t *data)
{
    for (int i = 0; i < 10; i++)  data[i] = nrf24_ReadReg(i);
    nrf24_ReadRegMulti(RX_ADDR_P0, data + 10, 5);
    nrf24_ReadRegMulti(RX_ADDR_P1, data + 15, 5);

    data[20] = nrf24_ReadReg(RX_ADDR_P2);
    data[21] = nrf24_ReadReg(RX_ADDR_P3);
    data[22] = nrf24_ReadReg(RX_ADDR_P4);
    data[23] = nrf24_ReadReg(RX_ADDR_P5);

    nrf24_ReadRegMulti(RX_ADDR_P0, data + 24, 5);

    for (int i = 29; i < 38; i++) data[i] = nrf24_ReadReg(i - 12);
}

/* ===== Simple UART dump ================================================== */
static void hex1(const char *name, uint8_t v){
    char s[32];
    snprintf(s, sizeof(s), "%s=%02X ", name, v);
    uputs(s);
}

static void dump_hex5(const char *name, const uint8_t *v){
    char s[64];
    snprintf(s, sizeof(s), "%s=%02X %02X %02X %02X %02X  ",
            name, v[0], v[1], v[2], v[3], v[4]);
    uputs(s);
}

void NRF24_Dump(void)
{
    uint8_t v, addr[5];
    v = nrf24_ReadReg(CONFIG);     hex1("CFG",   v);
    v = nrf24_ReadReg(EN_AA);      hex1("EN_AA", v);
    v = nrf24_ReadReg(SETUP_RETR); hex1("RETR",  v);
    v = nrf24_ReadReg(RF_CH);      hex1("CH",    v);
    v = nrf24_ReadReg(RF_SETUP);   hex1("RF",    v);
    v = nrf24_ReadReg(FEATURE);    hex1("FEAT",  v);
    v = nrf24_ReadReg(DYNPD);      hex1("DYNPD", v);

    nrf24_ReadRegMulti(TX_ADDR,    addr, 5); dump_hex5("TX",  addr);
    nrf24_ReadRegMulti(RX_ADDR_P0, addr, 5); dump_hex5("RX0", addr);

    v = nrf24_ReadReg(STATUS);     hex1("STAT",  v);
    uputs("\r\n");
}
uint8_t NRF24_StatusNOP(void) {
    uint8_t cmd = NOP, st = 0x00;
    CS_Select();
    HAL_SPI_TransmitReceive(NRF24_SPI, &cmd, &st, 1, 100);
    CS_UnSelect();
    return st;
}

// nrf24.c (helpers; keep static if you already have similar)
static inline void csn_low(void){ HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_RESET); }
static inline void csn_high(void){ HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_SET); }
static inline void ce_low(void){ HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_RESET); }
static inline void ce_high(void){ HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_SET); }

uint8_t NRF24_ClearIRQs(void){         // clear RX_DR/TX_DS/MAX_RT
    nrf24_WriteReg(STATUS, 0x70);
    return nrf24_ReadReg(STATUS);
}

void NRF24_FlushTX(void){
    uint8_t cmd = 0xE1; csn_low(); HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 100); csn_high();
}
void NRF24_FlushRX(void){
    uint8_t cmd = 0xE2; csn_low(); HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 100); csn_high();
}

uint8_t NRF24_TxFIFOEmpty(void){
    return (nrf24_ReadReg(FIFO_STATUS) & 0x10) ? 1u : 0u; // TX_EMPTY bit
}

// Make sure your Init() does ACTIVATE (0x50 0x73) ONCE after power-up.
// You can fold this into NRF24_Init() if not already present.

// Robust TX that: clears flags, flushes if needed, loads payload, pulses CE, waits for TX_DS/MAX_RT
uint8_t NRF24_Transmit(const uint8_t *payload32)
{
    // 1) Recover if TX_FULL or not empty (stale data)
    uint8_t st = nrf24_ReadReg(STATUS);
    uint8_t fifo = nrf24_ReadReg(FIFO_STATUS);
    if ((st & 0x01) || !(fifo & 0x10)) {   // TX_FULL==1 or TX_EMPTY==0
        NRF24_ClearIRQs();
        NRF24_FlushTX();
    }

    // 2) Write payload (32 bytes expected by your app)
    uint8_t cmd = 0xA0; // W_TX_PAYLOAD
    csn_low();
    HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 100);
    HAL_SPI_Transmit(NRF24_SPI, (uint8_t*)payload32, 32, 100);
    csn_high();

    // 3) CE pulse (>10us). Keep it tiny; no HAL_Delay here.
    ce_high();
    // ~10–20us NOPs (H7 is fast: ~800 NOPs is plenty)
    for (volatile int i=0; i<20000; ++i) __NOP();
    ce_low();

    // 4) Wait a very short time for TX_DS or MAX_RT (fire-and-forget if EN_AA=0)
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 5) {     // small 5ms guard
        st = nrf24_ReadReg(STATUS);
        if (st & (1u<<5)) {                // TX_DS
            NRF24_ClearIRQs();
            return 1;
        }
        if (st & (1u<<4)) {                // MAX_RT
            NRF24_ClearIRQs();
            NRF24_FlushTX();
            return 0;
        }
        // If EN_AA is 0, TX_DS might not assert on some clones → still OK if FIFO empties
        if (NRF24_TxFIFOEmpty()) {
            NRF24_ClearIRQs();
            return 1;
        }
    }

    // Timeout → treat as fail, clean up
    NRF24_ClearIRQs();
    NRF24_FlushTX();
    return 0;
}

// Optional: the “manual one-shot” TX you tested, as a callable driver helper
void NRF24_ManualTxTest(void)
{
    NRF24_ClearIRQs(); NRF24_FlushTX(); NRF24_FlushRX();
    nrf24_WriteReg(EN_AA, 0x00);
    nrf24_WriteReg(SETUP_RETR, 0x00);

    uint8_t cmd = 0xA0; uint8_t pkt[32];
    for (int i=0;i<32;i++) pkt[i]=(uint8_t)(i+1);
    csn_low(); HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 100);
    HAL_SPI_Transmit(NRF24_SPI, pkt, 32, 100); csn_high();

    ce_high();
    for (volatile int i=0;i<800;i++) __NOP();
    ce_low();
}

