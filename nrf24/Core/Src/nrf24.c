/*
  ***************************************************************************************************************
  ***************************************************************************************************************
  ***************************************************************************************************************

  File:		  NRF24L01.c
  Author:     ControllersTech.com
  Updated:    30th APRIL 2021

  ***************************************************************************************************************
  Copyright (C) 2017 ControllersTech.com

  This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
  of the GNU General Public License version 3 as published by the Free Software Foundation.
  This software library is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
  or indirectly by this software, read more about this on the GNU General Public License.

  ***************************************************************************************************************
*/


#include "stm32g4xx_hal.h" //Cambiar al H7 en main
#include "nrf24.h"
#include <string.h>
#include <stdio.h>
extern UART_HandleTypeDef hlpuart1;

extern SPI_HandleTypeDef hspi2;
#define NRF24_SPI &hspi2

#define NRF24_CE_PORT   GPIOC
#define NRF24_CE_PIN    GPIO_PIN_3

#define NRF24_CSN_PORT   GPIOC
#define NRF24_CSN_PIN    GPIO_PIN_2


void CS_Select (void)
{
	HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_RESET);
}

void CS_UnSelect (void)
{
	HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_SET);
}


void CE_Enable (void)
{
	HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_SET);
}

void CE_Disable (void)
{
	HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_RESET);
}



// write a single byte to the particular register
void nrf24_WriteReg (uint8_t Reg, uint8_t Data)
{
	uint8_t buf[2];
	buf[0] = Reg|1<<5;
	buf[1] = Data;

	// Pull the CS Pin LOW to select the device
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, buf, 2, 1000);

	// Pull the CS HIGH to release the device
	CS_UnSelect();
}

//write multiple bytes starting from a particular register
void nrf24_WriteRegMulti (uint8_t Reg, uint8_t *data, int size)
{
	uint8_t buf[2];
	buf[0] = Reg|1<<5;
//	buf[1] = Data;

	// Pull the CS Pin LOW to select the device
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, buf, 1, 100);
	HAL_SPI_Transmit(NRF24_SPI, data, size, 1000);

	// Pull the CS HIGH to release the device
	CS_UnSelect();
}


uint8_t nrf24_ReadReg (uint8_t Reg)
{
	uint8_t data=0;

	// Pull the CS Pin LOW to select the device
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, &Reg, 1, 100);
	HAL_SPI_Receive(NRF24_SPI, &data, 1, 100);

	// Pull the CS HIGH to release the device
	CS_UnSelect();

	return data;
}


/* Read multiple bytes from the register */
void nrf24_ReadReg_Multi (uint8_t Reg, uint8_t *data, int size)
{
	// Pull the CS Pin LOW to select the device
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, &Reg, 1, 100);
	HAL_SPI_Receive(NRF24_SPI, data, size, 1000);

	// Pull the CS HIGH to release the device
	CS_UnSelect();
}


// send the command to the NRF
void nrfsendCmd (uint8_t cmd)
{
	// Pull the CS Pin LOW to select the device
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 100);

	// Pull the CS HIGH to release the device
	CS_UnSelect();
}

void nrf24_reset(uint8_t REG)
{
	if (REG == STATUS)
	{
		nrf24_WriteReg(STATUS, 0x00);
	}

	else if (REG == FIFO_STATUS)
	{
		nrf24_WriteReg(FIFO_STATUS, 0x11);
	}

	else {
	nrf24_WriteReg(CONFIG, 0x08);
	nrf24_WriteReg(EN_AA, 0x3F);
	nrf24_WriteReg(EN_RXADDR, 0x03);
	nrf24_WriteReg(SETUP_AW, 0x03);
	nrf24_WriteReg(SETUP_RETR, 0x03);
	nrf24_WriteReg(RF_CH, 0x02);
	nrf24_WriteReg(RF_SETUP, 0x0E);
	nrf24_WriteReg(STATUS, 0x00);// este
	nrf24_WriteReg(OBSERVE_TX, 0x00);
	nrf24_WriteReg(CD, 0x00);// este
	uint8_t rx_addr_p0_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	nrf24_WriteRegMulti(RX_ADDR_P0, rx_addr_p0_def, 5);
	uint8_t rx_addr_p1_def[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};// este
	nrf24_WriteRegMulti(RX_ADDR_P1, rx_addr_p1_def, 5);
	nrf24_WriteReg(RX_ADDR_P2, 0xC3);
	nrf24_WriteReg(RX_ADDR_P3, 0xC4);
	nrf24_WriteReg(RX_ADDR_P4, 0xC5);
	nrf24_WriteReg(RX_ADDR_P5, 0xC6);
	uint8_t tx_addr_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	nrf24_WriteRegMulti(TX_ADDR, tx_addr_def, 5);
	nrf24_WriteReg(RX_PW_P0, 0);
	nrf24_WriteReg(RX_PW_P1, 0);
	nrf24_WriteReg(RX_PW_P2, 0);
	nrf24_WriteReg(RX_PW_P3, 0);
	nrf24_WriteReg(RX_PW_P4, 0);
	nrf24_WriteReg(RX_PW_P5, 0);
	nrf24_WriteReg(FIFO_STATUS, 0x11);
	nrf24_WriteReg(DYNPD, 0);
	nrf24_WriteReg(FEATURE, 0);
	}
}




void NRF24_Init (void)
{
  CE_Disable();

  nrf24_reset(0);

  // --- Hard match RX settings for the first bring-up ---
  nrf24_WriteReg(EN_AA,     0x00);   // NO Auto-ACK on any pipe
  nrf24_WriteReg(SETUP_RETR,0x00);   // NO retries
  nrf24_WriteReg(EN_RXADDR, 0x03);   // P0 & P1 enabled (ok)
  nrf24_WriteReg(SETUP_AW,  0x03);   // 5-byte addresses
  nrf24_WriteReg(RF_CH,     76);     // Channel 76 (0x4C)
  nrf24_WriteReg(RF_SETUP,  0x06);   // 1 Mbps, 0 dBm, LNA on

  nrf24_WriteReg(FEATURE,   0x00);   // dynamic payloads OFF
  nrf24_WriteReg(DYNPD,     0x00);   // fixed payload widths
  nrf24_WriteReg(FIFO_STATUS,0x11);  // reset FIFOs
  nrf24_WriteReg(STATUS,    0x70);   // clear IRQs

  // Leave CONFIG to TxMode (power up there)
  CE_Enable();
}



// set up the Tx mode

void NRF24_TxMode (uint8_t *Address, uint8_t channel)
{
  CE_Disable();

  nrf24_WriteReg (RF_CH, channel);        // use the argument (76 now)

  nrf24_WriteRegMulti(TX_ADDR, Address, 5);
  nrf24_WriteRegMulti(RX_ADDR_P0, Address, 5); // P0 will be the ACK return addr once we enable ACKs

  // Power up TX, PRIM_RX=0, EN_CRC=1, CRCO=1 (16-bit CRC)
  uint8_t cfg = (1<<1) | (1<<3) | (1<<2); // 0x0E
  nrf24_WriteReg(CONFIG, cfg);

  CE_Enable();
}



// transmit the data

uint8_t NRF24_Transmit (uint8_t *data)
{
    uint8_t cmd, status;
    uint32_t t0;

    // 1) CE LOW before loading the payload
    CE_Disable();

    // 2) Load TX FIFO
    CS_Select();
    cmd = W_TX_PAYLOAD;
    HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 100);
    HAL_SPI_Transmit(NRF24_SPI, data, 32, 1000);
    CS_UnSelect();

    // 3) Pulse CE HIGH >= ~10 us to start ShockBurst
    CE_Enable();
    for (volatile int i = 0; i < 300; i++) { __NOP(); }  // NOTE: __NOP() not __NOP__
    CE_Disable();

    // 4) Poll STATUS for TX_DS or MAX_RT (timeout ~5 ms)
    t0 = HAL_GetTick();
    do {
        status = nrf24_ReadReg(STATUS);
        if (status & (1<<5)) break; // TX_DS
        if (status & (1<<4)) break; // MAX_RT
    } while ((HAL_GetTick() - t0) < 5);

    // 5) Clear IRQ flags
    nrf24_WriteReg(STATUS, (1<<5) | (1<<4) | (1<<6));

    // 6) On MAX_RT, flush and print quick diag, then fail
    if (status & (1<<4)) {
        cmd = FLUSH_TX;
        nrfsendCmd(cmd);

        uint8_t ob = nrf24_ReadReg(OBSERVE_TX); // [PLOS_CNT | ARC_CNT]
        char msg[64];
        snprintf(msg, sizeof(msg),
                 "[TX] MAX_RT. STATUS=%02X OBSERVE_TX=%02X\r\n", status, ob);
        HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return 0;
    }

    return (status & (1<<5)) ? 1 : 0;
}




void NRF24_RxMode (uint8_t *Address, uint8_t channel)
{
	// disable the chip before configuring the device
	CE_Disable();

	nrf24_reset (STATUS);

	nrf24_WriteReg (RF_CH, channel);  // select the channel

	// select data pipe 2
	uint8_t en_rxaddr = nrf24_ReadReg(EN_RXADDR);
	en_rxaddr = en_rxaddr | (1<<2);
	nrf24_WriteReg (EN_RXADDR, en_rxaddr);

	/* We must write the address for Data Pipe 1, if we want to use any pipe from 2 to 5
	 * The Address from DATA Pipe 2 to Data Pipe 5 differs only in the LSB
	 * Their 4 MSB Bytes will still be same as Data Pipe 1
	 *
	 * For Eg->
	 * Pipe 1 ADDR = 0xAABBCCDD11
	 * Pipe 2 ADDR = 0xAABBCCDD22
	 * Pipe 3 ADDR = 0xAABBCCDD33
	 *
	 */
	nrf24_WriteRegMulti(RX_ADDR_P1, Address, 5);  // Write the Pipe1 address
	nrf24_WriteReg(RX_ADDR_P2, 0xEE);  // Write the Pipe2 LSB address

	nrf24_WriteReg (RX_PW_P2, 32);   // 32 bit payload size for pipe 2


	// power up the device in Rx mode
	uint8_t config = nrf24_ReadReg(CONFIG);
	config = config | (1<<1) | (1<<0);
	nrf24_WriteReg (CONFIG, config);

	// Enable the chip after configuring the device
	CE_Enable();
}


uint8_t isDataAvailable (int pipenum)
{
	uint8_t status = nrf24_ReadReg(STATUS);

	if ((status&(1<<6))&&(status&(pipenum<<1)))
	{

		nrf24_WriteReg(STATUS, (1<<6));

		return 1;
	}

	return 0;
}


void NRF24_Receive (uint8_t *data)
{
	uint8_t cmdtosend = 0;

	// select the device
	CS_Select();

	// payload command
	cmdtosend = R_RX_PAYLOAD;
	HAL_SPI_Transmit(NRF24_SPI, &cmdtosend, 1, 100);

	// Receive the payload
	HAL_SPI_Receive(NRF24_SPI, data, 32, 1000);

	// Unselect the device
	CS_UnSelect();

	HAL_Delay(1);

	cmdtosend = FLUSH_RX;
	nrfsendCmd(cmdtosend);
}



// Read all the Register data
void NRF24_ReadAll (uint8_t *data)
{
	for (int i=0; i<10; i++)
	{
		*(data+i) = nrf24_ReadReg(i);
	}

	nrf24_ReadReg_Multi(RX_ADDR_P0, (data+10), 5);

	nrf24_ReadReg_Multi(RX_ADDR_P1, (data+15), 5);

	*(data+20) = nrf24_ReadReg(RX_ADDR_P2);
	*(data+21) = nrf24_ReadReg(RX_ADDR_P3);
	*(data+22) = nrf24_ReadReg(RX_ADDR_P4);
	*(data+23) = nrf24_ReadReg(RX_ADDR_P5);

	nrf24_ReadReg_Multi(RX_ADDR_P0, (data+24), 5);

	for (int i=29; i<38; i++)
	{
		*(data+i) = nrf24_ReadReg(i-12);
	}

}

// --- put near other prototypes ---
static void dump_hex5(const char *name, uint8_t *v);

// --- add at the bottom of nrf24.c ---
extern UART_HandleTypeDef hlpuart1;        // for prints
static char dbg[64];
static void put(const char *s){ HAL_UART_Transmit(&hlpuart1,(uint8_t*)s,strlen(s),HAL_MAX_DELAY); }

static void hex1(const char *name, uint8_t v){
  snprintf(dbg, sizeof(dbg), "%s=%02X ", name, v);
  put(dbg);
}
static void dump_hex5(const char *name, uint8_t *v){
  snprintf(dbg, sizeof(dbg), "%s=%02X %02X %02X %02X %02X  ",
           name, v[0], v[1], v[2], v[3], v[4]);
  put(dbg);
}

void NRF24_Dump(void)
{
  uint8_t v, addr[5];

  v = nrf24_ReadReg(CONFIG);      hex1("CFG", v);
  v = nrf24_ReadReg(EN_AA);       hex1("EN_AA", v);
  v = nrf24_ReadReg(SETUP_RETR);  hex1("RETR", v);
  v = nrf24_ReadReg(RF_CH);       hex1("CH", v);
  v = nrf24_ReadReg(RF_SETUP);    hex1("RF", v);
  v = nrf24_ReadReg(FEATURE);     hex1("FEAT", v);
  v = nrf24_ReadReg(DYNPD);       hex1("DYNPD", v);

  nrf24_ReadReg_Multi(TX_ADDR, addr, 5);      dump_hex5("TX", addr);
  nrf24_ReadReg_Multi(RX_ADDR_P0, addr, 5);   dump_hex5("RX0", addr);

  v = nrf24_ReadReg(STATUS);      hex1("STAT", v);
  put("\r\n");
}










