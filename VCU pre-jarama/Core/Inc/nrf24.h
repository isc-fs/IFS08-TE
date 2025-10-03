/*
  ***************************************************************************************************************
  ***************************************************************************************************************
  ***************************************************************************************************************

  File:		  NRF24L01.h
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

#ifndef INC_NRF24L01_H_
#define INC_NRF24L01_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>


// --- NRF24 pins (match board) ---
#define NRF24_CE_PORT    GPIOC
#define NRF24_CE_PIN     GPIO_PIN_6

#define NRF24_CSN_PORT   GPIOG
#define NRF24_CSN_PIN    GPIO_PIN_3

#ifdef __cplusplus
extern "C" {
#endif

/* ===== Public API ===== */
void    NRF24_Init(void);
void    NRF24_TxMode(uint8_t *addr5, uint8_t channel);
uint8_t NRF24_Transmit(const uint8_t *payload32);   /* payload must be 32 bytes */

void    NRF24_RxMode(uint8_t *addr5, uint8_t channel);
uint8_t NRF24_IsDataAvailable(int pipe);      /* 1 if data pending */
void    NRF24_Receive(uint8_t *payload32);    /* reads 32 bytes */

void    NRF24_ReadAll(uint8_t *out);          /* debug snapshot */
/* Public low-level accessors (needed by main.c and debug) */
uint8_t nrf24_ReadReg(uint8_t Reg);
void    nrf24_WriteReg(uint8_t Reg, uint8_t Data);
void    nrf24_WriteRegMulti(uint8_t Reg, const uint8_t *data, int size);
void    NRF24_Dump(void);
uint8_t NRF24_StatusNOP(void);   // read STATUS via NOP (SPI sanity check)

/* prints a short dump */

/* ===== Register Map ===== */
#define CONFIG       0x00
#define EN_AA        0x01
#define EN_RXADDR    0x02
#define SETUP_AW     0x03
#define SETUP_RETR   0x04
#define RF_CH        0x05
#define RF_SETUP     0x06
#define STATUS       0x07
#define OBSERVE_TX   0x08
#define CD           0x09
#define RX_ADDR_P0   0x0A
#define RX_ADDR_P1   0x0B
#define RX_ADDR_P2   0x0C
#define RX_ADDR_P3   0x0D
#define RX_ADDR_P4   0x0E
#define RX_ADDR_P5   0x0F
#define TX_ADDR      0x10
#define RX_PW_P0     0x11
#define RX_PW_P1     0x12
#define RX_PW_P2     0x13
#define RX_PW_P3     0x14
#define RX_PW_P4     0x15
#define RX_PW_P5     0x16
#define FIFO_STATUS  0x17
#define DYNPD        0x1C
#define FEATURE      0x1D

/* ===== Commands ===== */
#define R_REGISTER     0x00
#define W_REGISTER     0x20
#define REGISTER_MASK  0x1F
#define ACTIVATE       0x50
#define R_RX_PL_WID    0x60
#define R_RX_PAYLOAD   0x61
#define W_TX_PAYLOAD   0xA0
#define W_ACK_PAYLOAD  0xA8
#define FLUSH_TX       0xE1
#define FLUSH_RX       0xE2
#define REUSE_TX_PL    0xE3
#define NOP            0xFF

#ifdef __cplusplus
}
#endif
#endif /* NRF24_H */
