/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "nrf24.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HEX_CHARS      "0123456789ABCDEF"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char uart_msg[100];
void print(char uart_buffer[]);
void printValue(float value);
void printHex(uint8_t value);
void generate_data(uint16_t dataid, float* data);
void copyArray(float *source, float *destination, uint8_t size);
float rand_float(float min, float max);
uint8_t chooseRandomNumber(int *array, int size);

/* ---- RF address must be 5 bytes ---- */
uint8_t TxAddress[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7}; // 5-byte address (E7E7E7E7E7)

// Datos a enviar (con distintos largos)
float data1[] = {0x610, 1.23f, 4.56f, 7.89f, 0.12f, 3.45f, 6.78f};
float data2[] = {0x600, 10.11f, 12.13f, 14.15f, 16.17f, 18.19f, 20.21f, 22.23f};
float data3[] = {0x630, 23.24f, 25.26f};
float data4[] = {0x640, 26.27f, 28.29f, 30.31f};
float data5[] = {0x650, 31.32f, 33.34f, 35.36f, 37.38f};
float data6[] = {0x670, 38.39f, 40.41f, 42.43f, 44.45f};
float data7[] = {0x660, 45.46f, 47.48f, 49.50f, 51.52f};
float data8[] = {0x680, 52.53f, 54.55f, 56.57f, 58.59f};

/* Tabla de punteros y largos reales */
static float * const tables[] = { data1, data2, data3, data4, data5, data6, data7, data8 };
static const uint8_t lengths[] = {
  sizeof(data1)/sizeof(float),
  sizeof(data2)/sizeof(float),
  sizeof(data3)/sizeof(float),
  sizeof(data4)/sizeof(float),
  sizeof(data5)/sizeof(float),
  sizeof(data6)/sizeof(float),
  sizeof(data7)/sizeof(float),
  sizeof(data8)/sizeof(float)
};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_SPI2_Init();

  /* USER CODE BEGIN 2 */
  NRF24_Init();
  NRF24_TxMode(TxAddress, 76);
  NRF24_Dump();

  int i = 0;
  print("Inicializando TX");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
      HAL_Delay(300);

      /* ---- Construir SIEMPRE un frame de 8 floats (32 bytes) ---- */
      float pkt[8] = {0};                   // cero-relleno
      uint8_t n = lengths[i];
      if (n > 8) n = 8;
      memcpy(pkt, tables[i], n * sizeof(float));

      // Mostrar por UART exactamente lo que se enviará (desde pkt)
      sprintf(uart_msg, "\r\n[TX] Enviando paquete con ID: 0x%X", (uint16_t)pkt[0]);
      HAL_UART_Transmit(&hlpuart1, (uint8_t*)uart_msg, strlen(uart_msg), HAL_MAX_DELAY);

      for (int j = 1; j < 8; j++) {
          int ent = (int)pkt[j];
          int dec = (int)((pkt[j] - ent) * 100);
          if (dec < 0) dec = -dec;
          sprintf(uart_msg, ", V%d: %d.%02d", j, ent, dec);
          HAL_UART_Transmit(&hlpuart1, (uint8_t*)uart_msg, strlen(uart_msg), HAL_MAX_DELAY);
      }
      HAL_UART_Transmit(&hlpuart1, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);

      // Enviar por radio el frame bien formado (32B)
      uint8_t ok = NRF24_Transmit((uint8_t*)pkt);
      uint8_t st = nrf24_ReadReg(STATUS);
      uint8_t ob = nrf24_ReadReg(OBSERVE_TX);   // [PLOS_CNT | ARC_CNT]

      if (ok) {
          print("[TX] Transmisión exitosa.");
      } else {
          print("[TX] FALLO de transmisión.");
      }
      sprintf(uart_msg, "STATUS=%02X OBSERVE_TX=%02X\r\n", st, ob);
      HAL_UART_Transmit(&hlpuart1,(uint8_t*)uart_msg,strlen(uart_msg),HAL_MAX_DELAY);

      i++;
      if (i == 8) i = 0;
  }
  /* USER CODE END WHILE */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK) { Error_Handler(); }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) { Error_Handler(); }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) { Error_Handler(); }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Idle levels at reset: CSN=HIGH (not selected), CE=LOW */
  HAL_GPIO_WritePin(GPIOC, CSN_Pin, GPIO_PIN_SET);    // CSN HIGH
  HAL_GPIO_WritePin(GPIOC, CE_Pin,  GPIO_PIN_RESET);  // CE  LOW

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = NRF_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(NRF_IRQ_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = CSN_Pin|CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
void print(char uart_buffer[]){
	sprintf(uart_msg, "%s \n\r", uart_buffer);
	HAL_UART_Transmit(&hlpuart1,(uint8_t*)uart_msg,strlen(uart_msg),HAL_MAX_DELAY);
}

void printValue(float value){
	sprintf(uart_msg, "%.2f \n\r", value);
	HAL_UART_Transmit(&hlpuart1,(uint8_t*)uart_msg,strlen(uart_msg),HAL_MAX_DELAY);
}

void printHex(uint8_t value){
	sprintf(uart_msg, "%02X ", value);
	HAL_UART_Transmit(&hlpuart1,(uint8_t*)uart_msg,strlen(uart_msg),HAL_MAX_DELAY);
}
float rand_float(float min, float max) {
    return min + ((float)rand() / RAND_MAX) * (max - min);
}
uint8_t chooseRandomNumber(int *array, int size) {
    srand(time(NULL));
    uint8_t randomIndex = (uint8_t) (rand() % size);
    return array[randomIndex];
}
void copyArray(float *source, float *destination, uint8_t size) {
    for (uint8_t i = 0; i < size; i++) destination[i] = source[i];
}
void generate_data(uint16_t dataid, float* data) {
    uint8_t size = 0;
    switch (dataid) {
        case 0x610: {
            float tmp[7] = {dataid, rand_float(-10, 10), rand_float(-10, 10), rand_float(-10, 10), rand_float(-100, 100), rand_float(-100, 100), rand_float(-100, 100)};
            size = 7; copyArray(tmp, data, size); break;
        }
        case 0x600: {
            float tmp[7] = {dataid, rand_float(0, 100), rand_float(0, 100), rand_float(0, 100), rand_float(0, 10000), rand_float(0, 400), rand_float(0, 500)};
            size = 7; copyArray(tmp, data, size); break;
        }
        case 0x630: {
            float tmp[3] = {dataid, 1, 1};
            size = 3; copyArray(tmp, data, size); break;
        }
        case 0x640: {
            float tmp[4] = {dataid, rand_float(0, 500), rand_float(2.5f, 4.2f), rand_float(20, 60)};
            size = 4; copyArray(tmp, data, size); break;
        }
        case 0x650: {
            float tmp[5] = {dataid, rand_float(0, 200), rand_float(-90, 90), rand_float(-180, 180), rand_float(0, 8000)};
            size = 5; copyArray(tmp, data, size); break;
        }
        case 0x670: {
            float tmp[5] = {dataid, rand_float(0, 100), rand_float(0, 100), rand_float(0, 100), rand_float(0, 100)};
            size = 5; copyArray(tmp, data, size); break;
        }
        case 0x660: {
            float tmp[5] = {dataid, rand_float(0, 400), rand_float(0, 400), rand_float(0, 400), rand_float(0, 400)};
            size = 5; copyArray(tmp, data, size); break;
        }
        default:
            break;
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
}
#endif /* USE_FULL_ASSERT */
