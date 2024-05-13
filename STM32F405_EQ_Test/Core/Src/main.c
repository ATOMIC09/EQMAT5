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
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include "peaking_filter.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_i2s2_ext_rx;
DMA_HandleTypeDef hdma_spi2_tx;

/* USER CODE BEGIN PV */
                          // #define BUFFER_SIZE 2048
                          // uint16_t adcData[BUFFER_SIZE];
                          // uint16_t dacData[BUFFER_SIZE];
                          // int isDataReady = 0;
                          // static volatile uint16_t *inBuff;
                          // static volatile uint16_t *outBuff;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
/* USER CODE BEGIN PFP */
// float l_a0, l_a1, l_a2, l_b1, l_b2, lin_z1, lin_z2, lout_z1, lout_z2;
// float r_a0, r_a1, r_a2, r_b1, r_b2, rin_z1, rin_z2, rout_z1, rout_z2;

uint16_t rxBuf[256];
uint16_t txBuf[256];

// Define the FilterCoeffs struct to hold filter coefficients
typedef struct {
  float a0;
  float a1;
  float a2;
  float b1;
  float b2;
} FilterCoeffs;

// Define filt
peaking_filter filt;

// Define filter band selection constants (assuming integer values)
// #define LOW_SHELF_BAND 1
// #define LOW_MID_BAND 2
// #define MID_BAND 3
// #define HIGH_MID_BAND 4
// #define HIGH_SHELF_BAND 5

/* Filter Coefficients (replace with actual values) */
const FilterCoeffs lowShelfCoeffs = {1.0f, -1.995372f, 0.995383f, -1.995372f, 0.995383f};
const FilterCoeffs lowMidCoeffs = {0.009171f, 0.0f, -0.009171f, -1.981488f, 0.981658f};
const FilterCoeffs midBandCoeffs = {0.0648168f, 0.0f, -0.0648168f, -1.861360f, 0.870366f};
const FilterCoeffs highMidCoeffs = {0.1212275f, 0.0f, -0.1212275f, -1.723774f, 0.757545f};
const FilterCoeffs highShelfCoeffs = {1.0f, -1.109229f, 0.398152f, -1.109229f, 0.398152f};

// Left channel filter state variables
float lin_z1;
float lin_z2;
float lout_z1;
float lout_z2;

// Right channel filter state variables
float rin_z1;
float rin_z2;
float rout_z1;
float rout_z2;

// Placeholder for gain values from MATLAB GUI (replace with actual implementation)
float gainLowShelf = 1.0f;
float gainLowMid = 1.0f;
float gainMid = 1.0f;
float gainHighMid = 1.0f;
float gainHighShelf = 1.0f;

// int selectedBand = LOW_SHELF_BAND;  // Change this value to select the desired filter band

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2S2_Init();
  /* USER CODE BEGIN 2 */
  peaking_filter_init(&filt, 96000.0f);
  peaking_filter_set_params(&filt, 500.0f, 100.0f, 0.1f);
  HAL_I2SEx_TransmitReceive_DMA (&hi2s2, txBuf, rxBuf, 4);

  // Low, Fc=50Hz, Fs=96kHz, q=0.7071
  // l_a0 = 0.0000026711181320911584f;
  // l_a1 = 0.000005342236264182317f;
  // l_a2 = 0.0000026711181320911584f;
  // l_b1 = -1.9953719609930045f;
  // l_b2 = 0.9953826454655329f;

  // Low-mid, Fc=200Hz, Fs=96kHz, q=0.7071
  // How tf !?


  //left-channel, High-Pass, 1kHz, fs=96kHz, q=0.7
  // l_a0 = 0.9543457485325094f;
  // l_a1 = -1.9086914970650188f;
  // l_a2 = 0.9543457485325094f;
  // l_b1 = -1.9066459797557103f;
  // l_b2 = 0.9107370143743273f;

  //right-channel, Low-Pass, 1kHz, fs)96 kHz, q=0.7
  // r_a0 = 0.0010227586546542474f;
  // r_a1 = 0.002045517309308495f;
  // r_a2 = 0.0010227586546542474f;
  // r_b1 = -1.9066459797557103f;
  // r_b2 = 0.9107370143743273f;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // if (isDataReady){
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
//      isDataReady = 0;
	  	// }
	  // else {
		  // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0);
	// }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Function for left channel filtering (replace with your actual implementation)
int Calc_IIR_Left(int inSample, const FilterCoeffs *coeffs) {
  float inSampleF = (float)inSample;
  float outSampleF =
      coeffs->a0 * inSampleF
      + coeffs->a1 * lin_z1
      + coeffs->a2 * lin_z2
      - coeffs->b1 * lout_z1
      - coeffs->b2 * lout_z2;
  lin_z2 = lin_z1;
  lin_z1 = inSampleF;
  lout_z2 = lout_z1;
  lout_z1 = outSampleF;

  return (int) outSampleF;
}

// Function for right channel filtering (replace with your actual implementation)
int Calc_IIR_Right(int inSample, const FilterCoeffs *coeffs) {
  float inSampleF = (float)inSample;
  float outSampleF =
      coeffs->a0 * inSampleF
      + coeffs->a1 * rin_z1
      + coeffs->a2 * rin_z2
      - coeffs->b1 * rout_z1
      - coeffs->b2 * rout_z2;
  rin_z2 = rin_z1;
  rin_z1 = inSampleF;
  rout_z2 = rout_z1;
  rout_z1 = outSampleF;

  return (int) outSampleF;
}

void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s){
    // Restore signed 24-bit sample from 16-bit buffers
    int rSample = (int) (rxBuf[0]<<16) | rxBuf[1];
    int lSample = (int) (rxBuf[2]<<16) | rxBuf[3];

    // test peaking filter
    lSample = peaking_filter_update(&filt, lSample);
    rSample = peaking_filter_update(&filt, rSample);

    // multiply by 2 (leftshift) -> +3dB per sample (don't use, it will make a *click* noise)
    // lSample = lSample<<1;
    // rSample = rSample<<1;
    
    // Run filtering for each band on the left and right channels
    // int filteredOutputs[10];

    // filteredOutputs[0] = Calc_IIR_Left(lSample, &lowShelfCoeffs) * gainLowShelf;
    // filteredOutputs[1] = Calc_IIR_Right(rSample, &lowShelfCoeffs) * gainLowShelf;
    // filteredOutputs[2] = Calc_IIR_Left(lSample, &lowMidCoeffs) * gainLowMid;
    // filteredOutputs[3] = Calc_IIR_Right(rSample, &lowMidCoeffs) * gainLowMid;
    // filteredOutputs[4] = Calc_IIR_Left(lSample, &midBandCoeffs) * gainMid;
    // filteredOutputs[5] = Calc_IIR_Right(rSample, &midBandCoeffs) * gainMid;
    // filteredOutputs[6] = Calc_IIR_Left(lSample, &highMidCoeffs) * gainHighMid;
    // filteredOutputs[7] = Calc_IIR_Right(rSample, &highMidCoeffs) * gainHighMid;
    // filteredOutputs[8] = Calc_IIR_Left(lSample, &highShelfCoeffs) * gainHighShelf;
    // filteredOutputs[9] = Calc_IIR_Right(rSample, &highShelfCoeffs) * gainHighShelf;

    // Restore to buffer
    // for (int i = 0; i < 10; ++i) {
    //     txBuf[i * 2] = (int)(filteredOutputs[i] >> 16) & 0xFFFF;
    //     txBuf[i * 2 + 1] = (int)filteredOutputs[i] & 0xFFFF;
    // }

    //restore to buffer
    txBuf[0] = (lSample>>16)&0xFFFF;
    txBuf[1] = lSample&0xFFFF;
    txBuf[2] = (rSample>>16)&0xFFFF;
    txBuf[3] = rSample&0xFFFF;
}

void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s){
    // Restore signed 24-bit sample from 16-bit buffers
    int rSample = (int) (rxBuf[4]<<16) | rxBuf[5];
    int lSample = (int) (rxBuf[6]<<16) | rxBuf[7];

    // test peaking filter
    lSample = peaking_filter_update(&filt, lSample);
    rSample = peaking_filter_update(&filt, rSample);

    // multiply by 2 (leftshift) -> +3dB per sample (don't use, it will make a *click* noise)
    // lSample = lSample<<1;
    // rSample = rSample<<1;

    // Run filtering for each band on the left and right channels
    // int filteredOutputs[10];

    // filteredOutputs[0] = Calc_IIR_Left(lSample, &lowShelfCoeffs) * gainLowShelf;
    // filteredOutputs[1] = Calc_IIR_Right(rSample, &lowShelfCoeffs) * gainLowShelf;
    // filteredOutputs[2] = Calc_IIR_Left(lSample, &lowMidCoeffs) * gainLowMid;
    // filteredOutputs[3] = Calc_IIR_Right(rSample, &lowMidCoeffs) * gainLowMid;
    // filteredOutputs[4] = Calc_IIR_Left(lSample, &midBandCoeffs) * gainMid;
    // filteredOutputs[5] = Calc_IIR_Right(rSample, &midBandCoeffs) * gainMid;
    // filteredOutputs[6] = Calc_IIR_Left(lSample, &highMidCoeffs) * gainHighMid;
    // filteredOutputs[7] = Calc_IIR_Right(rSample, &highMidCoeffs) * gainHighMid;
    // filteredOutputs[8] = Calc_IIR_Left(lSample, &highShelfCoeffs) * gainHighShelf;
    // filteredOutputs[9] = Calc_IIR_Right(rSample, &highShelfCoeffs) * gainHighShelf;

    // Restore to buffer
    // for (int i = 0; i < 10; ++i) {
    //     txBuf[i * 2 + 4] = (int)(filteredOutputs[i] >> 16) & 0xFFFF;
    //     txBuf[i * 2 + 5] = (int)filteredOutputs[i] & 0xFFFF;
    // }

    //restore to buffer
    txBuf[4] = (lSample>>16)&0xFFFF;
    txBuf[5] = lSample&0xFFFF;
    txBuf[6] = (rSample>>16)&0xFFFF;
    txBuf[7] = rSample&0xFFFF;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
