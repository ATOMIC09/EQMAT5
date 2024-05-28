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

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

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

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// float l_a0, l_a1, l_a2, l_b1, l_b2, lin_z1, lin_z2, lout_z1, lout_z2;
// float r_a0, r_a1, r_a2, r_b1, r_b2, rin_z1, rin_z2, rout_z1, rout_z2;
// uint8_t tx_buffer[17] = "Serial Ready!\n\r";
// uint8_t rx_buffer[70];
uint16_t rxBuf[256];
uint16_t txBuf[256];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

// typedef struct {
//   float a0;
//   float a1;
//   float a2;
//   float b0;
//   float b1;
//   float b2;
// } FilterCoeffs;

// FilterCoeffs lowBandCoeffs;
// FilterCoeffs midBandCoeffs;
// FilterCoeffs highBandCoeffs;

// void parseAndStoreCoeffs(char *rx_buffer);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  inBufPtr = &adcData[0];
  outBufPtr = &dacData[0];
  dataReady = 1;
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  inBufPtr = &adcData[AUDIO_BUFFER_SIZE / 2];
  outBufPtr = &dacData[AUDIO_BUFFER_SIZE / 2];
  dataReady = 1;
}

void processData(){
  static float leftIn, leftOut, rightIn, rightOut;

  for (uint8_t n = 0; n < (AUDIO_BUFFER_SIZE / 2) - 1; n += 2){
    // Left channel
    // Convert ADC data to float
    leftIn = INT16_TO_FLOAT * inBufPtr[n];
    if (leftIn > 1.0f){
      leftIn = -2.0f;
    }
    // Compute the left channel output
    leftOut = peaking_filter_update(&lowfilt, leftIn);
    leftOut = peaking_filter_update(&midfilt, leftOut);
    leftOut = peaking_filter_update(&highfilt, leftOut);
    
    // Convert back to int16
    outBufPtr[n] = (int16_t)(FLOAT_TO_INT16 * leftOut);

    // Right channel
    // Convert ADC data to float
    rightIn = INT16_TO_FLOAT * inBufPtr[n + 1];
    if (rightIn > 1.0f){
      rightIn = -2.0f;
    }
    // Compute the right channel output
    rightOut = rightIn;
    // Convert back to int16
    outBufPtr[n + 1] = (int16_t)(FLOAT_TO_INT16 * rightOut);
  }
  dataReady = 0;
}

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  
  // Start UART communication
  // HAL_UART_Transmit(&huart1, tx_buffer, sizeof(tx_buffer), 10); // Send ready message
  // HAL_UART_Receive_IT(&huart1, rx_buffer, sizeof(rx_buffer)); // Start UART receive

  // Set default filter coefficients
  // parseAndStoreCoeffs("Reset");

  HAL_I2SEx_TransmitReceive_DMA (&hi2s2, txBuf, rxBuf, 4);

  // //left-channel, High-Pass, 1kHz, fs=96kHz, q=0.7
  // l_a0 = 0.9543457485325094f;
  // l_a1 = -1.9086914970650188f;
  // l_a2 = 0.9543457485325094f;
  // l_b1 = -1.9066459797557103f;
  // l_b2 = 0.9107370143743273f;

  // //right-channel, Low-Pass, 1kHz, fs)96 kHz, q=0.7
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

    // Blink the LED while working
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
    HAL_Delay(1000);
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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

int _write(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
	  ITM_SendChar(*ptr++);
  }
  return len;
}

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//   /* Prevent unused argument(s) compilation warning */
//   UNUSED(huart);
//   /* NOTE: This function should not be modified, when the callback is needed,
//            the HAL_UART_RxCpltCallback could be implemented in the user file
//    */
//   // HAL_UART_Transmit(&huart1, rx_buffer, sizeof(rx_buffer), 10); // Echo the received data
//   printf("Received UART: %s\n", rx_buffer); // Print the received data to serial

  // fix warning: pointer targets in passing argument 1 of 'parseAndStoreCoeffs' differ in signedness [-Wpointer-sign]
//   parseAndStoreCoeffs((char *)rx_buffer); // Parse the received data
  
//   memset(rx_buffer, 0, sizeof(rx_buffer)); // Clear the buffer
//   HAL_UART_Receive_IT(&huart1, rx_buffer, sizeof(rx_buffer)); // Start the next receive
// }

// void parseAndStoreCoeffs(char *rx_buffer) {
//     // Determine which band the coefficients are for
//     if (strncmp(rx_buffer, "Low", 3) == 0) {
//         sscanf(rx_buffer, "Low %f %f %f %f %f %f", 
//                &lowBandCoeffs.a0, &lowBandCoeffs.a1, &lowBandCoeffs.a2, 
//                &lowBandCoeffs.b0, &lowBandCoeffs.b1, &lowBandCoeffs.b2);
//         printf("Parsed Low: %f %f %f %f %f %f\n", 
//                lowBandCoeffs.a0, lowBandCoeffs.a1, lowBandCoeffs.a2, 
//                lowBandCoeffs.b0, lowBandCoeffs.b1, lowBandCoeffs.b2);
//     } else if (strncmp(rx_buffer, "Mid", 3) == 0) {
//         sscanf(rx_buffer, "Mid %f %f %f %f %f %f", 
//                &midBandCoeffs.a0, &midBandCoeffs.a1, &midBandCoeffs.a2, 
//                &midBandCoeffs.b0, &midBandCoeffs.b1, &midBandCoeffs.b2);
//         printf("Parsed Mid: %f %f %f %f %f %f\n",
//                 midBandCoeffs.a0, midBandCoeffs.a1, midBandCoeffs.a2, 
//                 midBandCoeffs.b0, midBandCoeffs.b1, midBandCoeffs.b2);
//     } else if (strncmp(rx_buffer, "High", 4) == 0) {
//         sscanf(rx_buffer, "High %f %f %f %f %f %f", 
//                &highBandCoeffs.a0, &highBandCoeffs.a1, &highBandCoeffs.a2, 
//                &highBandCoeffs.b0, &highBandCoeffs.b1, &highBandCoeffs.b2);
//         printf("Parsed High: %f %f %f %f %f %f\n",
//                 highBandCoeffs.a0, highBandCoeffs.a1, highBandCoeffs.a2, 
//                 highBandCoeffs.b0, highBandCoeffs.b1, highBandCoeffs.b2);
//     } else if (strncmp(rx_buffer, "Reset", 5) == 0) {
//         lowBandCoeffs.a0 = 1.001636;
//         lowBandCoeffs.a1 = -1.999989;
//         lowBandCoeffs.a2 = 0.998364;
//         lowBandCoeffs.b0 = 1.001636;
//         lowBandCoeffs.b1 = -1.999989;
//         lowBandCoeffs.b2 = 0.998364;
//         midBandCoeffs.a0 = 1.049009;
//         midBandCoeffs.a1 = -1.990369;
//         midBandCoeffs.a2 = 0.950991;
//         midBandCoeffs.b0 = 1.049009;
//         midBandCoeffs.b1 = -1.990369;
//         midBandCoeffs.b2 = 0.950991;
//         highBandCoeffs.a0 = 1.304381;
//         highBandCoeffs.a1 = -1.586707;
//         highBandCoeffs.a2 = 0.695619;
//         highBandCoeffs.b0 = 1.304381;
//         highBandCoeffs.b1 = -1.586707;
//         highBandCoeffs.b2 = 0.695619;
//         printf("Coefficients reset!\n");
//     } else {
//         printf("Invalid parameter\n");
//     }
// }

// int Calc_IIR_Left (int inSample) {
// 	float inSampleF = (float)inSample;
// 	float outSampleF =
// 			l_a0 * inSampleF
// 			+ l_a1 * lin_z1
// 			+ l_a2 * lin_z2
// 			- l_b1 * lout_z1
// 			- l_b2 * lout_z2;
// 	lin_z2 = lin_z1;
// 	lin_z1 = inSampleF;
// 	lout_z2 = lout_z1;
// 	lout_z1 = outSampleF;

// 	return (int) outSampleF;
// }

// int Calc_IIR_Right (int inSample) {
// 	float inSampleF = (float)inSample;
// 	float outSampleF =
// 			r_a0 * inSampleF
// 			+ r_a1 * rin_z1
// 			+ r_a2 * rin_z2
// 			- r_b1 * rout_z1
// 			- r_b2 * rout_z2;
// 	rin_z2 = rin_z1;
// 	rin_z1 = inSampleF;
// 	rout_z2 = rout_z1;
// 	rout_z1 = outSampleF;

// 	return (int) outSampleF;
// }

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  //restore signed 24 bit sample from 16-bit buffers
	int lSample = (int) (rxBuf[0]<<16)|rxBuf[1];
	int rSample = (int) (rxBuf[2]<<16)|rxBuf[3];

	//run HP on left channel and LP on right channel
	// lSample = Calc_IIR_Left(lSample);
	// rSample = Calc_IIR_Left(rSample);

	//restore to buffer
	txBuf[0] = (lSample>>16)&0xFFFF;
	txBuf[1] = lSample&0xFFFF;
	txBuf[2] = (rSample>>16)&0xFFFF;
	txBuf[3] = rSample&0xFFFF;
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  //restore signed 24 bit sample from 16-bit buffers
	int lSample = (int) (rxBuf[4]<<16)|rxBuf[5];
	int rSample = (int) (rxBuf[6]<<16)|rxBuf[7];

	//run HP on left channel and LP on right channel
	// lSample = Calc_IIR_Left(lSample);
	// rSample = Calc_IIR_Left(rSample);

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
