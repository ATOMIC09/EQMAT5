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
#include "arm_math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BLOCK_SIZE_FLOAT 512
#define BLOCK_SIZE_U16 2048

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

// UART
uint8_t tx_buffer[17] = "Serial Ready!\n\r";
uint8_t rx_buffer[65];

// I2S
float iir_l_state [4];
float iir_r_state [4];
uint16_t rxBuf[BLOCK_SIZE_U16*2];
uint16_t txBuf[BLOCK_SIZE_U16*2];
float l_buf_in [BLOCK_SIZE_FLOAT*2];
float r_buf_in [BLOCK_SIZE_FLOAT*2];
float l_buf_out [BLOCK_SIZE_FLOAT*2];
float r_buf_out [BLOCK_SIZE_FLOAT*2];
uint8_t callback_state = 0;
arm_biquad_casd_df1_inst_f32 iirsettings_l, iirsettings_r;

//IIR coefficient array
// Default settings: Q = 1.0, Fs = 96kHz, Gain = 0
float iir_coeffs_low [5] = {
				1.000000f,
				-1.996722f,
				0.996733f,
				1.996722f,
				-0.996733
};

float iir_coeffs_lowmid [5] = {
        1.000000f,
        -1.986825f,
        0.986996f,
        1.986825f,
        -0.986996
};

float iir_coeffs_mid [5] = {
        1.000000f,
        -1.897381f,
        0.906562f,
        1.897381f,
        -0.906562
};

float iir_coeffs_highmid [5] = {
        1.000000f,
        -1.787234f,
        0.822248f,
        1.787234f,
        -0.822248
};

float iir_coeffs_high [5] = {
        1.000000f,
        -1.216444f,
        0.533295f,
        1.216444f,
        -0.533295
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

static void parseAndStoreCoeffs(char *rx_buffer);

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // Start UART communication
  HAL_UART_Transmit(&huart1, tx_buffer, sizeof(tx_buffer), 10); // Send ready message
  HAL_UART_Receive_IT(&huart1, rx_buffer, sizeof(rx_buffer)); // Start UART receive
  
  // Start I2S communication
  // Initialize IIR filter coefficients
  arm_biquad_cascade_df1_init_f32 ( &iirsettings_l, 1, &iir_coeffs_low[0], &iir_l_state[0]);
  arm_biquad_cascade_df1_init_f32 ( &iirsettings_r, 1, &iir_coeffs_low[0], &iir_r_state[0]);
  arm_biquad_cascade_df1_init_f32 ( &iirsettings_l, 1, &iir_coeffs_lowmid[0], &iir_l_state[0]);
  arm_biquad_cascade_df1_init_f32 ( &iirsettings_r, 1, &iir_coeffs_lowmid[0], &iir_r_state[0]);
  arm_biquad_cascade_df1_init_f32 ( &iirsettings_l, 1, &iir_coeffs_mid[0], &iir_l_state[0]);
  arm_biquad_cascade_df1_init_f32 ( &iirsettings_r, 1, &iir_coeffs_mid[0], &iir_r_state[0]);
  arm_biquad_cascade_df1_init_f32 ( &iirsettings_l, 1, &iir_coeffs_highmid[0], &iir_l_state[0]);
  arm_biquad_cascade_df1_init_f32 ( &iirsettings_r, 1, &iir_coeffs_highmid[0], &iir_r_state[0]);
  arm_biquad_cascade_df1_init_f32 ( &iirsettings_l, 1, &iir_coeffs_high[0], &iir_l_state[0]);
  arm_biquad_cascade_df1_init_f32 ( &iirsettings_r, 1, &iir_coeffs_high[0], &iir_r_state[0]);

  // Start I2S with 2048 samples transmission -> 4096*u16 words
  HAL_I2SEx_TransmitReceive_DMA (&hi2s2, txBuf, rxBuf, BLOCK_SIZE_U16);

  int offset_r_ptr;
  int offset_w_ptr, w_ptr;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (callback_state != 0) {

		  // Decide if it was half or full cplt callback
		  if (callback_state == 1)   {
			  	  offset_r_ptr = 0;
			  	  offset_w_ptr = 0;
			  	  w_ptr = 0;
			  }

		  else if (callback_state == 2) {
			  offset_r_ptr = BLOCK_SIZE_U16;
			  offset_w_ptr = BLOCK_SIZE_FLOAT;
			  w_ptr = BLOCK_SIZE_FLOAT;
		  }


		  // Restore input sample buffer to float array
		  for (int i=offset_r_ptr; i<offset_r_ptr+BLOCK_SIZE_U16; i=i+4) {
			  l_buf_in[w_ptr] = (float) ((int) (rxBuf[i]<<16)|rxBuf[i+1]);
			  r_buf_in[w_ptr] = (float) ((int) (rxBuf[i+2]<<16)|rxBuf[i+3]);
			  w_ptr++;
		  }


		  // Process IIR
		  arm_biquad_cascade_df1_f32 (&iirsettings_l, &l_buf_in[offset_w_ptr], &l_buf_out[offset_w_ptr],BLOCK_SIZE_FLOAT);
		  arm_biquad_cascade_df1_f32 (&iirsettings_r, &r_buf_in[offset_w_ptr], &r_buf_out[offset_w_ptr],BLOCK_SIZE_FLOAT);

      // Bypass filter
      // for (int i=offset_w_ptr; i<offset_w_ptr+BLOCK_SIZE_FLOAT; i++) {
      //   l_buf_out[i] = l_buf_in[i];
      //   r_buf_out[i] = r_buf_in[i];
      // }

		  //restore processed float-array to output sample-buffer
		  w_ptr = offset_w_ptr;

		  for (int i=offset_r_ptr; i<offset_r_ptr+BLOCK_SIZE_U16; i=i+4) {
				txBuf[i] =  (((int)l_buf_out[w_ptr])>>16)&0xFFFF;
				txBuf[i+1] = ((int)l_buf_out[w_ptr])&0xFFFF;
				txBuf[i+2] = (((int)r_buf_out[w_ptr])>>16)&0xFFFF;
				txBuf[i+3] = ((int)r_buf_out[w_ptr])&0xFFFF;
				w_ptr++;
		  }

		  callback_state = 0;

	  }
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

// printf function
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
  // HAL_UART_Transmit(&huart1, rx_buffer, sizeof(rx_buffer), 10); // Echo the received data to serial
  printf("Received UART: %s\n", rx_buffer); // Print the received data to serial
  parseAndStoreCoeffs((char *)rx_buffer); // Parse the received data
  memset(rx_buffer, 0, sizeof(rx_buffer)); // Clear the buffer
  HAL_UART_Receive_IT(&huart1, rx_buffer, sizeof(rx_buffer)); // Start the next receive
}

static void parseAndStoreCoeffs(char *rx_buffer) {
    // Determine which band the coefficients are for
    if (strncmp(rx_buffer, "LowMid", 6) == 0) {
        sscanf(rx_buffer, "LowMid %f %f %f %f %f",
                &iir_coeffs_lowmid[0], &iir_coeffs_lowmid[1], &iir_coeffs_lowmid[2], &iir_coeffs_lowmid[3], &iir_coeffs_lowmid[4]);
        printf("Parsed LowMid : %f %f %f %f %f\n", iir_coeffs_lowmid[0], iir_coeffs_lowmid[1], iir_coeffs_lowmid[2], iir_coeffs_lowmid[3], iir_coeffs_lowmid[4]);
        arm_biquad_cascade_df1_init_f32 (&iirsettings_l, 1, &iir_coeffs_lowmid[0], &iir_l_state[0]);
        arm_biquad_cascade_df1_init_f32 (&iirsettings_r, 1, &iir_coeffs_lowmid[0], &iir_r_state[0]); 
    } else if (strncmp(rx_buffer, "HighMid", 7) == 0) {
        sscanf(rx_buffer, "HighMid %f %f %f %f %f",
                &iir_coeffs_highmid[0], &iir_coeffs_highmid[1], &iir_coeffs_highmid[2], &iir_coeffs_highmid[3], &iir_coeffs_highmid[4]);
        printf("Parsed HighMid : %f %f %f %f %f\n", iir_coeffs_highmid[0], iir_coeffs_highmid[1], iir_coeffs_highmid[2], iir_coeffs_highmid[3], iir_coeffs_highmid[4]);
        arm_biquad_cascade_df1_init_f32 (&iirsettings_l, 1, &iir_coeffs_highmid[0], &iir_l_state[0]);
        arm_biquad_cascade_df1_init_f32 (&iirsettings_r, 1, &iir_coeffs_highmid[0], &iir_r_state[0]); 
    } else if (strncmp(rx_buffer, "Mid", 3) == 0) {
        sscanf(rx_buffer, "Mid %f %f %f %f %f",
                &iir_coeffs_mid[0], &iir_coeffs_mid[1], &iir_coeffs_mid[2], &iir_coeffs_mid[3], &iir_coeffs_mid[4]);
        printf("Parsed Mid : %f %f %f %f %f\n", iir_coeffs_mid[0], iir_coeffs_mid[1], iir_coeffs_mid[2], iir_coeffs_mid[3], iir_coeffs_mid[4]);
        arm_biquad_cascade_df1_init_f32 (&iirsettings_l, 1, &iir_coeffs_mid[0], &iir_l_state[0]);
        arm_biquad_cascade_df1_init_f32 (&iirsettings_r, 1, &iir_coeffs_mid[0], &iir_r_state[0]); 
    } else if (strncmp(rx_buffer, "Low", 3) == 0) {
        sscanf(rx_buffer, "Low %f %f %f %f %f",
                &iir_coeffs_low[0], &iir_coeffs_low[1], &iir_coeffs_low[2], &iir_coeffs_low[3], &iir_coeffs_low[4]);
        printf("Parsed Low : %f %f %f %f %f\n", iir_coeffs_low[0], iir_coeffs_low[1], iir_coeffs_low[2], iir_coeffs_low[3], iir_coeffs_low[4]);
        arm_biquad_cascade_df1_init_f32 (&iirsettings_l, 1, &iir_coeffs_low[0], &iir_l_state[0]);
        arm_biquad_cascade_df1_init_f32 (&iirsettings_r, 1, &iir_coeffs_low[0], &iir_r_state[0]); 
    } else if (strncmp(rx_buffer, "High", 4) == 0) {
        sscanf(rx_buffer, "High %f %f %f %f %f",
                &iir_coeffs_high[0], &iir_coeffs_high[1], &iir_coeffs_high[2], &iir_coeffs_high[3], &iir_coeffs_high[4]);
        printf("Parsed High : %f %f %f %f %f\n", iir_coeffs_high[0], iir_coeffs_high[1], iir_coeffs_high[2], iir_coeffs_high[3], iir_coeffs_high[4]);
        arm_biquad_cascade_df1_init_f32 (&iirsettings_l, 1, &iir_coeffs_high[0], &iir_l_state[0]);
        arm_biquad_cascade_df1_init_f32 (&iirsettings_r, 1, &iir_coeffs_high[0], &iir_r_state[0]);
    } else if (strncmp(rx_buffer, "Reset", 5) == 0) {
        iir_coeffs_low[0] = 1.000000f;
        iir_coeffs_low[1] = -1.996722f;
        iir_coeffs_low[2] = 0.996733f;
        iir_coeffs_low[3] = 1.996722f;
        iir_coeffs_low[4] = -0.996733;

        iir_coeffs_lowmid[0] = 1.000000f;
        iir_coeffs_lowmid[1] = -1.986825f;
        iir_coeffs_lowmid[2] = 0.986996f;
        iir_coeffs_lowmid[3] = 1.98683f;
        iir_coeffs_lowmid[4] = -0.986996;

        iir_coeffs_mid[0] = 1.000000f;
        iir_coeffs_mid[1] = -1.897381f;
        iir_coeffs_mid[2] = 0.906562f;
        iir_coeffs_mid[3] = 1.897381f;
        iir_coeffs_mid[4] = -0.906562;

        iir_coeffs_highmid[0] = 0.999999f;
        iir_coeffs_highmid[1] = -1.787234f;
        iir_coeffs_highmid[2] = 0.822248f;
        iir_coeffs_highmid[3] = 1.787234f;
        iir_coeffs_highmid[4] = -0.822248;

        iir_coeffs_high[0] = 0.999999f;
        iir_coeffs_high[1] = -1.216444f;
        iir_coeffs_high[2] = 0.533295f;
        iir_coeffs_high[3] = 1.216444f;
        iir_coeffs_high[4] = -0.533295;

        arm_biquad_cascade_df1_init_f32 ( &iirsettings_l, 1, &iir_coeffs_low[0], &iir_l_state[0]);
        arm_biquad_cascade_df1_init_f32 ( &iirsettings_r, 1, &iir_coeffs_low[0], &iir_r_state[0]);
        arm_biquad_cascade_df1_init_f32 ( &iirsettings_l, 1, &iir_coeffs_lowmid[0], &iir_l_state[0]);
        arm_biquad_cascade_df1_init_f32 ( &iirsettings_r, 1, &iir_coeffs_lowmid[0], &iir_r_state[0]);
        arm_biquad_cascade_df1_init_f32 ( &iirsettings_l, 1, &iir_coeffs_mid[0], &iir_l_state[0]);
        arm_biquad_cascade_df1_init_f32 ( &iirsettings_r, 1, &iir_coeffs_mid[0], &iir_r_state[0]);
        arm_biquad_cascade_df1_init_f32 ( &iirsettings_l, 1, &iir_coeffs_highmid[0], &iir_l_state[0]);
        arm_biquad_cascade_df1_init_f32 ( &iirsettings_r, 1, &iir_coeffs_highmid[0], &iir_r_state[0]);
        arm_biquad_cascade_df1_init_f32 ( &iirsettings_l, 1, &iir_coeffs_high[0], &iir_l_state[0]);
        arm_biquad_cascade_df1_init_f32 ( &iirsettings_r, 1, &iir_coeffs_high[0], &iir_r_state[0]);
        printf("Coefficients reset!\n");
    } else {
        printf("Invalid command\n");
    }
}

void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s){
  callback_state = 1;
}

void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s){
  callback_state = 2;
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
