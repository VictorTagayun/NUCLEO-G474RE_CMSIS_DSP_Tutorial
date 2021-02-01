/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 VictorT Electronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by VT under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
//#include <math.h>
#include "arm_math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

arm_fir_instance_f32 FIR_F32_Struct;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ARM_MATH_CM4
#define NB_SAMPLES     	64
#define FFT_Length_Tab 	1024
#define SAMPLES		2048 			/* 256 real party and 256 imaginary parts */
#define FFT_INVERSE_FLAG        ((uint8_t)0)
#define FFT_Normal_OUTPUT_FLAG  ((uint8_t)1)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;

/* USER CODE BEGIN PV */

// FIRs
uint32_t blockSize = BLOCK_SIZE;
uint32_t numBlocks = TEST_LENGTH_SAMPLES/BLOCK_SIZE;

q31_t aFIR_Q31_1kHz_15kHz[TEST_LENGTH_SAMPLES];
q15_t aFIR_Q15_1kHz_15kHz[TEST_LENGTH_SAMPLES];

float32_t 	aFIR_F32_Output[TEST_LENGTH_SAMPLES];
q15_t 			aFIR_Q15_Output[TEST_LENGTH_SAMPLES];
q31_t 			aFIR_Q31_Output[TEST_LENGTH_SAMPLES];

/* ----------------------------------------------------------------------
** Test input signal contains 1000Hz + 15000 Hz
** ------------------------------------------------------------------- */
float32_t aFIR_F32_1kHz_15kHz[TEST_LENGTH_SAMPLES] =
{
+0.0000000000f, +0.5924659585f, -0.0947343455f, +0.1913417162f, +1.0000000000f, +0.4174197128f, +0.3535533906f, +1.2552931065f,
+0.8660254038f, +0.4619397663f, +1.3194792169f, +1.1827865776f, +0.5000000000f, +1.1827865776f, +1.3194792169f, +0.4619397663f,
+0.8660254038f, +1.2552931065f, +0.3535533906f, +0.4174197128f, +1.0000000000f, +0.1913417162f, -0.0947343455f, +0.5924659585f,
-0.0000000000f, -0.5924659585f, +0.0947343455f, -0.1913417162f, -1.0000000000f, -0.4174197128f, -0.3535533906f, -1.2552931065f,
-0.8660254038f, -0.4619397663f, -1.3194792169f, -1.1827865776f, -0.5000000000f, -1.1827865776f, -1.3194792169f, -0.4619397663f,
-0.8660254038f, -1.2552931065f, -0.3535533906f, -0.4174197128f, -1.0000000000f, -0.1913417162f, +0.0947343455f, -0.5924659585f,
+0.0000000000f, +0.5924659585f, -0.0947343455f, +0.1913417162f, +1.0000000000f, +0.4174197128f, +0.3535533906f, +1.2552931065f,
+0.8660254038f, +0.4619397663f, +1.3194792169f, +1.1827865776f, +0.5000000000f, +1.1827865776f, +1.3194792169f, +0.4619397663f,
+0.8660254038f, +1.2552931065f, +0.3535533906f, +0.4174197128f, +1.0000000000f, +0.1913417162f, -0.0947343455f, +0.5924659585f,
+0.0000000000f, -0.5924659585f, +0.0947343455f, -0.1913417162f, -1.0000000000f, -0.4174197128f, -0.3535533906f, -1.2552931065f,
-0.8660254038f, -0.4619397663f, -1.3194792169f, -1.1827865776f, -0.5000000000f, -1.1827865776f, -1.3194792169f, -0.4619397663f,
-0.8660254038f, -1.2552931065f, -0.3535533906f, -0.4174197128f, -1.0000000000f, -0.1913417162f, +0.0947343455f, -0.5924659585f,
+0.0000000000f, +0.5924659585f, -0.0947343455f, +0.1913417162f, +1.0000000000f, +0.4174197128f, +0.3535533906f, +1.2552931065f,
+0.8660254038f, +0.4619397663f, +1.3194792169f, +1.1827865776f, +0.5000000000f, +1.1827865776f, +1.3194792169f, +0.4619397663f,
+0.8660254038f, +1.2552931065f, +0.3535533906f, +0.4174197128f, +1.0000000000f, +0.1913417162f, -0.0947343455f, +0.5924659585f,
+0.0000000000f, -0.5924659585f, +0.0947343455f, -0.1913417162f, -1.0000000000f, -0.4174197128f, -0.3535533906f, -1.2552931065f,
-0.8660254038f, -0.4619397663f, -1.3194792169f, -1.1827865776f, -0.5000000000f, -1.1827865776f, -1.3194792169f, -0.4619397663f,
-0.8660254038f, -1.2552931065f, -0.3535533906f, -0.4174197128f, -1.0000000000f, -0.1913417162f, +0.0947343455f, -0.5924659585f,
-0.0000000000f, +0.5924659585f, -0.0947343455f, +0.1913417162f, +1.0000000000f, +0.4174197128f, +0.3535533906f, +1.2552931065f,
+0.8660254038f, +0.4619397663f, +1.3194792169f, +1.1827865776f, +0.5000000000f, +1.1827865776f, +1.3194792169f, +0.4619397663f,
+0.8660254038f, +1.2552931065f, +0.3535533906f, +0.4174197128f, +1.0000000000f, +0.1913417162f, -0.0947343455f, +0.5924659585f,
-0.0000000000f, -0.5924659585f, +0.0947343455f, -0.1913417162f, -1.0000000000f, -0.4174197128f, -0.3535533906f, -1.2552931065f,
-0.8660254038f, -0.4619397663f, -1.3194792169f, -1.1827865776f, -0.5000000000f, -1.1827865776f, -1.3194792169f, -0.4619397663f,
-0.8660254038f, -1.2552931065f, -0.3535533906f, -0.4174197128f, -1.0000000000f, -0.1913417162f, +0.0947343455f, -0.5924659585f,
+0.0000000000f, +0.5924659585f, -0.0947343455f, +0.1913417162f, +1.0000000000f, +0.4174197128f, +0.3535533906f, +1.2552931065f,
+0.8660254038f, +0.4619397663f, +1.3194792169f, +1.1827865776f, +0.5000000000f, +1.1827865776f, +1.3194792169f, +0.4619397663f,
+0.8660254038f, +1.2552931065f, +0.3535533906f, +0.4174197128f, +1.0000000000f, +0.1913417162f, -0.0947343455f, +0.5924659585f,
+0.0000000000f, -0.5924659585f, +0.0947343455f, -0.1913417162f, -1.0000000000f, -0.4174197128f, -0.3535533906f, -1.2552931065f,
-0.8660254038f, -0.4619397663f, -1.3194792169f, -1.1827865776f, -0.5000000000f, -1.1827865776f, -1.3194792169f, -0.4619397663f,
-0.8660254038f, -1.2552931065f, -0.3535533906f, -0.4174197128f, -1.0000000000f, -0.1913417162f, +0.0947343455f, -0.5924659585f,
};

/* ----------------------------------------------------------------------
** FIR Coefficients buffer generated using fir1() MATLAB function.
** fir1(28, 6/24)
** ------------------------------------------------------------------- */
float32_t aFIR_F32_Coeffs[NUM_TAPS] = {
-0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f, +0.0080754303f, +0.0085302217f, -0.0000000000f, -0.0173976984f,
-0.0341458607f, -0.0333591565f, +0.0000000000f, +0.0676308395f, +0.1522061835f, +0.2229246956f, +0.2504960933f, +0.2229246956f,
+0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f, -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f,
+0.0080754303f, +0.0036977508f, +0.0000000000f, -0.0015879294f, -0.0018225230f
};
/* ----------------------------------------------------------------------
** low pass at 1KHz with 40dB at 1.5KHz for SR=16KHz.
** ------------------------------------------------------------------- */

q15_t aFIR_Q15_Coeffs_LP[NUM_FIR_TAPS_Q15] = {
-217	,   40	,  120,  237,  366,  475,  527,  490,  346,
100		, -217	, -548, -818, -947, -864, -522,   86,  922,
1904	, 2918	, 3835, 4529, 4903, 4903, 4529, 3835, 2918,
1904	,  922	,   86, -522, -864, -947, -818, -548, -217,
100		,  346	,  490,  527,  475,  366,  237,  120,   40,
-217	,    0	,    0,    0,    0,    0,    0,    0,    0,
0,    0};
/* ----------------------------------------------------------------------
** high pass at 1.5KHz with 40dB at 1KHz for SR=16KHz
** ------------------------------------------------------------------- */

q15_t aFIR_Q15_Coeffs_HP[NUM_FIR_TAPS_Q15] = {
-654	,  483	,  393,  321,  222,   76, -108, -299, -447,
-501	, -422	, -200,  136,  520,  855, 1032,  953,  558,
-160	,-1148	,-2290,-3432,-4406,-5060,27477,-5060,-4406,
-3432	,-2290	,-1148, -160,  558,  953, 1032,  855,  520,
136		, -200	, -422, -501, -447, -299, -108,   76,  222,
321		,  393	,  483, -654,    0,    0,    0,    0,    0,
0			,    0	,};

/* -------------------------------------------------------------------
 * Declare State buffer of size (numTaps + blockSize - 1)
 * ------------------------------------------------------------------- */
float32_t 	firStateF32[BLOCK_SIZE + NUM_TAPS - 1];
q31_t 			firStateQ31[BLOCK_SIZE + NUM_TAPS - 1];
q15_t 			firStateQ15[NUM_FIR_TAPS_Q15 + BLOCKSIZE];
q31_t 		aFIR_Q31_Coeffs[NUM_TAPS];

// FIRs

const uint16_t Sine12bit[NB_SAMPLES] =
  {
    2047, 2447, 2831, 3185, 3498, 3750, 3939, 4056, 4095, 4056,
    3939, 3750, 3495, 3185, 2831, 2447, 2047, 1647, 1263, 909,
    599, 344, 155, 38, 0, 38, 155, 344, 599, 909, 1263, 1647,
    2047, 2447, 2831, 3185, 3498, 3750, 3939, 4056, 4095, 4056,
    3939, 3750, 3495, 3185, 2831, 2447, 2047, 1647, 1263, 909,
    599, 344, 155, 38, 0, 38, 155, 344, 599, 909, 1263, 1647
  };

float32_t aFFT_Input_f32[FFT_Length_Tab*2];
float32_t aFFT_Output_f32 [FFT_Length_Tab];

float32_t FFT_Input_Q15_f[FFT_Length_Tab*2];
q15_t aFFT_Input_Q15[FFT_Length_Tab*2];
q15_t FFT_Output_Q15[FFT_Length_Tab];

float32_t FFT_Input_Q31_f[FFT_Length_Tab*2];
q31_t aFFT_Input_Q31[FFT_Length_Tab*2];
q31_t FFT_Output_Q31[FFT_Length_Tab];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
/* USER CODE BEGIN PFP */

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
void FIR_PROCESSING_F32Process(void);
void FIR_PROCESSING_Q15Process_LP(void);
void FIR_PROCESSING_Q15Process_HP(void);
void FIR_PROCESSING_Q31Process(void);
void FFT_PROCESSING_Q15Process(uint32_t FFT_Length);
void FFT_PROCESSING_F32Process(uint32_t FFT_Length);
void FFT_PROCESSING_Q31Process(uint32_t FFT_Length);

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
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // Test USART1
//  printf("Testing USART1\n");

//  printf("aFIR_F32_Coeffs\n");
//  for (uint16_t Index=0; Index<NUM_TAPS; Index++)
//  {
//    printf("%d %.20f\n",Index, aFIR_F32_Coeffs[Index]);
//  }

//  printf("aFIR_Q15_Coeffs_LP\n");
//  for (uint16_t Index=0; Index<NUM_FIR_TAPS_Q15; Index++)
//  {
//    printf("%d %d\n",Index, aFIR_Q15_Coeffs_LP[Index]);
//  }

//  printf("aFIR_Q15_Coeffs_HP\n");
//  for (uint16_t Index=0; Index<NUM_FIR_TAPS_Q15; Index++)
//  {
//    printf("%d %d\n",Index, aFIR_Q15_Coeffs_HP[Index]);
//  }

  FIR_PROCESSING_F32Process();
//  printf("aFIR_F32_Output\n");
//  for (uint16_t Index=0; Index<TEST_LENGTH_SAMPLES; Index++)
//  {
//    printf("%d %.20f\n",Index, aFIR_F32_Output[Index]);
//  }


  FIR_PROCESSING_Q15Process_LP();
//  printf("aFIR_Q15_Output_LP\n");
//  for (uint16_t Index=0; Index<TEST_LENGTH_SAMPLES; Index++)
//  {
//    printf("%d %d\n",Index, aFIR_Q15_Output[Index]);
//  }


  FIR_PROCESSING_Q15Process_HP();
//  printf("aFIR_Q15_Output_HP\n");
//  for (uint16_t Index=0; Index<TEST_LENGTH_SAMPLES; Index++)
//  {
//    printf("%d %d\n",Index, aFIR_Q15_Output[Index]);
//  }


  FIR_PROCESSING_Q31Process();
//  printf("aFIR_Q31_Coeffs\n");
//  for (uint16_t Index=0; Index<NUM_TAPS; Index++)
//  {
//    printf("%d %d\n",Index, aFIR_Q31_Coeffs[Index]);
//  }
//  printf("aFIR_Q31_1kHz_15kHz\n");
//  for (uint16_t Index=0; Index<TEST_LENGTH_SAMPLES; Index++)
//  {
//    printf("%d %d\n",Index, aFIR_Q31_Output[Index]);
//  }

//  FFT_PROCESSING_Q15Process(64);
//  FFT_PROCESSING_Q15Process(256);
  FFT_PROCESSING_Q15Process(1024);
//
//  FFT_PROCESSING_F32Process(64);
//  FFT_PROCESSING_F32Process(256);
  FFT_PROCESSING_F32Process(1024);
//
//  FFT_PROCESSING_Q31Process(64);
//  FFT_PROCESSING_Q31Process(256);
  FFT_PROCESSING_Q31Process(1024);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	  HAL_Delay(200);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
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
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the LPUART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

/**
  * @brief  This function apply a LP FIR filter in to a F32 data signal.
  * @param  None
  * @retval None
  */
void FIR_PROCESSING_F32Process(void)
{

  arm_fir_instance_f32 FIR_F32_Struct;
  uint32_t counter_FIR_f32_p;
//  static int counter_FIR_Ds;
//  static int counter_FIR_Dd;
//  int DataID = 1;
//  uint32_t duration_us = 0x00;
  /* Call FIR init function to initialize the instance structure. */
  arm_fir_init_f32(&FIR_F32_Struct, NUM_TAPS, (float32_t *)&aFIR_F32_Coeffs[0], &firStateF32[0], blockSize);

  /* Call the FIR process function for every blockSize samples */
//  TimerCount_Start();

  for (counter_FIR_f32_p = 0; counter_FIR_f32_p < numBlocks; counter_FIR_f32_p++)
  {
    arm_fir_f32(&FIR_F32_Struct, aFIR_F32_1kHz_15kHz + (counter_FIR_f32_p * blockSize), aFIR_F32_Output + (counter_FIR_f32_p * blockSize), blockSize);
  }

//  printf("aFIR_F32_1kHz_15kHz\n");
//  for (uint16_t Index  =0; Index < TEST_LENGTH_SAMPLES; Index++)
//  {
//    printf("%d %.20f\n",Index, aFIR_F32_1kHz_15kHz[Index]);
//  }
//
//  printf("aFIR_F32_Coeffs\n");
//  for (uint16_t Index = 0; Index < NUM_TAPS; Index++)
//  {
//    printf("%d %.20f\n",Index, aFIR_F32_Coeffs[Index]);
//  }
//
//  printf("aFIR_F32_Output\n");
//  for (uint16_t Index = 0; Index < TEST_LENGTH_SAMPLES; Index++)
//  {
//    printf("%d %.20f\n",Index, aFIR_F32_Output[Index]);
//  }

//  TimerCount_Stop(nb_cycles);

//  GUI_Clear();
//  LCD_OUTPUT_Cycles(5, 305, nb_cycles);
//  duration_us = (uint32_t)(((uint64_t)US_IN_SECOND * (nb_cycles)) / SystemCoreClock);
//  LCD_OUTPUT_DURATION(120, 305, duration_us);

//  GUI_USE_PARA(DataID);

//  GRAPH_DATA_YT_AddValue(aGraph_Data[1], (aFIR_F32_1kHz_15kHz[counter_FIR_Dd])*25 + 100);
//  if (++counter_FIR_Dd == GUI_COUNTOF(aFIR_F32_1kHz_15kHz))
//  {
//    counter_FIR_Dd = 0;
//  }
//
//  GRAPH_DATA_YT_AddValue(aGraph_Data[0], (aFIR_F32_Output[counter_FIR_Ds])*20 + 50);
//  if (++counter_FIR_Ds == GUI_COUNTOF(aFIR_F32_Output))
//  {
//    counter_FIR_Ds = 0;
//  }
}

/**
  * @brief  This function apply a LP/HP FIR filter in to a Q15 data signal.
  * @param  LP or HP
  * @retval None
  */
void FIR_PROCESSING_Q15Process_LP(void)
{
  uint32_t counter_FIR_Q15_p;
  arm_fir_instance_q15 FIR_Q15_Struct;

//  static int counter_FIR_Ds;
//  static int counter_FIR_Dd;
//  int DataID = 1;
//  uint32_t duration_us = 0x00;
  arm_float_to_q15((float32_t *)&aFIR_F32_1kHz_15kHz[0], (q15_t *)&aFIR_Q15_1kHz_15kHz[0], TEST_LENGTH_SAMPLES);
  /* Call FIR init function to initialize the instance structure. */
//  if (LP_or_HP == LPF)
//  {
    arm_fir_init_q15(&FIR_Q15_Struct, NUM_FIR_TAPS_Q15, aFIR_Q15_Coeffs_LP, firStateQ15, BLOCKSIZE);
//  }
//  else if (LP_or_HP == HPF)
//  {
//    arm_fir_init_q15(&FIR_Q15_Struct, NUM_FIR_TAPS_Q15, aFIR_Q15_Coeffs_HP, firStateQ15, BLOCKSIZE);
//  }
//  else
//  {/* empty else */
//  }
//  TimerCount_Start();
  for (counter_FIR_Q15_p = 0; counter_FIR_Q15_p < numBlocks; counter_FIR_Q15_p++)
  {   // process with FIR
    arm_fir_q15(&FIR_Q15_Struct, aFIR_Q15_1kHz_15kHz + (counter_FIR_Q15_p * BLOCKSIZE), aFIR_Q15_Output + (counter_FIR_Q15_p * BLOCKSIZE), BLOCKSIZE);
  }

//  printf("aFIR_Q15_1kHz_15kHz\n");
//  for (uint16_t Index = 0; Index < TEST_LENGTH_SAMPLES; Index++)
//  {
//    printf("%d %d\n",Index, aFIR_Q15_1kHz_15kHz[Index]);
//  }
//
//  printf("aFIR_Q15_Coeffs_LP\n");
//  for (uint16_t Index = 0; Index < NUM_FIR_TAPS_Q15; Index++)
//  {
//    printf("%d %d\n",Index, aFIR_Q15_Coeffs_LP[Index]);
//  }
//
//  printf("aFIR_Q15_Output_LP\n");
//  for (uint16_t Index = 0; Index < TEST_LENGTH_SAMPLES; Index++)
//  {
//    printf("%d %d\n",Index, aFIR_Q15_Output[Index]);
//  }

//  TimerCount_Stop(nb_cycles);
//
//  GUI_Clear();
//  LCD_OUTPUT_Cycles(5, 305, nb_cycles);
//  duration_us = (uint32_t)(((uint64_t)US_IN_SECOND * (nb_cycles)) / SystemCoreClock);
//  LCD_OUTPUT_DURATION(120, 305, duration_us);
//
//  GUI_USE_PARA(DataID);
//
//  GRAPH_DATA_YT_AddValue(aGraph_Data[1], (aFIR_Q15_1kHz_15kHz[counter_FIR_Dd] / 1000) + 100);
//  if (++counter_FIR_Dd == GUI_COUNTOF(aFIR_Q15_1kHz_15kHz))
//  {
//    counter_FIR_Dd = 0;
//  }
//
//  GRAPH_DATA_YT_AddValue(aGraph_Data[0], aFIR_Q15_Output[counter_FIR_Ds] / 1350 + 50);
//  if (++counter_FIR_Ds == GUI_COUNTOF(aFIR_Q15_Output))
//  {
//    counter_FIR_Ds = 0;
//  }
}

void FIR_PROCESSING_Q15Process_HP(void)
{
  uint32_t counter_FIR_Q15_p;
  arm_fir_instance_q15 FIR_Q15_Struct;

//  static int counter_FIR_Ds;
//  static int counter_FIR_Dd;
//  int DataID = 1;
//  uint32_t duration_us = 0x00;
//  arm_float_to_q15((float32_t *)&aFIR_F32_1kHz_15kHz[0], (q15_t *)&aFIR_Q15_1kHz_15kHz[0], TEST_LENGTH_SAMPLES); // not needed as already done in FIR_PROCESSING_Q15Process_LP
  /* Call FIR init function to initialize the instance structure. */
//  if (LP_or_HP == LPF)
//  {
//    arm_fir_init_q15(&FIR_Q15_Struct, NUM_FIR_TAPS_Q15, aFIR_Q15_Coeffs_LP, firStateQ15, BLOCKSIZE);
//  }
//  else if (LP_or_HP == HPF)
//  {
    arm_fir_init_q15(&FIR_Q15_Struct, NUM_FIR_TAPS_Q15, aFIR_Q15_Coeffs_HP, firStateQ15, BLOCKSIZE);
//  }
//  else
//  {/* empty else */
//  }
//  TimerCount_Start();
  for (counter_FIR_Q15_p = 0; counter_FIR_Q15_p < numBlocks; counter_FIR_Q15_p++)
  {   // process with FIR
    arm_fir_q15(&FIR_Q15_Struct, aFIR_Q15_1kHz_15kHz + (counter_FIR_Q15_p * BLOCKSIZE), aFIR_Q15_Output + (counter_FIR_Q15_p * BLOCKSIZE), BLOCKSIZE);
  }

//  printf("aFIR_Q15_Coeffs_HP\n");
//  for (uint16_t Index = 0; Index < NUM_FIR_TAPS_Q15; Index++)
//  {
//    printf("%d %d\n",Index, aFIR_Q15_Coeffs_HP[Index]);
//  }
//
//  printf("aFIR_Q15_Output_HP\n");
//  for (uint16_t Index = 0; Index < TEST_LENGTH_SAMPLES; Index++)
//  {
//    printf("%d %d\n",Index, aFIR_Q15_Output[Index]);
//  }

//  TimerCount_Stop(nb_cycles);
//
//  GUI_Clear();
//  LCD_OUTPUT_Cycles(5, 305, nb_cycles);
//  duration_us = (uint32_t)(((uint64_t)US_IN_SECOND * (nb_cycles)) / SystemCoreClock);
//  LCD_OUTPUT_DURATION(120, 305, duration_us);
//
//  GUI_USE_PARA(DataID);
//
//  GRAPH_DATA_YT_AddValue(aGraph_Data[1], (aFIR_Q15_1kHz_15kHz[counter_FIR_Dd] / 1000) + 100);
//  if (++counter_FIR_Dd == GUI_COUNTOF(aFIR_Q15_1kHz_15kHz))
//  {
//    counter_FIR_Dd = 0;
//  }
//
//  GRAPH_DATA_YT_AddValue(aGraph_Data[0], aFIR_Q15_Output[counter_FIR_Ds] / 1350 + 50);
//  if (++counter_FIR_Ds == GUI_COUNTOF(aFIR_Q15_Output))
//  {
//    counter_FIR_Ds = 0;
//  }
}

/**
  * @brief  This function apply a LP FIR filter in to a Q31 data signal.
  * @param  None
  * @retval None
  */
void FIR_PROCESSING_Q31Process(void)
{
  uint32_t counter_FIR_Q31_p;
  arm_fir_instance_q31 FIR_Q31_Struct;

//  static int counter_FIR_Ds;
//  static int counter_FIR_Dd;
//  int DataID = 1;
//  uint32_t duration_us = 0x00;
  arm_float_to_q31((float32_t *)&aFIR_F32_Coeffs[0], (q31_t *)&aFIR_Q31_Coeffs[0], NUM_TAPS);

  arm_float_to_q31((float32_t *)&aFIR_F32_1kHz_15kHz[0], (q31_t *)&aFIR_Q31_1kHz_15kHz[0], TEST_LENGTH_SAMPLES);
  /* Call FIR init function to initialize the instance structure. */
  arm_fir_init_q31(&FIR_Q31_Struct, NUM_TAPS, (q31_t *)&aFIR_Q31_Coeffs[0], &firStateQ31[0], blockSize);

  /* Call the FIR process function for every blockSize samples  */
//  TimerCount_Start();

  for (counter_FIR_Q31_p = 0; counter_FIR_Q31_p < numBlocks; counter_FIR_Q31_p++)
  {
    arm_fir_q31(&FIR_Q31_Struct, aFIR_Q31_1kHz_15kHz + (counter_FIR_Q31_p * blockSize), aFIR_Q31_Output + (counter_FIR_Q31_p * blockSize), blockSize);
  }

//  printf("aFIR_Q31_1kHz_15kHz\n");
//  for (uint16_t Index = 0; Index < TEST_LENGTH_SAMPLES; Index++)
//  {
//    printf("%d %d\n",Index, aFIR_Q31_1kHz_15kHz[Index]);
//  }
//
//  printf("aFIR_Q31_Coeffs\n");
//  for (uint16_t Index = 0; Index < NUM_TAPS; Index++)
//  {
//    printf("%d %d\n",Index, aFIR_Q31_Coeffs[Index]);
//  }
//
//  printf("aFIR_Q31_Output\n");
//  for (uint16_t Index = 0; Index < TEST_LENGTH_SAMPLES; Index++)
//  {
//    printf("%d %d\n",Index, aFIR_Q31_Output[Index]);
//  }

//  TimerCount_Stop(nb_cycles);
//
//  GUI_Clear();
//  LCD_OUTPUT_Cycles(5, 305, nb_cycles);
//  duration_us = (uint32_t)(((uint64_t)US_IN_SECOND * (nb_cycles)) / SystemCoreClock);
//  LCD_OUTPUT_DURATION(120, 305, duration_us);
//
//  GUI_USE_PARA(DataID);
//
//  GRAPH_DATA_YT_AddValue(aGraph_Data[1], (aFIR_Q31_1kHz_15kHz[counter_FIR_Dd]) / 60092980 + 100);
//  if (++counter_FIR_Dd == GUI_COUNTOF(aFIR_Q31_1kHz_15kHz))
//  {
//    counter_FIR_Dd = 0;
//  }
//
//  GRAPH_DATA_YT_AddValue(aGraph_Data[0], aFIR_Q31_Output[counter_FIR_Ds] / 90139470 + 50);
//  if (++counter_FIR_Ds == GUI_COUNTOF(aFIR_Q31_Output))
//  {
//    counter_FIR_Ds = 0;
//  }
}

/**
  * @brief  This function Calculate FFT in Q15.
  * @param  FFT Length : 1024, 256, 64
  * @retval None
  */
void FFT_PROCESSING_Q15Process(uint32_t FFT_Length)
{

  arm_cfft_radix4_instance_q15  FFT_Q15_struct;

  q15_t maxValue;    /* Max FFT value is stored here */
  uint32_t maxIndex;    /* Index in Output array where max value is */

  uint16_t uhADCxConvertedValue = 0; // VT, simulated ADC reading but taken from DAC data Sine12bit
  uint16_t aADC1ConvertedValue_s [SAMPLES]; // VT, simulated ADC reading array but taken from DAC data Sine12bit

  uint32_t index_fill_input_buffer, index_fill_output_buffer, index_fill_adc_buffer = 0;
//  uint32_t duration_us = 0x00;

  for (index_fill_adc_buffer = 0; index_fill_adc_buffer < FFT_Length*2; index_fill_adc_buffer ++)
  {
	  uhADCxConvertedValue = Sine12bit[(index_fill_adc_buffer + NB_SAMPLES) % NB_SAMPLES]; // simulated adc reading from DAC data
    aADC1ConvertedValue_s[index_fill_adc_buffer] = uhADCxConvertedValue; // orig
//    TIM2_Config();
  }

  uint32_t cntr = 0;
  for (index_fill_input_buffer = 0; index_fill_input_buffer < FFT_Length*2; index_fill_input_buffer += 2)
  {
	  FFT_Input_Q15_f[(uint16_t)index_fill_input_buffer] = (float32_t)aADC1ConvertedValue_s[cntr++] / (float32_t)4096.0; // works alone, below 2 lines should be commented
//	  uhADCxConvertedValue = Sine12bit[((cntr++) + NB_SAMPLES) % NB_SAMPLES]; // simulated adc reading from DAC data, works together with below. Above line should be commented
//    FFT_Input_Q15_f[(uint16_t)index_fill_input_buffer] = (float32_t)uhADCxConvertedValue / (float32_t)4096.0; // orig
    /* Imaginary part */
    FFT_Input_Q15_f[(uint16_t)(index_fill_input_buffer + 1)] = 0;

//    TIM2_Config();
  }

  arm_float_to_q15((float32_t *)&FFT_Input_Q15_f[0], (q15_t *)&aFFT_Input_Q15[0], FFT_Length*2);

  /* Initialize the CFFT/CIFFT module, intFlag = 0, doBitReverse = 1 */
  arm_cfft_radix4_init_q15(&FFT_Q15_struct, FFT_Length, FFT_INVERSE_FLAG, FFT_Normal_OUTPUT_FLAG);

//  TimerCount_Start();
  arm_cfft_radix4_q15(&FFT_Q15_struct, aFFT_Input_Q15); // this will change the contents of aFFT_Input_Q15
//  TimerCount_Stop(nb_cycles);

//  combine arm_cfft_radix4_init_q15 and arm_cfft_radix4_q15 to this function (below)
//  void arm_cfft_q15	( const arm_cfft_instance_q15 * 	S,
//	  q15_t * 	p1,
//	  uint8_t 	ifftFlag,
//	  uint8_t 	bitReverseFlag
//  )
//  Parameters
//	  [in]	S	points to an instance of Q15 CFFT structure
//	  [in,out]	p1	points to the complex data buffer of size 2*fftLen. Processing occurs in-place
//	  [in]	ifftFlag	flag that selects transform direction
//		  value = 0: forward transform
//		  value = 1: inverse transform
//	  [in]	bitReverseFlag	flag that enables / disables bit reversal of output
//		  value = 0: disables bit reversal of output
//		  value = 1: enables bit reversal of output
//  Returns none


//  GUI_Clear();
//  LCD_OUTPUT_Cycles(5, 305, nb_cycles);
//  duration_us = (uint32_t)(((uint64_t)US_IN_SECOND * (nb_cycles)) / SystemCoreClock);
//  LCD_OUTPUT_DURATION(120, 305, duration_us);

  /* Process the data through the Complex Magniture Module for calculating the magnitude at each bin */
  arm_cmplx_mag_q15(aFFT_Input_Q15, FFT_Output_Q15, FFT_Length);

  /* Calculates maxValue and returns corresponding value */
  arm_max_q15(FFT_Output_Q15, FFT_Length, &maxValue, &maxIndex);
  maxValue = 0;

//  for (index_fill_input_buffer = 0; index_fill_input_buffer < FFT_Length*2; index_fill_input_buffer++)
//  {
//    GRAPH_DATA_YT_SetAlign(aGraph_Data[1], GRAPH_ALIGN_LEFT);
//    GRAPH_DATA_YT_MirrorX (aGraph_Data[1], 1);
//    GRAPH_DATA_YT_AddValue(aGraph_Data[1], aADC1ConvertedValue_s[index_fill_input_buffer] / 100 + 50);
//  }
//  for (index_fill_output_buffer = 0; index_fill_output_buffer < FFT_Length; index_fill_output_buffer++)
//  {
//    GRAPH_DATA_YT_SetAlign(aGraph_Data[0], GRAPH_ALIGN_LEFT);
//    GRAPH_DATA_YT_MirrorX (aGraph_Data[0], 1);
//    GRAPH_DATA_YT_AddValue(aGraph_Data[0], FFT_Output_Q15[index_fill_output_buffer] / 50 + 10);
//
//  }

  printf("FFT_PROCESSING_Q15Process %d samples\n", FFT_Length);
  for (uint16_t Index = 0; Index < FFT_Length*2; Index++)
  {
    printf("%d %d\n",Index, aADC1ConvertedValue_s[Index]);
  }

//  printf("FFT_Input_Q15_f\n");
//  for (uint16_t Index = 0; Index < FFT_Length*2; Index++)
//  {
//    printf("%d %.20f\n",Index, FFT_Input_Q15_f[Index]); // FFT_Input_Q15_f is float
//  }
//
//  printf("aFFT_Input_Q15\n");
//  for (uint16_t Index = 0; Index < FFT_Length*2; Index++)
//  {
//    printf("%d %d\n",Index, aFFT_Input_Q15[Index]);
//  }

  printf("FFT_Output_Q15\n");
  for (uint16_t Index = 0; Index < FFT_Length; Index++)
  {
    printf("%d %d\n",Index, FFT_Output_Q15[Index]);
  }

}

/**
  * @brief  This function Calculate FFT in F32.
  * @param  FFT Length : 1024, 256, 64
  * @retval None
  */
void FFT_PROCESSING_F32Process(uint32_t FFT_Length)
{
  arm_cfft_radix4_instance_f32  FFT_F32_struct;

  float32_t maxValue;    /* Max FFT value is stored here */
  uint32_t maxIndex;    /* Index in Output array where max value is */

  uint16_t uhADCxConvertedValue = 0; // VT, simulated ADC reading but taken from DAC data Sine12bit
  uint16_t aADC1ConvertedValue_s [SAMPLES]; // VT, simulated ADC reading array but taken from DAC data Sine12bit

  uint32_t index_fill_input_buffer, index_fill_output_buffer, index_fill_adc_buffer = 0;
//  uint32_t duration_us = 0x00;

  for (index_fill_adc_buffer = 0; index_fill_adc_buffer < FFT_Length*2; index_fill_adc_buffer ++)
  {
	  uhADCxConvertedValue = Sine12bit[(index_fill_adc_buffer + NB_SAMPLES) % NB_SAMPLES];
    aADC1ConvertedValue_s[index_fill_adc_buffer] = uhADCxConvertedValue; // orig
//    TIM2_Config();
  }

  uint32_t cntr = 0;
  for (index_fill_input_buffer = 0; index_fill_input_buffer < FFT_Length*2; index_fill_input_buffer += 2)
  {
//	  aFFT_Input_f32[(uint16_t)index_fill_input_buffer] = (float32_t)aADC1ConvertedValue_s[cntr++] / (float32_t)4096.0; // works alone, below 2 lines should be commented
	  uhADCxConvertedValue = Sine12bit[((cntr++) + NB_SAMPLES) % NB_SAMPLES]; // simulated adc reading from DAC data, works together with below. Above line should be commented
    aFFT_Input_f32[(uint16_t)index_fill_input_buffer] = (float32_t)uhADCxConvertedValue / (float32_t)4096.0; // orig
    /* Imaginary part */
    aFFT_Input_f32[(uint16_t)(index_fill_input_buffer + 1)] = 0;
//    TIM2_Config();
  }

//  printf("aFFT_Input_f32\n");
//  for (uint16_t Index = 0; Index < FFT_Length*2; Index++)
//  {
//    printf("%d %.20f\n",Index, aFFT_Input_f32[Index]);
//  }

  /* Initialize the CFFT/CIFFT module, intFlag = 0, doBitReverse = 1 */
  arm_cfft_radix4_init_f32(&FFT_F32_struct, FFT_Length, FFT_INVERSE_FLAG, FFT_Normal_OUTPUT_FLAG);

//  TimerCount_Start();
  arm_cfft_radix4_f32(&FFT_F32_struct, aFFT_Input_f32);
//  TimerCount_Stop(nb_cycles);

//  combine arm_cfft_radix4_init_f32 and arm_cfft_radix4_f32 to this function (below)
//  void arm_cfft_f32	(const arm_cfft_instance_f32 * 	S,
//  float32_t * 	p1,
//  uint8_t 	ifftFlag,
//  uint8_t 	bitReverseFlag
//  )
//  Parameters
//	  [in]	S	points to an instance of the floating-point CFFT structure
//	  [in,out]	p1	points to the complex data buffer of size 2*fftLen. Processing occurs in-place
//	  [in]	ifftFlag	flag that selects transform direction
//		  value = 0: forward transform
//		  value = 1: inverse transform
//	  [in]	bitReverseFlag	flag that enables / disables bit reversal of output
//		  value = 0: disables bit reversal of output
//		  value = 1: enables bit reversal of output
//  Returns none

//  GUI_Clear();
//  LCD_OUTPUT_Cycles(5, 305, nb_cycles);
//  duration_us = (uint32_t)(((uint64_t)US_IN_SECOND * (nb_cycles)) / SystemCoreClock);
//  LCD_OUTPUT_DURATION(120, 305, duration_us);

  /* Process the data through the Complex Magniture Module for calculating the magnitude at each bin */
  arm_cmplx_mag_f32(aFFT_Input_f32, aFFT_Output_f32, FFT_Length);

  /* Calculates maxValue and returns corresponding value */
  arm_max_f32(aFFT_Output_f32, FFT_Length, &maxValue, &maxIndex);
  maxValue = 0;

//  for (index_fill_input_buffer = 0; index_fill_input_buffer < FFT_Length*2; index_fill_input_buffer++)
//  {
//    GRAPH_DATA_YT_SetAlign(aGraph_Data[1], GRAPH_ALIGN_LEFT);
//    GRAPH_DATA_YT_MirrorX (aGraph_Data[1], 1);
//    GRAPH_DATA_YT_AddValue(aGraph_Data[1], aADC1ConvertedValue_s[index_fill_input_buffer] / 50 + 50);
//  }
//  for (index_fill_output_buffer = 0; index_fill_output_buffer < FFT_Length; index_fill_output_buffer++)
//  {
//    GRAPH_DATA_YT_SetAlign(aGraph_Data[0], GRAPH_ALIGN_LEFT);
//    GRAPH_DATA_YT_MirrorX (aGraph_Data[0], 1);
//    GRAPH_DATA_YT_AddValue(aGraph_Data[0], aFFT_Output_f32[index_fill_output_buffer] + 10);
//  }

//  printf("FFT_PROCESSING_F32Process %d samples\n", FFT_Length);
//  for (uint16_t Index = 0; Index < FFT_Length*2; Index++)
//  {
//    printf("%d %d\n",Index, aADC1ConvertedValue_s[Index]);
//  }

//  printf("aFFT_Input_f32\n");
//  for (uint16_t Index = 0; Index < FFT_Length*2; Index++)
//  {
//    printf("%d %.20f\n",Index, aFFT_Input_f32[Index]);
//  }

  printf("aFFT_Output_f32\n");
  for (uint16_t Index = 0; Index < FFT_Length*2; Index++)
  {
    printf("%d %.20f\n",Index, aFFT_Output_f32[Index]);
  }

}

/**
  * @brief  This function Calculate FFT in Q31.
  * @param  FFT Length : 1024, 256, 64
  * @retval None
  */
void FFT_PROCESSING_Q31Process(uint32_t FFT_Length)
{
  arm_cfft_radix4_instance_q31  FFT_Q31_struct;

  q31_t maxValue;    /* Max FFT value is stored here */
  uint32_t maxIndex;    /* Index in Output array where max value is */

  uint16_t uhADCxConvertedValue = 0; // VT, simulated ADC reading but taken from DAC data Sine12bit
  uint16_t aADC1ConvertedValue_s [SAMPLES]; // VT, simulated ADC reading array but taken from DAC data Sine12bit

  uint32_t index_fill_input_buffer, index_fill_output_buffer, index_fill_adc_buffer = 0;
//  uint32_t duration_us = 0x00;

  for (index_fill_adc_buffer = 0; index_fill_adc_buffer < FFT_Length*2; index_fill_adc_buffer ++)
  {
	  uhADCxConvertedValue = Sine12bit[(index_fill_adc_buffer + NB_SAMPLES) % NB_SAMPLES];
    aADC1ConvertedValue_s[index_fill_adc_buffer] = uhADCxConvertedValue;
//    TIM2_Config();
  }

  uint32_t cntr = 0;
  for (index_fill_input_buffer = 0; index_fill_input_buffer < FFT_Length*2; index_fill_input_buffer += 2)
  {
	  uhADCxConvertedValue = Sine12bit[((cntr++) + NB_SAMPLES) % NB_SAMPLES];
    FFT_Input_Q31_f[(uint16_t)index_fill_input_buffer] = (float32_t)uhADCxConvertedValue / (float32_t)4096.0;
    /* Imaginary part */
    FFT_Input_Q31_f[(uint16_t)(index_fill_input_buffer + 1)] = 0;

//    TIM2_Config();
  }

  arm_float_to_q31((float32_t *)&FFT_Input_Q31_f[0], (q31_t *)&aFFT_Input_Q31[0], FFT_Length*2);

  /* Initialize the CFFT/CIFFT module, intFlag = 0, doBitReverse = 1 */
  arm_cfft_radix4_init_q31(&FFT_Q31_struct, FFT_Length, FFT_INVERSE_FLAG, FFT_Normal_OUTPUT_FLAG);

//  TimerCount_Start();
  arm_cfft_radix4_q31(&FFT_Q31_struct, aFFT_Input_Q31);
//  TimerCount_Stop(nb_cycles);

//  GUI_Clear();
//  LCD_OUTPUT_Cycles(5, 305, nb_cycles);
//  duration_us = (uint32_t)(((uint64_t)US_IN_SECOND * (nb_cycles)) / SystemCoreClock);
//  LCD_OUTPUT_DURATION(120, 305, duration_us);

  /* Process the data through the Complex Magniture Module for calculating the magnitude at each bin */
  arm_cmplx_mag_q31(aFFT_Input_Q31, FFT_Output_Q31, FFT_Length);

  /* Calculates maxValue and returns corresponding value */
  arm_max_q31(FFT_Output_Q31, FFT_Length, &maxValue, &maxIndex);
  maxValue = 0;

//  for (index_fill_input_buffer = 0; index_fill_input_buffer < FFT_Length*2; index_fill_input_buffer++)
//  {
//    GRAPH_DATA_YT_SetAlign(aGraph_Data[1], GRAPH_ALIGN_LEFT);
//    GRAPH_DATA_YT_MirrorX (aGraph_Data[1], 1);
//    GRAPH_DATA_YT_AddValue(aGraph_Data[1], aADC1ConvertedValue_s[index_fill_input_buffer] / 50 + 50);
//  }
//  for (index_fill_output_buffer = 0; index_fill_output_buffer < FFT_Length; index_fill_output_buffer++)
//  {
//    GRAPH_DATA_YT_SetAlign(aGraph_Data[0], GRAPH_ALIGN_LEFT);
//    GRAPH_DATA_YT_MirrorX (aGraph_Data[0], 1);
//    GRAPH_DATA_YT_AddValue(aGraph_Data[0], FFT_Output_Q31[index_fill_output_buffer] / 5376212 + 10);
//
//  }

//  printf("FFT_PROCESSING_Q31Process %d samples\n", FFT_Length);
//  for (uint16_t Index = 0; Index < FFT_Length*2; Index++)
//  {
//    printf("%d %d\n",Index, aADC1ConvertedValue_s[Index]);
//  }
//
//    printf("FFT_Input_Q31_f\n");
//    for (uint16_t Index = 0; Index < FFT_Length*2; Index++)
//    {
//      printf("%d %.20f\n",Index, FFT_Input_Q31_f[Index]); // FFT_Input_Q15_f is float
//    }
//
//    printf("aFFT_Input_Q31\n");
//    for (uint16_t Index = 0; Index < FFT_Length*2; Index++)
//    {
//      printf("%d %d\n",Index, aFFT_Input_Q31[Index]);
//    }

  printf("FFT_Output_Q31\n");
  for (uint16_t Index = 0; Index < FFT_Length; Index++)
  {
    printf("%d %d\n",Index, FFT_Output_Q31[Index]);
  }


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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
