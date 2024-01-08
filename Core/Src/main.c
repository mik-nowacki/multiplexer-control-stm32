/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "math.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SELECT_PINS_PORT GPIOE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */
const int MUX1 = 0;
const int MUX2 = 1;
const int MUX3 = 2;
const int MUX_SELECT_PINS[3][3] = {{M1_S0_Pin, M1_S1_Pin, M1_S2_Pin}, {M1_S0_Pin, M1_S1_Pin, M1_S2_Pin}, {M1_S0_Pin, M1_S1_Pin, M1_S2_Pin}};
const int NUMBER_OF_SENSORS[3] = {4, 4 ,4};
const int NUMBER_OF_RESISTORS = 8;
const int MAX_RESOLUTION_VALUE = 4095;
const double VOLT_IN = 2.95;
const int RESISTOR_47 = 4700; // resistor 4.7 kOhm
const int RESISTOR_22 = 2200; // resistor 2.2 kOhm
//duzy konduktancja
//	  double a = 0.06828695044280314;
//	  double b = 1.430615164520744;
//mały konduktancja działa :)
const double A = 0.21618162974535243;
const double B = 1.404494382022472;

uint16_t analogRead = 0;
double voltageAOC = 0.0;
double current = 0.0;
double sensorResistance = 0.0;
double conductance = 0.0;
double grams = 0.0;
double forceArray[8] = {0, 0, 0, 0, 0, 0, 0, 0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

void selectMuxPin(int pin, int mux) {
    for (int i = 0; i < 8; i++) {
        if (pin & (1 << i))
            HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][i], GPIO_PIN_SET);
        else
            HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][i], GPIO_PIN_RESET);
    }

}


void switchMuxPin(int mux, int pin)
{
	switch (pin){
	case 0:
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][0], GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][1], GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][2], GPIO_PIN_RESET);
		break;
	case 1:
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][0], GPIO_PIN_SET);
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][1], GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][2], GPIO_PIN_RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][0], GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][1], GPIO_PIN_SET);
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][2], GPIO_PIN_RESET);
		break;
	case 3:
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][0], GPIO_PIN_SET);
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][1], GPIO_PIN_SET);
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][2], GPIO_PIN_RESET);
		break;
	case 4:
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][0], GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][1], GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][2], GPIO_PIN_SET);
		break;
	case 5:
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][0], GPIO_PIN_SET);
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][1], GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][2], GPIO_PIN_SET);
		break;
	case 6:
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][0], GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][1], GPIO_PIN_SET);
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][2], GPIO_PIN_SET);
		break;
	case 7:
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][0], GPIO_PIN_SET);
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][1], GPIO_PIN_SET);
		HAL_GPIO_WritePin(SELECT_PINS_PORT, MUX_SELECT_PINS[mux][2], GPIO_PIN_SET);
		break;
	}

}


double calculateForce(uint16_t _analogRead, int _resistor)
{
//  convert analogRead to Volt
	voltageAOC = (double)_analogRead * (VOLT_IN / MAX_RESOLUTION_VALUE);  // 500mV max value
	current = voltageAOC / _resistor;
	sensorResistance = (VOLT_IN / current) - _resistor;
	conductance = 1000000 / sensorResistance;

	return ((A * pow(conductance, B)) / 1000) * 9.81; // return value in [N]
}


void readMultiplexer(int mux, int yFirst, int yLast)
{
	for (int pin = yFirst; pin < yLast; pin++)
	{
	  switchMuxPin(mux, pin);

	  HAL_ADC_Start(&hadc1); // start the ADC

	  if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)  // poll for conversion
		  {
		  analogRead = HAL_ADC_GetValue(&hadc1);
		  }

	  HAL_ADC_Stop(&hadc1);

	  forceArray[pin] = calculateForce(analogRead, RESISTOR_47);
	}
}
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
  int mux1LastPinIndex = NUMBER_OF_SENSORS[MUX1];
  int mux2LastPinIndex = NUMBER_OF_SENSORS[MUX1] + NUMBER_OF_SENSORS[MUX2];
  int mux3LastPinIndex = NUMBER_OF_SENSORS[MUX1] + NUMBER_OF_SENSORS[MUX2] + NUMBER_OF_SENSORS[MUX3];
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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	  Read multiplexer pins
	  readMultiplexer(MUX1, 0, mux1LastPinIndex);
	  readMultiplexer(MUX2, mux1LastPinIndex, mux2LastPinIndex);
	  readMultiplexer(MUX3, mux2LastPinIndex, mux3LastPinIndex);

	  HAL_Delay(100);
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|M1_S0_Pin|M1_S1_Pin|M1_S2_Pin
                          |M2_S0_Pin|M2_S1_Pin|M2_S2_Pin|M3_S0_Pin
                          |M3_S1_Pin|M3_S2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_I2C_SPI_Pin M1_S0_Pin M1_S1_Pin M1_S2_Pin
                           M2_S0_Pin M2_S1_Pin M2_S2_Pin M3_S0_Pin
                           M3_S1_Pin M3_S2_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|M1_S0_Pin|M1_S1_Pin|M1_S2_Pin
                          |M2_S0_Pin|M2_S1_Pin|M2_S2_Pin|M3_S0_Pin
                          |M3_S1_Pin|M3_S2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
