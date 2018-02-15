/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "arm_math.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile int systick_flag = 0;
int systick_counter = 0;
int current_display_digit = 0;
int current_display_mode = 0;
float running_min = 5;
float running_max = 0;
float ADC_value = 0.0;
int button_ticks = 0;
int button_debounce_delay = 10;
uint32_t ADCReadings[1]; //ADC Readings
float raw_data[10];
float filtered_data[10];
float rms_value;
float max_value;
float min_value;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
// Handle ADC input signal

void update_rms_and_running_max_min(){
	// RMS
	arm_rms_f32(filtered_data, 10, &rms_value);
	// MAX & INDEX
	float32_t max;
	uint32_t maxIndex;
	arm_max_f32(filtered_data, 10, &max, &maxIndex);
	// MIN & INDEX
	float32_t min;
	uint32_t minIndex;
	arm_min_f32(filtered_data, 10, &min, &minIndex);
	if(running_max<max){
		running_max = max;
	}
	if(running_min>min){
		running_min = min;
	}
}

// Isolates for DOR in the following formula: DAC_OUTx = VREF+ * DOR / 4095.
uint32_t voltage_to_ADC_DOR(float voltage)
{
	return (uint32_t)((voltage * 4095) / V_REF);
}

void set_DAC_value(float voltage)
{
	uint32_t converted_DAC_value = voltage_to_ADC_DOR(voltage);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, converted_DAC_value);
}

// 7-Segment Display
// Pins PD0 to PD7: Segments
// Pins PE0 to PE3: Digits


void display_digit(int digit)
{
	switch(digit)
	{
		case 0:
			HAL_GPIO_WritePin(GPIOD, SegmentA_Pin|SegmentB_Pin|SegmentC_Pin|SegmentD_Pin 
                          |SegmentE_Pin|SegmentF_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SegmentG_Pin, GPIO_PIN_RESET);
			break;
		case 1:
			HAL_GPIO_WritePin(GPIOD, SegmentB_Pin|SegmentC_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SegmentA_Pin|SegmentD_Pin 
                          |SegmentE_Pin|SegmentF_Pin|SegmentG_Pin, GPIO_PIN_RESET);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOD, SegmentA_Pin|SegmentB_Pin|SegmentD_Pin 
                          |SegmentE_Pin|SegmentG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SegmentC_Pin|SegmentF_Pin, GPIO_PIN_RESET);
			break;
		case 3:
			HAL_GPIO_WritePin(GPIOD, SegmentA_Pin|SegmentB_Pin|SegmentC_Pin|SegmentD_Pin 
                          |SegmentG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SegmentE_Pin|SegmentF_Pin, GPIO_PIN_RESET);
			break;
		case 4:
			HAL_GPIO_WritePin(GPIOD, SegmentB_Pin|SegmentC_Pin|SegmentF_Pin|SegmentG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SegmentA_Pin|SegmentD_Pin|SegmentE_Pin, GPIO_PIN_RESET);
			break;
		case 5:
			HAL_GPIO_WritePin(GPIOD, SegmentA_Pin|SegmentC_Pin|SegmentD_Pin 
                          |SegmentF_Pin|SegmentG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SegmentB_Pin|SegmentE_Pin, GPIO_PIN_RESET);
			break;
		case 6:
			HAL_GPIO_WritePin(GPIOD, SegmentA_Pin|SegmentC_Pin|SegmentD_Pin 
                          |SegmentE_Pin|SegmentF_Pin|SegmentG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SegmentB_Pin, GPIO_PIN_RESET);
			break;
		case 7:
			HAL_GPIO_WritePin(GPIOD, SegmentA_Pin|SegmentB_Pin|SegmentC_Pin|SegmentD_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SegmentD_Pin|SegmentE_Pin|SegmentF_Pin|SegmentG_Pin|SegmentDP_Pin, GPIO_PIN_RESET);
			break;
		case 8:
			HAL_GPIO_WritePin(GPIOD, SegmentA_Pin|SegmentB_Pin|SegmentC_Pin|SegmentD_Pin 
                          |SegmentE_Pin|SegmentF_Pin|SegmentG_Pin|SegmentDP_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SegmentDP_Pin, GPIO_PIN_RESET);
			break;
		case 9:
			HAL_GPIO_WritePin(GPIOD, SegmentA_Pin|SegmentB_Pin|SegmentC_Pin|SegmentF_Pin|SegmentG_Pin|SegmentDP_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SegmentD_Pin|SegmentE_Pin|SegmentDP_Pin, GPIO_PIN_RESET);
			break;
	}
}

// display a 3 digit number
void display_number(float num)
{
	switch(current_display_digit)
	{
		case 0:
			HAL_GPIO_WritePin(GPIOE, Digit0_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, Digit1_Pin|Digit2_Pin|Digit3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SegmentDP_Pin, GPIO_PIN_RESET);
			display_digit(current_display_mode);
			break;
		case 1:
			HAL_GPIO_WritePin(GPIOE, Digit1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, Digit0_Pin|Digit2_Pin|Digit3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SegmentDP_Pin, GPIO_PIN_SET); // Set decimal point
			display_digit((int) num);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOE, Digit2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, Digit0_Pin|Digit1_Pin|Digit3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SegmentDP_Pin, GPIO_PIN_RESET);
			display_digit(((int)(num * 10) % 10));
			break;
		case 3:
			HAL_GPIO_WritePin(GPIOE, Digit3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, Digit0_Pin|Digit1_Pin|Digit2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SegmentDP_Pin, GPIO_PIN_RESET);
			display_digit(((int)(num * 100) % 10));
			break;
	}
	current_display_digit = (current_display_digit + 1) % 4;
}

void display_current_number()
{
	switch(current_display_mode)
	{
		case 0:
			display_number(rms_value);
			break;
		case 1:
			display_number(max_value);
			break;
		case 2:
			display_number(min_value);
			break;
	}
}

void display_all_on()
{
  HAL_GPIO_WritePin(GPIOD, SegmentA_Pin|SegmentB_Pin|SegmentC_Pin|SegmentD_Pin 
                          |SegmentE_Pin|SegmentF_Pin|SegmentG_Pin|SegmentDP_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, Digit2_Pin|Digit3_Pin|Digit0_Pin|Digit1_Pin, GPIO_PIN_RESET);
}


void display_all_off()
{
  HAL_GPIO_WritePin(GPIOD, SegmentA_Pin|SegmentB_Pin|SegmentC_Pin|SegmentD_Pin 
                          |SegmentE_Pin|SegmentF_Pin|SegmentG_Pin|SegmentDP_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, Digit2_Pin|Digit3_Pin|Digit0_Pin|Digit1_Pin, GPIO_PIN_RESET);
}

float FIR_filter()
{
	// TODO: Complete simple filter
	float coeff[10] = {1,1,1,1,1,1,1,1,1,1};
	int order = 10;
	float output = 0;
	float sum = 0;
	for(int i = 0; i<order;i++){
		sum+=coeff[i]*raw_data[i];
	}
	return sum;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 200); // Set SysTick interrupt frequency to 200 Hz (5 ms)

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  /* USER CODE BEGIN 2 */
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	if (HAL_ADC_Start(&hadc1) != HAL_OK)
	{
		printf("HAL ADC failed.\n");
		return 0;
	}
	if (HAL_ADC_Start_DMA(&hadc1, ADCReadings, 1) != HAL_OK)
	{
		printf("HAL ADC DMA failed.\n");
		return 0;
	}
	GPIO_PinState last_button_state = GPIO_PIN_RESET;
	GPIO_PinState button_state = GPIO_PIN_RESET;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		// Read the B1 button (PA0) with debouncing
		// Debounce inspired from https://www.arduino.cc/en/Tutorial/Debounce
		GPIO_PinState reading = HAL_GPIO_ReadPin(Button_GPIO_Port, Button_Pin);
		if(reading != last_button_state)
		{
			button_ticks = 0;
		}
		if(button_ticks > button_debounce_delay)
		{
			if (reading != button_state) {
				button_state = reading;
				
				if (button_state)
				{
					current_display_mode = (current_display_mode + 1) % 3;
					printf("Mode: %i\n", current_display_mode);
				}
			}
		}
		last_button_state = reading;
		
		// Every 5 ms
		if (systick_flag)
		{
			// Set the DAC voltage (PA4)
			set_DAC_value(1.5);
			
			// Display on 7 segment display
			display_current_number();
			
			// Update counters
			button_ticks++;
			systick_counter = (systick_counter + 1) % 2000;
			
			// Reset flag
			systick_flag = 0;
		}
		
		// Every 20 ms
		if (systick_counter % 4 == 0)
		{
			// Shift raw data array up
			for (int i = 1; i < 10; i++)
			{
				raw_data[i] = raw_data[i - 1];
			}
			
			// Sample raw ADC data
			raw_data[0] = (ADCReadings[0] / 1023.0) * V_REF;
			
			// Shift filtered data array up
			for (int i = 1; i < 10; i++)
			{
				filtered_data[i] = filtered_data[i - 1];
			}
			
			// Update filtered data
			filtered_data[0] = FIR_filter();
		}
		
		// Every 200 ms
		if (systick_counter % 40 == 0)
		{
			// Update RMS value and running MAX and running MIN
			update_rms_and_running_max_min();
		}
		
		// Every 10 s
		if (systick_counter == 0)
		{
			// Update MAX and MIN values
			max_value = running_max;
			min_value = running_min;
			
			// Reset running MIN and MAX values
			running_max = 0;
			running_min = 5;
		}
		

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, Digit2_Pin|Digit3_Pin|Digit0_Pin|Digit1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, SegmentA_Pin|SegmentB_Pin|SegmentC_Pin|SegmentD_Pin 
                          |SegmentE_Pin|SegmentF_Pin|SegmentG_Pin|SegmentDP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Digit2_Pin Digit3_Pin Digit0_Pin Digit1_Pin */
  GPIO_InitStruct.Pin = Digit2_Pin|Digit3_Pin|Digit0_Pin|Digit1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SegmentA_Pin SegmentB_Pin SegmentC_Pin SegmentD_Pin 
                           SegmentE_Pin SegmentF_Pin SegmentG_Pin SegmentDP_Pin */
  GPIO_InitStruct.Pin = SegmentA_Pin|SegmentB_Pin|SegmentC_Pin|SegmentD_Pin 
                          |SegmentE_Pin|SegmentF_Pin|SegmentG_Pin|SegmentDP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
