/*******************************************************************************
  * @file    main.c
  * @author  Sean Stappas and Eric Vuong
	* @version V1.0.0
  * @date    10-March-2018
  * @brief   Main file for Lab 4.
  ******************************************************************************
  */

#include "main.h"
#include "stm32f4xx_hal.h"              // Keil::Device:STM32Cube HAL:Common
#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX
#include "RTE_Components.h"             // Component selection

/* USER CODE BEGIN Includes */
#include "button.h"
#include "display.h"
#include "filter.h"
#include "keypad.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile int systick_flag = 0;
volatile int current_phase = INPUT_PHASE;
volatile int current_display_mode = DISPLAY_MODE_RMS;
volatile int voltage_digits[2];
volatile float display_rms_value;
volatile float display_max_value;
volatile float display_min_value;
volatile float desired_output_voltage;
volatile float rms_value;
volatile float max_value;
volatile float min_value;
uint32_t adc_readings[1];
int systick_counter = 0;
int pwm = 0;
int adc_counter = 0;
int pulse_width = 50;
uint32_t adc_readings[1];
int last_phase = INPUT_PHASE;
osThreadId thread_id_button;
osThreadId thread_id_display;
osThreadId thread_id_keypad;
osThreadId thread_id_check_sleep;
/* USER CODE END PV */

//The thread code is written in Thread_LED.c, just telling the toolchain that the 
//functions are declared externally
void SystemClock_Config(void);
GPIO_InitTypeDef 				LED_configuration;
/**
	These lines are mandatory to make CMSIS-RTOS RTX work with te new Cube HAL
*/
#ifdef RTE_CMSIS_RTOS_RTX
extern uint32_t os_time;

uint32_t HAL_GetTick(void) { 
  return os_time; 
}
#endif

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void adc_callback(void);
void pwm_feedback_control(void);
void enter_low_power_mode(void);
void exit_low_power_mode(void);
void start_peripherals(void);
void stop_peripherals(void);
void check_sleep_phase_transition(void);
void thread_button (void const *argument);
void thread_display (void const *argument);
void thread_keypad (void const *argument);
void thread_check_sleep (void const *argument);
void restart_suspended_threads(void);
osThreadDef(thread_button, osPriorityNormal, 1, 0);
osThreadDef(thread_display, osPriorityNormal, 1, 0);
osThreadDef(thread_keypad, osPriorityNormal, 1, 0);
osThreadDef(thread_check_sleep, osPriorityNormal, 1, 0);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/**
  * @brief  Update the pulse width to obtain the desired output voltage.
  * @retval None
  */
void pwm_feedback_control(void)
{
	float diff = rms_value - desired_output_voltage;
	int pulse_delta = 0;
	if (diff >= 1 || diff <= -1)
	{
		pulse_delta = 20;
	}
	else if (diff >= 0.5f || diff <= -0.5f)
	{
		pulse_delta = 15;
	}
	else if (diff >= 0.1f || diff <= -0.1f)
	{
		pulse_delta = 10;
	}
	else if (diff >= 0.05f || diff <= -0.05f)
	{
		pulse_delta = 5;
	}
	else if (diff >= 0.01f || diff <= -0.01f)
	{
		pulse_delta = 2;
	}
	else if (diff >= 0.005f || diff <= -0.005f)
	{
		pulse_delta = 1;
	}
	if (diff > 0)
	{
		if (pulse_width >= pulse_delta)
		{
			pulse_width -= pulse_delta;
		}
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulse_width);
	}
	else if (diff < 0)
	{
		if (pulse_width <= TIM3_PERIOD - pulse_delta)
		{
			pulse_width += pulse_delta;
		}
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulse_width);
	}
}

/**
  * @brief  Callback when the ADC is finished converting a value.
  * @retval None
  */
void adc_callback(void) // 10 kHz (every 0.1 ms)
{
	if (current_phase == DISPLAY_PHASE)
	{
		adc_counter = (adc_counter + 1) % 1000000;
		
		// Every 0.1 ms
		update_raw_and_filtered_data(adc_readings[0]);
		
		// Every 1 ms
		if (adc_counter % 10 == 0)
		{
			update_rms_and_running_max_min();
			pwm_feedback_control();
		}
		
		// Every 50 ms
		if (adc_counter % 500 == 0)
		{
			update_max_and_min();
			display_rms_value = rms_value;
			display_max_value = max_value;
			display_min_value = min_value;
		}
	}
}

/**
  * @brief  Enter low power mode (during SLEEP phase).
  * @retval None
  */
void enter_low_power_mode(void)
{
	disable_display();
	stop_peripherals();
}

/**
  * @brief  Exit low power mode (not SLEEP phase).
  * @retval None
  */
void exit_low_power_mode(void)
{
	start_peripherals();
	restart_suspended_threads();
}

/**
  * @brief  Start all peripherals.
  * @retval None
  */
void start_peripherals(void)
{
	// Initialize peripherals
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
	
	// Start ADC
	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, adc_readings, 1);
	HAL_ADC_Start_IT(&hadc1);
	
	// Start TIM 2 for ADC readings
	HAL_TIM_Base_Start(&htim2);
	
	// Start TIM 3 for PWM
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
}

/**
  * @brief  Stop all peripherals.
  * @retval None
  */
void stop_peripherals(void)
{
	HAL_ADC_DeInit(&hadc1);
	HAL_TIM_Base_DeInit(&htim2);
	HAL_TIM_PWM_DeInit(&htim3);
}

/**
  * @brief  Check if a transition has occurred to/from SLEEP phase. This 
	*					causes a switch to/from low power mode, respectively.
  * @retval None
  */
void check_sleep_phase_transition(void)
{
	if (current_phase == SLEEP_PHASE && last_phase != SLEEP_PHASE)
	{
		enter_low_power_mode();
	}
	else if (current_phase != SLEEP_PHASE && last_phase == SLEEP_PHASE)
	{
		exit_low_power_mode();
	}
	last_phase = current_phase;
}
/**
  * @brief  Start all the needed threads.
  * @retval None
  */
void start_threads (void)
{
  thread_id_button = osThreadCreate(osThread(thread_button), NULL); 
  thread_id_display = osThreadCreate(osThread(thread_display), NULL); 
  thread_id_keypad = osThreadCreate(osThread(thread_keypad), NULL);
  thread_id_check_sleep = osThreadCreate(osThread(thread_check_sleep), NULL);
}

/**
  * @brief  Restarts threads waiting in SLEEP mode.
  * @retval None
  */
void restart_suspended_threads(void)
{
	osSignalSet(thread_id_button, EXIT_SLEEP_SIGNAL);
	osSignalSet(thread_id_display, EXIT_SLEEP_SIGNAL);
}

/**
  * @brief  Starts the thread to read the blue button.
  * @retval None
  */
void thread_button(void const *argument)
{
	while(1)
	{
		if (current_phase == SLEEP_PHASE)
		{
			osSignalWait(EXIT_SLEEP_SIGNAL, osWaitForever);
		}
		osDelay(5);
		read_button_debounce();
	}
}

/**
  * @brief  Starts the thread controlling the 7-segment display.
  * @retval None
  */
void thread_display(void const *argument)
{
	while(1)
	{
		if (current_phase == SLEEP_PHASE)
		{
			disable_display();
			osSignalWait(EXIT_SLEEP_SIGNAL, osWaitForever);
		}
		osDelay(5);
		display_current_number();
	}
}

/**
  * @brief  Starts the thread controlling the keypad.
  * @retval None
  */
void thread_keypad(void const *argument)
{
	while(1)
	{
		osDelay(5);
		read_keypad_debounce();
	}
}

/**
  * @brief  Starts the thread to check for a transition to or from sleep mode.
  * @retval None
  */
void thread_check_sleep(void const *argument)
{
	while(1)
	{
		osDelay(5);
		check_sleep_phase_transition();
	}
}
/* USER CODE END 0 */

/**
  * Main function
  */
int main (void)
{
	/* USER CODE BEGIN 1 */
  osKernelInitialize();                     /* initialize CMSIS-RTOS          */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	start_peripherals();
	start_threads();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	osKernelStart();                          /* start thread execution         */
  /* USER CODE END 3 */
}

/**
  * System Clock Configuration
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
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
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

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 839;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = TIM3_PERIOD;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pulse_width;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, Digit2_Pin|Digit3_Pin|ROW0_Pin|ROW1_Pin 
                          |ROW2_Pin|ROW3_Pin|Digit0_Pin|Digit1_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : COL0_Pin COL1_Pin COL2_Pin */
  GPIO_InitStruct.Pin = COL0_Pin|COL1_Pin|COL2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : ROW0_Pin ROW1_Pin ROW2_Pin ROW3_Pin */
  GPIO_InitStruct.Pin = ROW0_Pin|ROW1_Pin|ROW2_Pin|ROW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

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
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) // 10 kHz (every 0.1 ms)
{
	adc_callback();
}
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
