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

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static const char keypad [4][3] = {
	{'1', '2', '3'},
	{'4', '5', '6'},
	{'7', '8', '9'},
	{'*', '0', '#'}
};
static const int INPUT_PHASE = 0;
static const int DISPLAY_PHASE = 1;
static const int SLEEP_PHASE = 2;
static const int TIM3_PERIOD = 8400;
volatile int systick_flag = 0;
int systick_counter = 0;
int current_display_digit = 0;
int current_display_mode = 0;
float running_min = 5;
float running_max = 0;
float adc_value = 0.0;
int button_ticks = 0;
int button_debounce_delay = 10;
uint32_t adc_readings[1];
float raw_data[10];
float filtered_data[10];
float rms_value;
float max_value;
float min_value;
float display_rms_value;
float display_max_value;
float display_min_value;
float fir_coeff[10] = {
	-0.0490319314416,
	-0.0698589404353,
	0.0145608566286,
	0.213556898362,
	0.390773116886,
	0.390773116886,
	0.213556898362,
	0.0145608566286,
	-0.0698589404353,
	-0.0490319314416};
int current_keypad_phase = INPUT_PHASE;
char last_pressed_key = 0;
int keypad_counter = 0;
int voltage_digits[2];
int current_input_digit = 0;
int pwm = 0;
int keypad_debounce_ticks = 0;
char current_keypad_char = 0;
char last_pressed_key_debounce = 0;
int keypad_debounce_delay = 10;
int adc_counter = 0;
float desired_output_voltage = 1.6;
int pulse_width = 50;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void update_rms_and_running_max_min(void);
uint32_t voltage_to_DAC_DOR(float voltage);
void set_DAC_value(float voltage);
void display_digit(int digit);
void display_number(float num);
void display_desired_voltage(void);
void display_current_number(void);
void disable_display(void);
float fir_filter(void);
void update_raw_and_filtered_data(void);
void update_max_and_min(void);
float convert_user_input_to_desired_range(int first_digit, int second_digit);
char read_keypad_char(void);
void handle_keypad_pressed_key(char pressed_key);
void read_keypad_debounce(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/**
  * @brief  Updates the RMS, running MAX and running MIN based on the filtered data array.
  * @retval None
  */
void update_rms_and_running_max_min(void){
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

/**
  * @brief  Isolates for DOR in the following formula: DAC_OUTx = VREF+ * DOR / 4095.
  * @param  voltage: The voltage to be converted.
  * @retval The analog voltage converted to digital number.
  */
uint32_t voltage_to_DAC_DOR(float voltage)
{
	return (uint32_t)((voltage * 4095) / V_REF);
}

/**
  * @brief  Sets the DAC port to the desired voltage value.
  * @param  voltage: The voltage to set the DAC port to.
  * @retval None
  */
void set_DAC_value(float voltage)
{
	uint32_t converted_DAC_value = voltage_to_DAC_DOR(voltage);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, converted_DAC_value);
}

// 7-Segment Display
// Pins PD0 to PD7: Segments
// Pins PE0 to PE3: Digits

/**
  * @brief  Activate the required segments to display the desired digit.
  * @param  digit: The digit to display.
  * @retval None
  */
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
			HAL_GPIO_WritePin(GPIOD, SegmentD_Pin|SegmentE_Pin|SegmentF_Pin|SegmentG_Pin, GPIO_PIN_RESET);
			break;
		case 8:
			HAL_GPIO_WritePin(GPIOD, SegmentA_Pin|SegmentB_Pin|SegmentC_Pin|SegmentD_Pin 
                          |SegmentE_Pin|SegmentF_Pin|SegmentG_Pin, GPIO_PIN_SET);
			break;
		case 9:
			HAL_GPIO_WritePin(GPIOD, SegmentA_Pin|SegmentB_Pin|SegmentC_Pin|SegmentF_Pin|SegmentG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SegmentD_Pin|SegmentE_Pin, GPIO_PIN_RESET);
			break;
	}
}

/**
  * @brief  Displays a 3-digit number (ranging from 0.00 to 9.99) on the 7 segment display.
  * @param  num: The number to display.
  * @retval None
  */
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

/**
  * @brief  Displays the desired voltage on the 7 segment display.
  * @retval None
  */
void display_desired_voltage()
{
	switch(current_display_digit)
	{
		case 1:
			HAL_GPIO_WritePin(GPIOE, Digit1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, Digit0_Pin|Digit2_Pin|Digit3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SegmentDP_Pin, GPIO_PIN_SET); // Set decimal point
			display_digit(voltage_digits[0]);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOE, Digit2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, Digit0_Pin|Digit1_Pin|Digit3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SegmentDP_Pin, GPIO_PIN_RESET);
			display_digit(voltage_digits[1]);
			break;
	}
	current_display_digit = (current_display_digit + 1) % 4;
}

/**
  * @brief  Displays either the RMS, MAX or MIN value depending on the current mode.
  * @retval None
  */
void display_current_number(void)
{	
	if (current_keypad_phase == DISPLAY_PHASE)
	{
		switch(current_display_mode)
		{
			case 0:
				// Display RMS value
				display_number(display_rms_value);
				break;
			case 1:
				// Display MAX value
				display_number(display_max_value);
				break;
			case 2:
				// Display MIN value
				display_number(display_min_value);
				break;
		}
	}
	else if (current_keypad_phase == INPUT_PHASE)
	{
		display_desired_voltage();
	}
}

/**
  * @brief  Turns off the 7 segment display.
  * @retval None
  */
void disable_display(void)
{
	HAL_GPIO_WritePin(GPIOE, Digit0_Pin|Digit1_Pin|Digit2_Pin|Digit3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, SegmentA_Pin|SegmentB_Pin|SegmentC_Pin|SegmentD_Pin|SegmentE_Pin|SegmentF_Pin|SegmentG_Pin|SegmentDP_Pin, GPIO_PIN_SET);
}

/**
  * @brief  Processes the raw ADC data with an FIR filter, returning the filtered value.
  * @retval The filtered value based on the previous ADC readings.
  */
float fir_filter(void)
{
	int order = 10;
	float sum = 0;
	for(int i = 0; i<order;i++){
		sum+=fir_coeff[i]*raw_data[i];
	}
	return sum;
}

/**
  * @brief  Updates the raw data vector and the filtered data vector.
  * @retval None
  */
void update_raw_and_filtered_data(void)
{	
	// Shift raw data array up
	for (int i = 1; i < 10; i++)
	{
		raw_data[i] = raw_data[i - 1];
	}
	
	// Sample raw ADC data
	raw_data[0] = (adc_readings[0] / 1023.0) * V_REF;
	
	// Shift filtered data array up
	for (int i = 1; i < 10; i++)
	{
		filtered_data[i] = filtered_data[i - 1];
	}
	
	// Update filtered data
	filtered_data[0] = fir_filter();
}

/**
  * @brief  Updates the max and min values.
  * @retval None
  */
void update_max_and_min(void)
{
	// Update MAX and MIN values
	max_value = running_max;
	min_value = running_min;
	
	// Reset running MIN and MAX values
	running_max = 0;
	running_min = 5;
}

/**
  * @brief  Converts the user input desired voltage to a voltage within the allowable range.
  * @param  first_digit: The first digit of the desired voltage.
  * @param  second_digit: The second digit of the desired voltage.
  * @retval the converted voltage
  */
float convert_user_input_to_desired_range(int first_digit, int second_digit)
{
	float converted_number = first_digit + 0.1f * second_digit;
	if (converted_number > 2.3f)
	{
		converted_number = 2.3f;
	}
	else if (converted_number < 0)
	{
		converted_number = 0;
	}
	return converted_number;
}

/**
  * @brief  Reads the currently pressed keypad character.
  * @retval the pressed character, or 0 if no character is pressed
  */
char read_keypad_char(void)
{
	for (int row = 0; row < 4; row++)
	{
		switch(row)
		{
			case 0:
				HAL_GPIO_WritePin(GPIOE, ROW0_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE, ROW1_Pin|ROW2_Pin|ROW3_Pin, GPIO_PIN_RESET);
				break;
			case 1:
				HAL_GPIO_WritePin(GPIOE, ROW1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE, ROW0_Pin|ROW2_Pin|ROW3_Pin, GPIO_PIN_RESET);
				break;
			case 2:
				HAL_GPIO_WritePin(GPIOE, ROW2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE, ROW0_Pin|ROW1_Pin|ROW3_Pin, GPIO_PIN_RESET);
				break;
			case 3:
				HAL_GPIO_WritePin(GPIOE, ROW3_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE, ROW0_Pin|ROW1_Pin|ROW2_Pin, GPIO_PIN_RESET);
				break;
		}
		if (HAL_GPIO_ReadPin(GPIOE, COL1_Pin))
		{
			return keypad[row][1];
		}
		else if (HAL_GPIO_ReadPin(GPIOE, COL0_Pin))
		{
			return keypad[row][0];
		}
		else if (HAL_GPIO_ReadPin(GPIOE, COL2_Pin))
		{
			return keypad[row][2];
		}
	}
	return 0;
}

/**
  * @brief  Changes the state of the system based on the currently pressed key.
  * @param  pressed_key: The currently pressed keypad key.
  * @retval None
  */
void handle_keypad_pressed_key(char pressed_key)
{
	if (pressed_key != 0)
	{
		if (pressed_key == '*' && pressed_key == last_pressed_key)
		{
			keypad_counter++;
			// Press for 3 s
			if (keypad_counter >= 600)
			{
				// Enter sleep phase
				current_keypad_phase = SLEEP_PHASE;
				disable_display();
				keypad_counter = 0;
			}
		}
		else
		{
			keypad_counter = 0;
		}
	}
	else
	{
		if (last_pressed_key != 0)
		{
			if (last_pressed_key == '*')
			{
				// Press for >= 1 s
				if (keypad_counter >= 200)
				{
					// Enter input phase
					current_keypad_phase = INPUT_PHASE;
					disable_display();
				}
				else if (current_keypad_phase == INPUT_PHASE)
				{
					// Delete last digit
					if (voltage_digits[1] != 0)
					{
						voltage_digits[1] = 0;
						current_input_digit = 1;
					}
					else
					{
						voltage_digits[0] = 0;
						current_input_digit = 0;
					}
					desired_output_voltage = convert_user_input_to_desired_range(voltage_digits[0], voltage_digits[1]);
				}
			}
			else if (current_keypad_phase == INPUT_PHASE)
			{
				if (last_pressed_key == '#')
				{
					// Enter display phase
					current_keypad_phase = DISPLAY_PHASE;
				}
				else if (last_pressed_key != '*' && last_pressed_key != '#')
				{
					// User Input
					voltage_digits[current_input_digit] = last_pressed_key - '0';
					current_input_digit = (current_input_digit + 1) % 2;
					desired_output_voltage = convert_user_input_to_desired_range(voltage_digits[0], voltage_digits[1]);
				}
			}
		}
		keypad_counter = 0;
	}
	last_pressed_key = pressed_key;
}

/**
  * @brief  Reads the currently pressed keypad key with debouncing.
  * @retval None
  */
void read_keypad_debounce(void)
{
	char pressed_key = read_keypad_char();
	if (pressed_key != last_pressed_key_debounce)
	{
		keypad_debounce_ticks = 0;
	}
	if (keypad_debounce_ticks > keypad_debounce_delay)
	{
			handle_keypad_pressed_key(pressed_key);
	}
	last_pressed_key_debounce = pressed_key;
	keypad_debounce_ticks++;
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
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	if (HAL_ADC_Start(&hadc1) != HAL_OK)
	{
		printf("HAL ADC failed.\n");
		return 0;
	}
	if (HAL_ADC_Start_DMA(&hadc1, adc_readings, 1) != HAL_OK)
	{
		printf("HAL ADC DMA failed.\n");
		return 0;
	}
	HAL_ADC_Start_IT(&hadc1);
	GPIO_PinState last_button_state = GPIO_PIN_RESET;
	GPIO_PinState button_state = GPIO_PIN_RESET;
	set_DAC_value(1.5); // Set the DAC voltage (PA4)
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
				}
			}
		}
		last_button_state = reading;
		
		// Every 5 ms
		if (systick_flag)
		{
			// Reset flag
			systick_flag = 0;
			
			// Display on 7 segment display
			display_current_number();
			
			// Update counters
			button_ticks++;
			
			read_keypad_debounce();
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
	if (current_keypad_phase == DISPLAY_PHASE)
	{
		adc_counter = (adc_counter + 1) % 1000000;
		
		// Every 0.1 ms
		update_raw_and_filtered_data();
		
		// Every 1 ms
		if (adc_counter % 10 == 0)
		{
			// Update RMS value and running MAX and running MIN
			update_rms_and_running_max_min();
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
