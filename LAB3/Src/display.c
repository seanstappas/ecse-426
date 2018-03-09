#include "stm32f4xx_hal.h"
#include "display.h"

// 7-Segment Display
// Pins PD0 to PD7: Segments
// Pins PE0 to PE3: Digits

int current_display_digit = 0;

void display_digit(int digit);
void display_number(float num);
void display_desired_voltage(void);

/**
  * @brief  Displays either the RMS, MAX or MIN value depending on the current mode.
  * @retval None
  */
void display_current_number(void)
{	
	switch(current_phase)
	{
		case INPUT_PHASE:
			display_desired_voltage();
			break;
		case DISPLAY_PHASE:
			switch(current_display_mode)
			{
				case DISPLAY_MODE_RMS:
					display_number(display_rms_value);
					break;
				case DISPLAY_MODE_MAX:
					display_number(display_max_value);
					break;
				case DISPLAY_MODE_MIN:
					display_number(display_min_value);
					break;
			}
			break;
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
