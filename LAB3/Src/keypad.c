#include "stm32f4xx_hal.h"
#include "keypad.h"

static const char keypad [4][3] = {
	{'1', '2', '3'},
	{'4', '5', '6'},
	{'7', '8', '9'},
	{'*', '0', '#'}
};

char last_pressed_key = 0;
int keypad_counter = 0;
int keypad_debounce_ticks = 0;
char current_keypad_char = 0;
char last_pressed_key_debounce = 0;
int keypad_debounce_delay = 10;
int current_input_digit = 0;

void disable_display(void);
float convert_user_input_to_desired_range(int first_digit, int second_digit);

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
		else if (pressed_key == '#' && pressed_key == last_pressed_key)
		{
			keypad_counter++;
			// Press for 3 s
			if (keypad_counter >= 600)
			{
				// Enter INPUT phase
				current_keypad_phase = INPUT_PHASE;
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
				if (keypad_counter <= 600 && last_pressed_key == '#')
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
