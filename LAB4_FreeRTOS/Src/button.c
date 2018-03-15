/**
  ******************************************************************************
  * @file    button.c
  * @author  Sean Stappas and Eric Vuong
  * @version V1.0.0
  * @date    09-March-2018
  * @brief   Controls the button debouncing and reading.
  ******************************************************************************
  */
	
#include "stm32f4xx_hal.h"
#include "button.h"

GPIO_PinState last_button_state = GPIO_PIN_RESET;
GPIO_PinState button_state = GPIO_PIN_RESET;
int button_ticks = 0;

/**
  * @brief  Reads the B1 button with debouncing.
  * @retval None
  */
void read_button_debounce(void)
{
	// Read the B1 button (PA0) with debouncing
	// Debounce inspired from https://www.arduino.cc/en/Tutorial/Debounce
	if (current_phase != SLEEP_PHASE)
	{
		GPIO_PinState reading = HAL_GPIO_ReadPin(Button_GPIO_Port, Button_Pin);
		if(reading != last_button_state)
		{
			button_ticks = 0;
		}
		if(button_ticks > BUTTON_DEBOUNCE_DELAY)
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
		button_ticks++;
	}
}
