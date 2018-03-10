/**
  ******************************************************************************
  * @file    button.h
  * @author  Sean Stappas and Eric Vuong
  * @version V1.0.0
  * @date    09-March-2018
  * @brief   Header file for the button control.
  ******************************************************************************
  */
	
#ifndef __BUTTON_H__
#define __BUTTON_H__

#define BUTTON_DEBOUNCE_DELAY 10

extern volatile int current_display_mode;
extern volatile int current_phase;

void read_button_debounce(void);

#endif
