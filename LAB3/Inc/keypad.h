#ifndef __KEYPAD_H__
#define __KEYPAD_H__

extern volatile float desired_output_voltage;
extern volatile int current_keypad_phase;
extern volatile int voltage_digits[2];
	
void handle_keypad_pressed_key(char pressed_key);
char read_keypad_char(void);
void read_keypad_debounce(void);

#endif