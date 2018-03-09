#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#define DISPLAY_MODE_RMS 0
#define DISPLAY_MODE_MAX 1
#define DISPLAY_MODE_MIN 2

extern volatile int current_display_mode;
extern volatile int current_phase;
extern volatile int voltage_digits[2];
extern volatile float display_rms_value;
extern volatile float display_max_value;
extern volatile float display_min_value;

void display_current_number(void);
void disable_display(void);

#endif
