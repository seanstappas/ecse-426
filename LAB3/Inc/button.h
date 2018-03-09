#ifndef __BUTTON_H__
#define __BUTTON_H__

#define BUTTON_DEBOUNCE_DELAY 10

extern volatile int current_display_mode;

void read_button_debounce(void);

#endif
