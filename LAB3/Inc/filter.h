/**
	******************************************************************************
  * @file    filter.h
  * @author  Sean Stappas and Eric Vuong
  * @version V1.0.0
  * @date    09-March-2018
  * @brief   Header file for the filter.
  ******************************************************************************
  */

#ifndef __FILTER_H__
#define __FILTER_H__

extern volatile float rms_value;
extern volatile float max_value;
extern volatile float min_value;

void update_rms_and_running_max_min(void);
void update_raw_and_filtered_data(uint32_t datum);
void update_max_and_min(void);

#endif
