#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include "filter.h"

float filtered_data[10];
float running_min = 5;
float running_max = 0;
float raw_data[10];
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

float fir_filter(void);

/**
  * @brief  Updates the RMS, running MAX and running MIN based on the filtered data array.
  * @retval None
  */
void update_rms_and_running_max_min(void){
	// RMS
	float32_t rms;
	arm_rms_f32(filtered_data, 10, &rms);
	rms_value = rms;
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
  * @param  datum: The incoming reading to filter.
  * @retval None
  */
void update_raw_and_filtered_data(uint32_t datum)
{	
	// Shift raw data array up
	for (int i = 1; i < 10; i++)
	{
		raw_data[i] = raw_data[i - 1];
	}
	
	// Sample raw ADC data
	raw_data[0] = (datum / 1023.0) * V_REF;
	
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
