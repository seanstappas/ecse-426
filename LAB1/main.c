#include <stdio.h>
#include "arm_math.h"

// Assembly math function
extern int asm_math(float* input_array, float* output_array, int array_length);

// C math function
int C_math(float* input_array, float* output_array, int array_length) {
		//initiliaze variables
		int cMinIndex = 0;
		int cMaxIndex = 0; float cMax = 0;
		float cMin = 0; float RMS = 0;
	//loop through input to compute max,min,indices and rms
		for(int i=0; i<array_length; i++){
			if(cMin>input_array[i] || cMin == 0){
				cMinIndex = i;
				cMin = input_array[i];
			}
			if(cMax<input_array[i] || cMax == 0){
				cMaxIndex = i;
				cMax = input_array[i];
			}
			RMS+=i*i;
		}
		RMS = RMS/array_length;
		RMS = sqrt(RMS);
		//output array in order RMS Max Min MaxIndex MinIndex
		output_array[0]=RMS; 
		output_array[1] = cMax; 
		output_array[2] = cMin;
		output_array[3] = cMaxIndex; 
		output_array[4] = cMinIndex;
		
	return 0;
}

// CMSIS math function
int CMSIS_math(float* input_array, float* output_array, int array_length) {
	//RMS
	float rms;
	arm_rms_f32(input_array, array_length, &rms);
	//MAX & INDEX
	float32_t* max;
	uint32_t maxIndex;
	arm_max_f32(input_array, array_length, max, &maxIndex);
	//MIN & INDEX
	float32_t* min;
	uint32_t minIndex;
	arm_min_f32(input_array, array_length, min, &minIndex);
	
			output_array[0] = rms;
			output_array[1] = *max;
			output_array[2] = *min;
			output_array[3] = maxIndex; 
			output_array[4] = minIndex;
	
	return 0;
}

// FIR function
//TODO: Do we need to start at i=0, where previous values are D.N.E.?
int FIR_C(int* input_array, float* output_array) {
	//coefficient of the filter
	float coeff[5] = {0.1,0.15,0.5,0.15,0.1};
	int order = 5;
	//loop through array
	for(int i = order-1;i<(sizeof(input_array)/sizeof(input_array[0]));i++){
		int sum=0;
			//compute sum
			for(int j = 0; j < order; j++){
				sum += input_array[i]*coeff[j];
			}
			output_array[i] = sum;
	}
	
	return 0;
}

int main() {
	float input_array[] = {0.1, 5.2, 3.1, 8};
	
	float output_array[5];
	int array_length = sizeof(input_array) / sizeof(float);
	
	for (int i = 0; i < sizeof(input_array) / sizeof(float); i++) {
		printf("Element %i of input: %f\n", i, input_array[i]);
	}
	
	asm_math(input_array, output_array, array_length);
	
	printf("RMS value: %f\n", output_array[0]);
	printf("Max value: %f\n", output_array[1]);
	printf("Min value: %f\n", output_array[2]);
	printf("Max index: %f\n", output_array[3]);
	printf("Min index: %f\n", output_array[4]);
	
	return 0;
}
