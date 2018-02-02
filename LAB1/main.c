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
			RMS+=input_array[i]*input_array[i];
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
	float32_t max;
	uint32_t maxIndex;
	arm_max_f32(input_array, array_length, &max, &maxIndex);
	//MIN & INDEX
	float32_t min;
	uint32_t minIndex;
	arm_min_f32(input_array, array_length, &min, &minIndex);
	
			output_array[0] = rms;
			output_array[1] = max;
			output_array[2] = min;
			output_array[3] = maxIndex; 
			output_array[4] = minIndex;
	
	return 0;
}

float FIR_C2(int input, float output){
	float coeff[5] = {0.2,0.2,0.2,0.2,0.2};
	int order = 5;
	int* ptr = &input;
	ptr -= 4;
	for(int i=0;i<order;i++){
		output += *ptr * coeff[i];
	}
	return output;
}

// C FIR function
int FIR_C(int* input_array, float* output_array, int array_length) {
	//coefficient of the filter
	float coeff[5] = {0.1,0.15,0.5,0.15,0.1};
	int order = 5;
	//loop through array
	for(int i = 0;i<array_length;i++){
		float sum = 0.0;
			//compute sum
			for(int j = order-1; j >=0; j--){
					int sumIndex = i-j;
					if(sumIndex>=0){
						sum += input_array[sumIndex]*coeff[j];
					}
			}
			output_array[i] = sum;
	}
	
	return 0;
}

// Tests the C FIR filter
int test_filter(int* input_array, float* output_array, int array_length) {
	printf("Input vector:\n");
	for (int i = 0; i < array_length; i++) {
		printf("%i\n", input_array[i]);
	}
	printf("\n");
	
	FIR_C(input_array, output_array, array_length);
	
	printf("Filtered vector:\n");
	for (int i = 0; i < array_length; i++) {
		printf("%f\n", output_array[i]);
	}
	printf("\n");
	
	return 0;
}

// Tests all the math functions (assembly, CMSIS and C)
int test_math_functions(float* input_array, int array_length) {
	float output_array_cmsis[5];
	float output_array_c[5];
	float output_array_asm[5];
	
	asm_math(input_array, output_array_asm, array_length);
	CMSIS_math(input_array, output_array_cmsis, array_length);
	C_math(input_array, output_array_c, array_length);
	
	printf("RMS value (ASM)  : %f\n", output_array_asm[0]);
	printf("RMS value (CMSIS): %f\n", output_array_cmsis[0]);
	printf("RMS value (C)    : %f\n", output_array_c[0]);
	printf("\n");
	
	printf("Max value (ASM)  : %f\n", output_array_asm[1]);
	printf("Max value (CMSIS): %f\n", output_array_cmsis[1]);
	printf("Max value (C)    : %f\n", output_array_c[1]);
	printf("\n");
	
	printf("Min value (ASM)  : %f\n", output_array_asm[2]);
	printf("Min value (CMSIS): %f\n", output_array_cmsis[2]);
	printf("Min value (C)    : %f\n", output_array_c[2]);
	printf("\n");
	
	printf("Max index (ASM)  : %f\n", output_array_asm[3]);
	printf("Max index (CMSIS): %f\n", output_array_cmsis[3]);
	printf("Max index (C)    : %f\n", output_array_c[3]);
	printf("\n");
	
	printf("Min index (ASM)  : %f\n", output_array_asm[4]);
	printf("Min index (CMSIS): %f\n", output_array_cmsis[4]);
	printf("Min index (C)    : %f\n", output_array_c[4]);
	printf("\n");
	
	return 0;
}

// Tests a CMSIS FIR filter for reference.
int test_reference_filter() {
	float input_array[] = {-3,-4,3,4,20,10};
	float coeff[5] = {0.1,0.15,0.5,0.15,0.1};
	int array_length = sizeof(input_array) / sizeof(float);
	float firStateF32[array_length + 5 - 1];
	
  arm_fir_instance_f32 S;
  float outputF32[array_length];
  /* Initialize input and output buffer pointers */
  arm_fir_init_f32(&S, 5, coeff, firStateF32, array_length);
  arm_fir_f32(&S, input_array, outputF32, array_length);
	
	printf("Reference vector:\n");
	for (int i = 0; i < array_length; i++) {
		printf("%f\n", outputF32[i]);
	}
	printf("\n");
	
	return 0;
}

// Tests the filter in combination with the math functions
int test_integration() {
	int input_array[] = {-3,-4,3,4,20,10};
	int array_length = sizeof(input_array) / sizeof(float);
	float output_array[array_length];
	
	test_filter(input_array, output_array, array_length);
	test_math_functions(output_array, array_length);
	
	return 0;
}

int main() {
	
	test_integration();
	return 0;
}
