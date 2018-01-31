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



// FIR function
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

int test_math_functions(float* input_array, int array_length) {
	float output_array_cmsis[5];
	CMSIS_math(input_array, output_array_cmsis, array_length);
	
	float output_array_c[5];
	C_math(input_array, output_array_c, array_length);
	
	float output_array_asm[5];
	asm_math(input_array, output_array_asm, array_length);
	
	printf("RMS value (CMSIS): %f\n", output_array_cmsis[0]);
	printf("RMS value (C)    : %f\n", output_array_c[0]);
	printf("RMS value (ASM)  : %f\n", output_array_asm[0]);
	printf("\n");
	
	printf("Max value (CMSIS): %f\n", output_array_cmsis[1]);
	printf("Max value (C)    : %f\n", output_array_c[1]);
	printf("Max value (ASM)  : %f\n", output_array_asm[1]);
	printf("\n");
	
	printf("Min value (CMSIS): %f\n", output_array_cmsis[2]);
	printf("Min value (C)    : %f\n", output_array_c[2]);
	printf("Min value (ASM)  : %f\n", output_array_asm[2]);
	printf("\n");
	
	printf("Max index (CMSIS): %f\n", output_array_cmsis[3]);
	printf("Max index (C)    : %f\n", output_array_c[3]);
	printf("Max index (ASM)  : %f\n", output_array_asm[3]);
	printf("\n");
	
	printf("Min index (CMSIS): %f\n", output_array_cmsis[4]);
	printf("Min index (C)    : %f\n", output_array_c[4]);
	printf("Min index (ASM)  : %f\n", output_array_asm[4]);
	printf("\n");
	
	return 0;
}

int test_filter() {
	int input_array2[] = {-3,-4,3};
	int array_length2 = sizeof(input_array2)/sizeof(int);
	float output_array2[array_length2];
	FIR_C(input_array2, output_array2, array_length2);
	
	for (int i = 0; i < array_length2; i++) {
		printf("index %i: %f\n", i, output_array2[i]);
	}
	
	return 0;
}

int main() {
	//float input_array1[] = {0.1, 5.2, 3.1, 8.0};
	//int array_length = sizeof(input_array1) / sizeof(float);
	//test_math_functions(input_array1, array_length);
	
	//float input_array2[] = {-51040, 5002.32, 3001.21, 8414, 40402.3};
	//int array_length = sizeof(input_array2) / sizeof(float);
	//test_math_functions(input_array2, array_length);
	
	//float input_array3[] = {0.3311, 14124.322, 300.21, 800.323};
	//int array_length = sizeof(input_array3) / sizeof(float);
	//test_math_functions(input_array3, array_length);
	
	//test_filter();
	
	//float input_array4[] = {0.131231, 0.51232, 0.1231321, 0.1231238};
	//int array_length = sizeof(input_array4) / sizeof(float);
	//test_math_functions(input_array4, array_length);
	
	int input_array[] = {-3,-4,3,4,20,10};
	int array_length = sizeof(input_array) / sizeof(float);
	
	printf("Input vector:\n");
	for (int i = 0; i < array_length; i++) {
		printf("%i\n", i, input_array[i]);
	}
	printf("\n");
	
	float output_array[array_length];
	FIR_C(input_array, output_array, array_length);
	
	printf("Filtered vector:\n");
	for (int i = 0; i < array_length; i++) {
		printf("%f\n", i, output_array[i]);
	}
	printf("\n");
	
	test_math_functions(output_array, array_length);
	
	return 0;
}
