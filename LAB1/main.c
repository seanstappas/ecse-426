#include <stdio.h>
#include "arm_math.h"

// Assembly math function
extern int asm_math(float* input_array, float* output_array, int array_length);

// C math function
int C_math(float* input_array, float* output_array, int array_length) {
	return 0;
}

// CMSIS math function
int CMSIS_math(float* input_array, float* output_array, int array_length) {
	return 0;
}

// FIR function
int FIR_C(int* input_array, float* output_array) {
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
	
	for (int i = 0; i < 5; i++) {
		printf("Element %i of output: %f\n", i, output_array[i]);
	}
	
	return 0;
}
