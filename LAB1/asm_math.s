	AREA text, CODE, READONLY
	EXPORT asm_math

; R0: pointer to the input data array
; R1: pointer to output array
; R2: input array length

; R3: current index in loop over array
; R4: current max index
; R5: current min index

; S0: current array value / square of current value (for RMS) / returned RMS value
; S1: current max value / returned max value
; S2: current min value / returned min value
; S3: running sum of squares (for RMS)
; S4: input array length (converted to floating point)

asm_math
	VLDR.f32		S1, [R0, #0]		; set initial max value to first element of array
	VLDR.f32		S2, [R0, #0]		; set initial min value to first element of array
	MOV 			R3, #0				; clear R3 (for current index in array)
	MOV 			R4, #0				; clear R4 (for current max index in array)
	MOV 			R5, #0				; clear R5 (for current min index in array)
	
start_loop
	CMP				R3, R2				; check if the end of the array is reached in the loop
	BEQ				store_result		; branch to store results			
	
	VLDR.f32		S0, [R0, #0]		; load S0 with the current array value
	
	VCMP.f32		S0, S1				; compare current array value to current max
	VMRS.f32	    APSR_nzcv, FPSCR	; get the comparison flags into APSR
	BGT				update_max			; branch to update max if current value is greater
	
	VCMP.f32		S0, S2				; compare current array value to current min
	VMRS.f32	    APSR_nzcv, FPSCR	; get the comparison flags into APSR
	BLT				update_min			; branch to update min if current value is smaller
	
	B				end_loop			; branch to the end of loop operations
	
update_max
	VMOV.f32		S1, S0				; move current value to the max value register
	MOV				R4, R3				; move current index to the max index register
	B				end_loop

update_min
	VMOV.f32		S2, S0				; move current value to the min value register
	MOV				R5, R3				; move current value to the min index register
	B				end_loop

end_loop
	VMUL.f32		S0, S0, S0			; square array value
	VADD.f32		S3, S3, S0			; add to running sum of squares
	ADD				R3, R3, #1			; increment current index of loop
	ADD				R0, R0, #4			; increase input array memory address
	B				start_loop			; branch back to the start of the loop

store_result
	VMOV.f32		S4, R2				; move array length to floating point register
	VCVT.f32.s32	S4, S4				; convert array length from integer to floating point
	
	VDIV.f32		S0, S3, S4			; divide sum of squares by length of array
	VSQRT.f32		S0, S0				; take square root to obtain RMS value in S0

	VSTR.f32		S0, [R1, #0]		; store the RMS value in the output array
	VSTR.f32		S1, [R1, #4]		; store the max value in the output array
	VSTR.f32		S2, [R1, #8]		; store the min value in the output array
	
	VMOV.f32		S0, R4				; move max index to floating point register
	VCVT.f32.s32	S0, S0				; convert max index from integer to floating point
	VSTR.f32		S0, [R1, #12]		; store the max index in the output array
	
	VMOV.f32		S0, R5				; move min index to floating point register
	VCVT.f32.s32	S0, S0				; convert min index from integer to floating point
	VSTR.f32		S0, [R1, #16]		; store the min index in the output array
	
	B 				exit				; branch to exit the subroutine

exit
	BX				LR					; branch to next instruction
	END									; end subroutine