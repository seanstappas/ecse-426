	AREA text, CODE, READONLY
	EXPORT asm_math

; R0: pointer to the input data array / returned max index
; R1: pointer to output array / returned min index
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
	
start_loop
	CMP			R3, R2			; check if the end of the array is reached in the loop
	BEQ			store_result	; branch to store results			
	
	VLDR.f32	S0, [R0, #0]	; load S0 with the current array value
	
	VCMP.f32	S0, S1			; compare current array value to current max
	BGT			update_max		; branch to update max if current value is greater
	
	VCMP.f32	S0, S2			; compare current array value to current min
	BLT			update_min		; branch to update min if current value is smaller
	
	B			end_loop		; branch to the end of loop operations
	
update_max
	VMOV.f32	S1, S0			; move current value to the max value register
	MOV			R4, R3			; move current index to the max index register
	B			end_loop

update_min
	VMOV.f32	S2, S0			; move current value to the min value register
	MOV			R5, R3			; move current value to the min index register
	B			end_loop

end_loop
	VMUL.f32	S0, S0, S0		; square array value
	VADD.f32	S3, S3, S0		; add to running sum of squares
	ADD			R3, R3, #4		; increment current index of loop
	ADD			R0, R0, #4		; increase input array memory address
	B			start_loop		; branch back to the start of the loop

store_result
	MOV			R0, R4			; move max index to R0
	MOV			R1, R5			; move min index to R1

	VMOV.f32	S4, R2			; convert input array length to floating point
	VDIV.f32	S0, S3, S4		; divide sum of squares by length of array (todo; convert R2 to floating point?)
	VSQRT.f32	S0, S0			; take square root to obtain RMS value in S0
	
	B 			exit			; branch to exit the subroutine

exit
	BX			LR				; branch to next instruction
	END								; end subroutine