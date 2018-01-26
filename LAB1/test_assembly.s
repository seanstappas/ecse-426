	AREA text, CODE, READONLY
	EXPORT example1
	import myString1
	import myString2
example1
	LDR R0, =myString3
	LDR R1, [R0]
	LDR R2, =myString1
	LDR R1, [R2]
	BX LR
	
	ALIGN
myString3 DCB 1, 2, 3, 4
	
	END