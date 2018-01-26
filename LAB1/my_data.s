	AREA myData, DATA, READWRITE
	export myString1
	export myString2
myString1 DCB 1, 2, 3, 4, 5
myString2 FILL 50, 0xAB
	END