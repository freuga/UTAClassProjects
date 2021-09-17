; Defining the functions
	.def getSVC
	.def getPSP
	.def setPSP
	.def pushPC
	.def setControl
	.def getControl
	.def getMSP
	.def pushR4toR11toPSP
	.def popR4toR11fromPSP
	.def getR0
	.def getR1

; options for the thumb instruction compilation
.thumb
.const

.text
; returns Process Stack Pointer
getPSP:
		MRS		r0, PSP
		BX		LR

; Sets the ASP to 1
setControl:
		MSR		CONTROL, r0
		BX		LR

; Get control bits
getControl:
		MRS		r0, CONTROL
		BX 		LR

; Sets the Process Stack Pointer
setPSP:
		MSR		PSP, r0
		BX		LR

; returns Main Stack Pointer
getMSP:
		MRS		r0, MSP
		BX		LR

;Pushes R4-R11 to PSP
pushR4toR11toPSP:
		MRS		r0, PSP
		SUB		r0, r0, #4
		STR		r4, [r0]
		SUB		r0, r0, #4
		STR		r5, [r0]
		SUB		r0, r0, #4
		STR		r6, [r0]
		SUB		r0, r0, #4
		STR		r7, [r0]
		SUB		r0, r0, #4
		STR		r8, [r0]
		SUB		r0, r0, #4
		STR		r9, [r0]
		SUB		r0, r0, #4
		STR		r10, [r0]
		SUB		r0, r0, #4
		STR		r11, [r0]
		MSR		PSP, r0
		BX		LR

; Pops R4-R11 from PSP
popR4toR11fromPSP:
		MRS		r0, PSP
		LDR		r4, [r0]
		ADD		r0, r0, #4
		LDR		r5, [r0]
		ADD		r0, r0, #4
		LDR		r6, [r0]
		ADD		r0, r0, #4
		LDR		r7, [r0]
		ADD		r0, r0, #4
		LDR		r8, [r0]
		ADD		r0, r0, #4
		LDR		r9, [r0]
		ADD		r0, r0, #4
		LDR		r10, [r0]
		ADD		r0, r0, #4
		LDR		r11, [r0]
		ADD		r0, r0, #4
		MSR		PSP, r0
		BX		LR
pushPC:
		MRS		r4, PSP
		SUB		r4, r4, #4
		STR		r0, [r4]
		MSR 	PSP, r4
		BX		LR
getSVC:
		MRS		r4, PSP
		ADD		r4, r4, #24
		LDR		r0, [r4]
		SUB 	r0, r0, #2
		LDR		r0, [r0]
		BX		LR
getR0:
		MRS		r0, PSP
		LDR		r0, [r0]
		BX		LR
getR1:
		MRS		r4, PSP
		ADD		r4, r4, #4
		LDR		r0, [r4]
		BX		LR
.endm
