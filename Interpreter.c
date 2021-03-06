// Interpreter.c
// Runs on LM4F120/TM4C123
// Tests the UART0 to implement bidirectional data transfer to and from a
// computer running HyperTerminal.  This time, interrupts and FIFOs
// are used.
// Daniel Valvano
// September 12, 2013
// Modified by Kenneth Lee, Dalton Altstaetter 2/9/2015

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2014
   Program 5.11 Section 5.6, Program 3.10

 Copyright 2014 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

// U0Rx (VCP receive) connected to PA0
// U0Tx (VCP transmit) connected to PA1
#include <stdio.h>
#include <stdint.h>
#include "PLL.h"
#include "UART.h"
#include "ST7735.h"
#include "ADC.h"
#include <rt_misc.h>
#include <string.h>
#include "OS.h"
#include "ifdef.h"

void Interpreter(void);
//---------------------OutCRLF---------------------
// Output a CR,LF to UART to go to a new line
// Input: none
// Output: none
void OutCRLF(void){
  UART_OutChar(CR);
  UART_OutChar(LF);
}

#define PE4  (*((volatile unsigned long *)0x40024040))
#ifdef INTERPRETER
uint16_t TestBuffer[64];
extern uint32_t FIR_Status;
extern uint32_t ADCTrigMode;
extern void Producer(unsigned long data);
extern void Consumer(void);
void Interpreter(void){
	char input_str[30];
	int input_num,i,device,line;
	int freq, numSamples;
	UART_Init();              // initialize UART
	OutCRLF();
	OutCRLF();
	
	//Print Interpreter Menu
	printf("Debugging Interpreter Lab 1\n\r");
	printf("Commands:\n\r");
	printf("LCD\n\r");
	printf("OS-K - Kill the Interpreter\n\r");
	#ifdef PROFILER
	printf("PROFILE - get profiling info for past events\n\r");
	#endif
	printf("ADC - start timer-triggered ADC\n\r");
	printf("FIR - toggle the FIR filter\n\r");
	printf("ADC-S - toggle the ADC\n\r");
	printf("ADC-T - set ADC trigger mode\n\r");
	printf("FFT - toggle FFT display\n\r");
	
	while(1){
		//PE4^=0x10;
		printf("\n\rEnter a command:\n\r");
		for(i=0;input_str[i]!=0;i++){input_str[i]=0;}		//Flush the input_str
		UART_InString(input_str,30);
		if(!strcmp(input_str,"LCD")){	
			printf("\n\rMessage to Print: ");
			for(i=0;input_str[i]!=0;i++){input_str[i]=0;}		//Flush the input_str
			UART_InString(input_str,30);
			printf("\n\rNumber to Print: ");
			input_num=UART_InUDec();
			printf("\n\rDevice to Print to: ");
			device = UART_InUDec();
			printf("\n\rLine to Print to: ");
			line = UART_InUDec();
			ST7735_Message(device,line,input_str,input_num);
		} else if(!strcmp(input_str,"OS-K")){
			OS_Kill();
			#ifdef PROFILER
		} else if(!strcmp(input_str,"PROFILE")){
				printf("\n\rThreadAddress\tThreadAction\tThreadTime\n\r");
				for(i=0; i<PROFSIZE; i++){
					printf("%lu\t\t%lu\t\t%lu\n\r",(unsigned long)ThreadArray[i],ThreadAction[i],ThreadTime[i]/80000);
				}
			#endif 
		} else if(!strcmp(input_str,"ADC")){
			  ADC_Collect(4, 12800, &Producer); // start ADC sampling, channel 4, PD3, 12800 Hz 
				OS_AddThread(&Consumer,128,1);
		} else if(!strcmp(input_str,"FIR")){
				FIR_Status^=0x01;
		} else if(!strcmp(input_str,"ADC-S")){
				ADC0_IM_R ^= 0x08;  //toggles the arm bit for sequencer 3 ADC
		} else if(!strcmp(input_str,"ADC-T")){
				printf("\n\rSelect Trigger Mode\n\r");
				printf("0 = Frame on Button Press\n\r");
				printf("1 = Continuous Sampling\n\r");
				input_num = UART_InUDec();
				switch(input_num){
					case 0:
						ADCTrigMode = 0; //button press
						break;
					case 1:
						ADCTrigMode = 1;  //continuous sampling
						break;
					default:
						printf("Invalid Input\n\r");
						break;
				}
		}
			else{
			printf("\n\rInvalid Command. Try Again\n\r");
		}
	}
}
#endif

