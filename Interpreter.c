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
//#define INTERPRETER

void Interpreter(void);




//---------------------OutCRLF---------------------
// Output a CR,LF to UART to go to a new line
// Input: none
// Output: none
void OutCRLF(void){
  UART_OutChar(CR);
  UART_OutChar(LF);
}
//debug code
int main1(void){
  char i;
  char string[20];  // global to assist in debugging
  uint32_t n;

  PLL_Init();               // set system clock to 50 MHz
  UART_Init();              // initialize UART
  OutCRLF();
  for(i='A'; i<='Z'; i=i+1){// print the uppercase alphabet
    UART_OutChar(i);
  }
  OutCRLF();
  UART_OutChar(' ');
  for(i='a'; i<='z'; i=i+1){// print the lowercase alphabet
    UART_OutChar(i);
  }
  OutCRLF();
  UART_OutChar('-');
  UART_OutChar('-');
  UART_OutChar('>');
  while(1){
    UART_OutString("InString: ");
    UART_InString(string,19);
    UART_OutString(" OutString="); UART_OutString(string); OutCRLF();

    UART_OutString("InUDec: ");  n=UART_InUDec();
    UART_OutString(" OutUDec="); UART_OutUDec(n); OutCRLF();

    UART_OutString("InUHex: ");  n=UART_InUHex();
    UART_OutString(" OutUHex="); UART_OutUHex(n); OutCRLF();

  }
}

int main2(void){
	PLL_Init();
	Output_Init();
	ST7735_Message(0,0,"Hello World",45);
	ST7735_Message(1,8,"Hello World",45);
	ST7735_Message(0,0,"A",12345);
	while(1){;}
}
#define PE4  (*((volatile unsigned long *)0x40024040))
#ifdef INTERPRETER
uint16_t TestBuffer[64];
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
		}
		else{
			printf("\n\rInvalid Command. Try Again\n\r");
		}
		//OS_Sleep(1000);
	}
}
#endif

