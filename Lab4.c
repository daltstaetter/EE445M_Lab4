// Lab4.c
// Runs on LM4F120/TM4C123
// Real Time Operating System for Labs 2 and 3
// Lab2 Part 1: Testmain1 and Testmain2
// Lab2 Part 2: Testmain3 Testmain4  and main
// Lab3: Testmain5 Testmain6, Testmain7, and main (with SW2)

// Jonathan W. Valvano 1/31/14, valvano@mail.utexas.edu
// EE445M/EE380L.6 
// You may use, edit, run or distribute this file 
// You are free to change the syntax/organization of this file

// LED outputs to logic analyzer for OS profile 
// PF1 is preemptive thread switch
// PF2 is periodic task, samples PD3
// PF3 is SW1 task (touch PF4 button)

// Button inputs
// PF0 is SW2 task (Lab3)
// PF4 is SW1 button input

// Analog inputs
// PD3 Ain3 sampled at 2k, sequencer 3, by DAS software start in ISR
// PD2 Ain5 sampled at 250Hz, sequencer 0, by Producer, timer tigger

#include "OS.h"
#include "tm4c123gh6pm.h"
#include "ST7735.h"
#include "ADC.h"
#include "UART.h"
#include <string.h> 
#include "ifdef.h"
#include "LinkedList.h"

extern void Interpreter(void);
int32_t StartCritical(void);
void EndCritical(int32_t primask);

#ifdef TASKS
//*********Prototype for FFT in cr4_fft_64_stm32.s, STMicroelectronics
void cr4_fft_64_stm32(void *pssOUT, void *pssIN, unsigned short Nbin);
//*********Prototype for PID in PID_stm32.s, STMicroelectronics
short PID_stm32(short Error, short *Coeff);

unsigned long NumCreated;   // number of foreground threads created
unsigned long PIDWork;      // current number of PID calculations finished
unsigned long FilterWork;   // number of digital filter calculations finished
unsigned long NumSamples;   // incremented every ADC sample, in Producer
#define FS 12800            // producer/consumer sampling
#define RUNLENGTH (20*FS) // display results and quit when NumSamples==RUNLENGTH
// 20-sec finite time experiment duration 

#define PERIOD TIME_500US // DAS 2kHz sampling period in system time units
//#define PERIOD 800000   //100 Hz
//#define PERIOD 160000   // 500 Hz
//#define PERIOD 80000    //1000 Hz


//---------------------User debugging-----------------------
unsigned long DataLost;     // data sent by Producer, but not received by Consumer
long MaxJitter;             // largest time jitter between interrupts in usec
#define JITTERSIZE 64
unsigned long const JitterSize=JITTERSIZE;
unsigned long JitterHistogram[JITTERSIZE]={0,};
#define PE0  (*((volatile unsigned long *)0x40024004))
#define PE1  (*((volatile unsigned long *)0x40024008))
#define PE2  (*((volatile unsigned long *)0x40024010))
#define PE3  (*((volatile unsigned long *)0x40024020))
#define PE4  (*((volatile unsigned long *)0x40024040))
#define PE5  (*((volatile unsigned long *)0x40024080))
#define PE6  (*((volatile unsigned long *)0x40024100))
	

void PortE_Init(void){ 
	
	unsigned long volatile delay;
	SYSCTL_RCGCGPIO_R  |= SYSCTL_RCGC2_GPIOE;       // activate port E
  delay = SYSCTL_RCGC2_R;        
  delay = SYSCTL_RCGC2_R;         
  GPIO_PORTE_DIR_R |= 0x7F;    // make PE6-0 output heartbeats
  GPIO_PORTE_AFSEL_R &= ~0x7F;   // disable alt funct on PE6-0
  GPIO_PORTE_DEN_R |= 0x7F;     // enable digital I/O on PE6-0
  GPIO_PORTE_PCTL_R = ~0x0000FFFF;
  GPIO_PORTE_AMSEL_R &= ~0x7F;;      // disable analog functionality on PE
}
//------------------Task 1--------------------------------
//51-tap FIR filter
const long h[51]={4,-1,-8,-14,-16,-10,-1,6,5,-3,-13,
     -15,-8,3,5,-5,-20,-25,-8,25,46,26,-49,-159,-257,
     984,-257,-159,-49,26,46,25,-8,-25,-20,-5,5,3,-8,
     -15,-13,-3,5,6,-1,-10,-16,-14,-8,-1,4};
long Filter(long data){
static long x[102]; // this MACQ needs twice
long y=0;
static unsigned long n=3;   // 3, 4, or 5
int i;
  n++;
  if(n==102) n=51;     
  x[n] = x[n-51] = data;  // two copies of new data
  for(i=n-51;i<n;i++){
		y=y+h[i]*x[n-i];
	}
  return y;
} 
//******** DAS *************** 
// background thread, calculates 60Hz notch filter
// runs 2000 times/sec
// samples channel 4, PD3,
// inputs:  none
// outputs: none
unsigned long DASoutput;
void DAS(void)
{ 
	unsigned long input;  
	unsigned static long LastTime;  // time at previous ADC sample
	unsigned long thisTime;         // time at current ADC sample
	long jitter;                    // time between measured and expected, in us
  
	if(NumSamples < RUNLENGTH)
	{   // finite time run
		int i;
    PE0 ^= 0x01;
    input = ADC_In();           // channel set when calling ADC_Init
    PE0 ^= 0x01;
    thisTime = OS_Time();       // current time, 12.5 ns
		//for(i=0;i<=500;i++){}
    DASoutput = Filter(input);
    FilterWork++;        // calculation finished
    if(FilterWork > 1)
		{    // ignore timing of first interrupt
      unsigned long diff = OS_TimeDifference(LastTime,thisTime);
			jitter = (diff > PERIOD) ? (diff-PERIOD+4)/8:(PERIOD-diff+4)/8; // in 0.1 usec
			
      if(jitter > MaxJitter)
			{
        MaxJitter = jitter; // in usec
      }       // jitter should be 0
      if(jitter >= JitterSize)
			{
        jitter = JITTERSIZE-1;
      }
      JitterHistogram[jitter]++; 
    }
    LastTime = thisTime;
    PE0 ^= 0x01;
  }
}
//--------------end of Task 1-----------------------------

//------------------Task 2--------------------------------
// background thread executes with SW1 button
// one foreground task created with button push
// foreground threads run for 2 sec and die
// ***********ButtonWork*************
extern int g_NumAliveThreads;
void ButtonWork(void)
{
	long status; 
	unsigned long myId = OS_Id(); 
  PE1 ^= 0x02;
  ST7735_Message(1,8,"NumCreated =",g_NumAliveThreads); 
  PE1 ^= 0x02;
  OS_Sleep(50);     // set this to sleep for 50msec
  ST7735_Message(1,9,"PIDWork     =",PIDWork);
  ST7735_Message(1,10,"DataLost    =",DataLost);
  ST7735_Message(1,11,"Jitter 0.1us=",MaxJitter);
  PE1 ^= 0x02;
  OS_Kill();  // done, OS does not return from a Kill
} 


//************SW1Push*************
// Called when SW1 Button pushed
// Adds another foreground task
// background threads execute once and return
void SW1Push(void)
{
   NumCreated += OS_AddThread(&ButtonWork,128,SWITCHPRI);
}
//************SW2Push*************
// Called when SW2 Button pushed, Lab 3 only
// Adds another foreground task
// background threads execute once and return
void SW2Push(void){
  NumCreated += OS_AddThread(&ButtonWork,128,SWITCHPRI);
}
//--------------end of Task 2-----------------------------

//------------------Task 3--------------------------------
// hardware timer-triggered ADC sampling at 400Hz
// Producer runs as part of ADC ISR
// Producer uses fifo to transmit 400 samples/sec to Consumer
// every 64 samples, Consumer calculates FFT
// every 2.5ms*64 = 160 ms (6.25 Hz), consumer sends data to Display via mailbox
// Display thread updates LCD with measurement

//******** Producer *************** 
// The Producer in this lab will be called from your ADC ISR
// A timer runs at 400Hz, started by your po
// The timer triggers the ADC, creating the 12800 Hz sampling
// Your ADC ISR runs when ADC data is ready
// Your ADC ISR calls this function with a 12-bit sample 
// sends data to the consumer, runs periodically at 400Hz
// inputs:  none
// outputs: none
uint32_t FIR_Status = 1;
void Producer(unsigned long data){  
	unsigned long temp=data;
    NumSamples++; 		// number of samples
		if(FIR_Status==1){
			temp = Filter(data);
		}
    if(OS_Fifo_Put(temp) == 0){ // send to consumer
      DataLost++;
    } 
}
void Display(void); 

//******** Consumer *************** 
// foreground thread, accepts data from producer
// calculates FFT, sends DC component to Display
// inputs:  none
// outputs: none
long x[64],y[64];         // input and output arrays for FFT
uint32_t Coordinates[2][128];
void Consumer(void)
{ 
	unsigned long data,DCcomponent;   // 12-bit raw ADC sample, 0 to 4095
	unsigned long t;                  // time in 2.5 ms
	unsigned long myId = OS_Id(); 
	static uint32_t LCDx=0,LCDy;
  ADC_Collect(4, FS, &Producer); // start ADC sampling, channel 4, PD3, 12800 Hz              
  //NumCreated += OS_AddThread(&Display,128,1); 
	PE2 = 0x04;
	while(1){
		for(t = 0; t < 64; t++)
		{   // collect 64 ADC samples
			data = OS_Fifo_Get();    // get from producer
			
			//print voltage vs. time
			LCDy = 100;	//map ADC value to pixel value
			ST7735_DrawPixel(Coordinates[0][LCDx], Coordinates[1][LCDx],0x0000);     //Clear the previous pixel value
			ST7735_DrawPixel(LCDx,LCDy,0xFFFF);				//Draw new pixel value
			Coordinates[0][LCDx]=LCDx;					//Store current setpoint and speed in array to be overwritten on next pass 
			Coordinates[1][LCDx]=LCDy;
			LCDx++;
			if(LCDx==128)LCDx=0;								//If the plot reaches the end of the screen, reset x coordinate
			
			x[t] = data;             // real part is 0 to 4095, imaginary part is 0
		}
		PE2 = 0x00;
		cr4_fft_64_stm32(y,x,64);  // complex FFT of last 64 ADC values
	}
//	DCcomponent = y[0]&0xFFFF; // Real part at frequency 0, imaginary part should be zero
//	OS_MailBox_Send(DCcomponent); // called every 2.5ms*64 = 160ms

//  OS_Kill();  // done
}
//******** Display *************** 
// foreground thread, accepts data from consumer
// displays calculated results on the LCD
// inputs:  none                            
// outputs: none
void Display(void){ 
unsigned long data,voltage;
  ST7735_Message(0,0,"Run length = ",(RUNLENGTH)/FS);   // top half used for Display
  while(NumSamples < RUNLENGTH) { 
    data = OS_MailBox_Recv();
    voltage = 3000*data/4095;               // calibrate your device so voltage is in mV
    PE3 = 0x08;
    ST7735_Message(0,1,"v(mV) =",voltage);  
    PE3 = 0x00;
  } 
  OS_Kill();  // done
} 

//--------------end of Task 3-----------------------------

//------------------Task 4--------------------------------
// foreground thread that runs without waiting or sleeping
// it executes a digital controller 
//******** PID *************** 
// foreground thread, runs a PID controller
// never blocks, never sleeps, never dies
// inputs:  none
// outputs: none
short IntTerm;     // accumulated error, RPM-sec
short PrevError;   // previous error, RPM
short Coeff[3];    // PID coefficients
short Actuator;
void PID(void){ 
short err;  // speed error, range -100 to 100 RPM
unsigned long myId = OS_Id(); 
  PIDWork = 0;
  IntTerm = 0;
  PrevError = 0;
  Coeff[0] = 384;   // 1.5 = 384/256 proportional coefficient
  Coeff[1] = 128;   // 0.5 = 128/256 integral coefficient
  Coeff[2] = 64;    // 0.25 = 64/256 derivative coefficient*
  while(NumSamples < RUNLENGTH) { 
		PE4^=0x10;
    for(err = -1000; err <= 1000; err++){    // made-up data
      Actuator = PID_stm32(err,Coeff)/256;
    }
    PIDWork++;        // calculation finished
  }
  for(;;){ 
		PE4^=0x10;
	}          // done
}
//--------------end of Task 4-----------------------------

//------------------Task 5--------------------------------
// UART background ISR performs serial input/output
// Two software fifos are used to pass I/O data to foreground
// The interpreter runs as a foreground thread
// The UART driver should call OS_Wait(&RxDataAvailable) when foreground tries to receive
// The UART ISR should call OS_Signal(&RxDataAvailable) when it receives data from Rx
// Similarly, the transmit channel waits on a semaphore in the foreground
// and the UART ISR signals this semaphore (TxRoomLeft) when getting data from fifo
// Modify your intepreter from Lab 1, adding commands to help debug 
// Interpreter is a foreground thread, accepts input from serial port, outputs to serial port
// inputs:  none
// outputs: none
//void Interpreter(void);    // just a prototype, link to your interpreter
// add the following commands, leave other commands, if they make sense
// 1) print performance measures 
//    time-jitter, number of data points lost, number of calculations performed
//    i.e., NumSamples, NumCreated, MaxJitter, DataLost, FilterWork, PIDwork
      
// 2) print debugging parameters 
//    i.e., x[], y[] 
//--------------end of Task 5-----------------------------
#endif

void doNothing0(void)
{
	;
}

#ifdef FINAL
//Spectrum Analyzer Main
int main(void){ 
  OS_Init();           // initialize, disable interrupts
  PortE_Init();
	OS_InitSemaphore(&LCDmutex,1);
	Output_Init();
	UART_Init();
  DataLost = 0;        // lost data between producer and consumer
  NumSamples = 0;
  MaxJitter = 0;       // in 1us units
//********initialize communication channel
  OS_MailBox_Init();
  OS_Fifo_Init(128);    // ***note*** 4 is not big enough*****

//*******attach background tasks***********
  //OS_AddSwitchTasks(&SW1Push,&SW2Push,SWITCHPRI);
  
  NumCreated = 0 ;
// create initial foreground threads
  //NumCreated += OS_AddThread(&Interpreter,128,2); 
  NumCreated += OS_AddThread(&Consumer,128,1); 
  NumCreated += OS_AddThread(&PID,128,3);  // Lab 3, make this lowest priority										
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}
#endif


#ifdef DEBUG
//+++++++++++++++++++++++++DEBUGGING CODE++++++++++++++++++++++++
// ONCE YOUR RTOS WORKS YOU CAN COMMENT OUT THE REMAINING CODE
//LinkedList Test Code
int TestmainLL(void){
	tcbType* frontLL1=NULL;
	tcbType* endLL1=NULL;
	tcbType* semaFront=NULL;
	tcbType* semaEnd = NULL;
	tcbType tcbs[3];
	int i;
	for(i=0;i<3;i++){
		tcbs[i].ID=i;
		LLAdd(&frontLL1,&tcbs[i],&endLL1);
	}
	LLRemove(&frontLL1,&tcbs[1],&endLL1);
	LLAdd(&semaFront,&tcbs[1],&semaEnd);
	LLRemove(&frontLL1,&tcbs[2],&endLL1);
	LLAdd(&semaFront,&tcbs[2],&semaEnd);
	LLRemove(&semaFront,&tcbs[1],&semaEnd);
	LLAdd(&frontLL1,&tcbs[1],&endLL1);
	LLRemove(&semaFront,&tcbs[2],&semaEnd);
	LLAdd(&frontLL1,&tcbs[2],&endLL1);
 
 
	while(1){;}
}




//*******************Initial TEST**********
// This is the simplest configuration, test this first, (Lab 1 part 1)
// run this with 
// no UART interrupts
// no SYSTICK interrupts
// no timer interrupts
// no switch interrupts
// no ADC serial port or LCD output
// no calls to semaphores



unsigned long Count1;   // number of times thread1 loops
unsigned long Count2;   // number of times thread2 loops
unsigned long Count3;   // number of times thread3 loops
unsigned long Count4;   // number of times thread4 loops
unsigned long Count5;   // number of times thread5 loops
void Thread1(void){
  Count1 = 0;          
  for(;;){
    PE0 ^= 0x01;       // heartbeat
    Count1++;
    OS_Suspend(0);      // cooperative multitasking
  }
}
void Thread2(void){
  Count2 = 0;          
  for(;;){
    PE1 ^= 0x02;       // heartbeat
    Count2++;
    OS_Suspend(0);      // cooperative multitasking
  }
}
void Thread3(void){
  Count3 = 0;          
  for(;;){
    PE2 ^= 0x04;       // heartbeat
    Count3++;
    OS_Suspend(0);      // cooperative multitasking
  }
}

int Testmain1(void){  // Testmain1
  OS_Init();          // initialize, disable interrupts
  PortE_Init();       // profile user threads
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread1,128,1); 
  NumCreated += OS_AddThread(&Thread2,128,1); 
  NumCreated += OS_AddThread(&Thread3,128,0); 
  // Count1 Count2 Count3 should be equal or off by one at all times
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

//*******************Second TEST**********
// Once the initalize test runs, test this (Lab 1 part 1)
// no UART interrupts
// SYSTICK interrupts, with or without period established by OS_Launch
// no timer interrupts
// no switch interrupts
// no ADC serial port or LCD output
//no calls to semaphoreshyp
Sema4Type Ready2;
void Thread1b(void){
	int i;
  Count1 = 0;    
	OS_InitSemaphore(&Ready2,0);
  for(;;){
    PE0 ^= 0x01;       // heartbeat
    Count1++;
		OS_Sleep(10);
		OS_Kill();
  }
}

void Thread2b(void){
	int i;
	i = 0;
	Count2 = 0;
  for(;;){
		//OS_bWait(&Ready2);
    PE1 ^= 0x02;       // heartbeat
    Count2++;
		OS_Sleep(10);
		OS_Kill();
  }
}
void Thread3b(void){
	int i;
  Count3 = 0;          
  for(;;){
		//OS_bWait(&Ready2);
    PE2 ^= 0x04;       // heartbeat
    Count3++;
		//OS_bSignal(&Ready2);
  }
}
void Thread4b(void){
	int i;
  Count4 = 0;          
  for(;;){
    PE3 ^= 0x08;       // heartbeat
    Count4++;
  }
}
int Switch1Count=0; 
int Switch2Count=0;
 void DoNothing1(void){
	 PE3^=0x08;
	 Switch1Count++;
 }
 void DoNothing2(void){
	 Switch2Count++;
 }

int Testmain2(void){  // Testmain2

	OS_Init();           // initialize, disable interrupts
	//OS_MailBox_Init();
	//Output_Init();				//Initialize LCD
  PortE_Init();       // profile user threads

  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread1b,128,1); 
  NumCreated += OS_AddThread(&Thread2b,128,2); 
  NumCreated += OS_AddThread(&Thread3b,128,3);
	OS_AddSwitchTasks(&DoNothing1,&DoNothing2,0);	
  // Count1 Count2 Count3 should be equal on average
  // counts are larger than testmain1
 
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

//*******************Third TEST**********
// Once the second test runs, test this (Lab 1 part 2)
// no UART1 interrupts
// SYSTICK interrupts, with or without period established by OS_Launch
// Timer interrupts, with or without period established by OS_AddPeriodicThread
// PortF GPIO interrupts, active low
// no ADC serial port or LCD output
// tests the spinlock semaphores, tests Sleep and Kill
Sema4Type Readyc;        // set in background
int Lost;
void BackgroundThread1c(void){   // called at 1000 Hz
	PE0^=0x01;
  Count1++;
  OS_Signal(&Readyc);
}
void Thread5c(void){
  for(;;){
    OS_Wait(&Readyc);
		PE1^=0x02;
    Count5++;   // Count2 + Count5 should equal Count1 
    Lost = Count1-Count5-Count2;
  }
}
void Thread2c(void){
  OS_InitSemaphore(&Readyc,0);
  Count1 = 0;    // number of times signal is called      
  Count2 = 0;    
  Count5 = 0;    // Count2 + Count5 should equal Count1  
  NumCreated += OS_AddThread(&Thread5c,128,3); 
  OS_AddPeriodicThread(&BackgroundThread1c,4,1000,0);
  for(;;){
    OS_Wait(&Readyc);
		PE2^=0x04;
    Count2++;   // Count2 + Count5 should equal Count1
  }
}

void Thread3c(void){
  Count3 = 0;          
  for(;;){
		PE3^=0x08;
    Count3++;
  }
}
void Thread4c(void){ int i;
  for(i=0;i<64;i++){
    Count4++;
		PE4^=0x10;
    OS_Sleep(10);
  }
  OS_Kill();
  Count4 = 0;
}
void BackgroundThread5c(void){   // called when Select button pushed
  NumCreated += OS_AddThread(&Thread4c,128,3); 
}

void DoNothing(void){
	;
}
      
int Testmain3(void){   // Testmain3
  Count4 = 0;          
  OS_Init();           // initialize, disable interrupts
	PortE_Init();
// Count2 + Count5 should equal Count1
  NumCreated = 0 ;
  OS_AddSwitchTasks(&BackgroundThread5c,&DoNothing,0);
  NumCreated += OS_AddThread(&Thread2c,128,3); 
  NumCreated += OS_AddThread(&Thread3c,128,3); 
  NumCreated += OS_AddThread(&Thread4c,128,3); 
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

//*******************Fourth TEST**********
// Once the third test runs, run this example (Lab 1 part 2)
// Count1 should exactly equal Count2
// Count3 should be very large
// Count4 increases by 640 every time select is pressed
// NumCreated increase by 1 every time select is pressed

// no UART interrupts
// SYSTICK interrupts, with or without period established by OS_Launch
// Timer interrupts, with or without period established by OS_AddPeriodicThread
// Select switch interrupts, active low
// no ADC serial port or LCD output
// tests the spinlock semaphores, tests Sleep and Kill
Sema4Type Readyd;        // set in background
void BackgroundThread1d(void){   // called at 1000 Hz
static int i=0;
  i++;
  if(i==50){
    i = 0;         //every 50 ms
    Count1++;
    OS_bSignal(&Readyd);
		PE0^=0x01;
  }
}
void Thread2d(void){
  OS_InitSemaphore(&Readyd,0);
  Count1 = 0;          
  Count2 = 0;          
  for(;;){
    OS_bWait(&Readyd);
		PE1^=0x02;
    Count2++;     
  }
}
void Thread3d(void){
  Count3 = 0;          
  for(;;){
    Count3++;
		PE2^=0x04;
  }
}
void Thread4d(void){ int i;
  for(i=0;i<640;i++){
    Count4++;
    OS_Sleep(4);
		PE3^=0x08;
  }
  OS_Kill();
}

void doNothing(void)
{
	;
}

void BackgroundThread5d(void){   // called when Select button pushed
	PE4^=0x10;
  NumCreated += OS_AddThread(&Thread4d,128,3); 
}
int Testmain4(void){   // Testmain4
  Count4 = 0;          
  OS_Init();           // initialize, disable interrupts
	PortE_Init();
  NumCreated = 0 ;
  OS_AddPeriodicThread(&BackgroundThread1d,4,1000,0); //Timer2 

  OS_AddSwitchTasks(&BackgroundThread5d,&doNothing,2);

  NumCreated += OS_AddThread(&Thread2d,128,3); 
  NumCreated += OS_AddThread(&Thread3d,128,3); 
  NumCreated += OS_AddThread(&Thread4d,128,3); 
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

//******************* Lab 3 Preparation 2**********
// Modify this so it runs with your RTOS (i.e., fix the time units to match your OS)
// run this with 
// UART0, 115200 baud rate, used to output results 
// SYSTICK interrupts, period established by OS_Launch
// first timer interrupts, period established by first call to OS_AddPeriodicThread
// second timer interrupts, period established by second call to OS_AddPeriodicThread
// SW1 no interrupts
// SW2 no interrupts
unsigned long CountA;   // number of times Task A called
unsigned long CountB;   // number of times Task B called
unsigned long Count1;   // number of times thread1 loops

//*******PseudoWork*************
// simple time delay, simulates user program doing real work
// Input: amount of work in 100ns units (free free to change units
// Output: none
void PseudoWork(unsigned long work){
unsigned long startTime;
	long status;
	PE3=0x08;
  startTime = OS_Time();    // current cycle count
	unsigned long currentTime;
	//status=StartCritical();
  while(1){
		currentTime=OS_Time();
		if((startTime-currentTime) >= work){
			break;
		}
	}
	PE3=0x00;
	//EndCritical(status);
}
void Thread6(void){  // foreground thread
  Count1 = 0;          
  for(;;){
    Count1++; 
    PE0 ^= 0x01;        // debugging toggle bit 0  
  }
}
extern void Jitter(void);   // prints jitter information (write this)
void Thread7(void){  // foreground thread
  UART_OutString("\n\rEE445M/EE380L, Lab 3 Preparation 2\n\r");
  OS_Sleep(10000);   // 10 seconds        
  Jitter();         // print jitter information
  UART_OutString("\n\r\n\r");
  OS_Kill();
}
#define workA 5       // {5,50,500 us} work in Task A
#define workAcycles 40
#define counts1us 10    // number of OS_Time counts per 1us
#define PERIODA 80000
void TaskA(void){       // called every {1000, 2990us} in background
	uint32_t jitter;
	static uint32_t LastTime=0;
	uint32_t thisTime;
	thisTime = OS_Time();
  PE1 = 0x02;      // debugging profile  
  CountA++;
  PseudoWork(workAcycles); //  do work (100ns time resolution)
  PE1 = 0x00;      // debugging profile  
	if(LastTime!=0){
	unsigned long diff = OS_TimeDifference(LastTime,thisTime);
	jitter = (diff > (PERIODA+workAcycles)) ? (diff-(PERIODA+workAcycles)+4)/8:(PERIODA+workAcycles-diff+4)/8; // in 0.1 usec
	
	if(jitter > MaxJitter)
	{
		MaxJitter = jitter; // in usec
	}       // jitter should be 0
	if(jitter >= JitterSize)
	{
		jitter = JITTERSIZE-1;
	}
	//JitterHistogram[jitter]++;
}
	LastTime = thisTime;	
}
#define workB 250       // 250 us work in Task B
#define workBcycles 20000
#define PERIODB 20000
uint32_t MaxJitterB=0;
void TaskB(void){       // called every pB in background
	uint32_t jitter;
	static uint32_t LastTime=0;
	uint32_t thisTime;
	thisTime = OS_Time();
  PE2 = 0x04;      // debugging profile  
  CountB++;
  PseudoWork(workBcycles); //  do work (100ns time resolution)
  PE2 = 0x00;      // debugging profile 
if(LastTime!=0){	
	unsigned long diff = OS_TimeDifference(LastTime,thisTime);
	jitter = (diff > (PERIODB+workBcycles)) ? (diff-(PERIODB+workBcycles)+4)/8:(PERIODB+workBcycles-diff+4)/8; // in 0.1 usec
	
	if(jitter > MaxJitterB)
	{
		MaxJitterB = jitter; // in usec
	}       // jitter should be 0
	if(jitter >= JitterSize)
	{
		jitter = JITTERSIZE-1;
	}
	JitterHistogram[jitter]++;
}
	LastTime = thisTime;	
}

int Testmain5(void){       // Testmain5 Lab 3
  PortE_Init();
  OS_Init();           // initialize, disable interrupts
	UART_Init();
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread6,128,2); 
  NumCreated += OS_AddThread(&Thread7,128,1); 
  OS_AddPeriodicThread(&TaskA,4,1000,0);           // 1 ms, higher priority, Timer2
  OS_AddPeriodicThread(&TaskB,6,2000,1);         // 2 ms, lower priority, Timer3
 
  OS_Launch(TIME_2MS); // 2ms, doesn't return, interrupts enabled in here
  return 0;             // this never executes
}


//******************* Lab 3 Preparation 4**********
// Modify this so it runs with your RTOS used to test blocking semaphores
// run this with 
// UART0, 115200 baud rate,  used to output results 
// SYSTICK interrupts, period established by OS_Launch
// first timer interrupts, period established by first call to OS_AddPeriodicThread
// second timer interrupts, period established by second call to OS_AddPeriodicThread
// SW1 no interrupts, 
// SW2 no interrupts
Sema4Type s;            // test of this counting semaphore
unsigned long SignalCount1;   // number of times s is signaled
unsigned long SignalCount2;   // number of times s is signaled
unsigned long SignalCount3;   // number of times s is signaled
unsigned long WaitCount1;     // number of times s is successfully waited on
unsigned long WaitCount2;     // number of times s is successfully waited on
unsigned long WaitCount3;     // number of times s is successfully waited on
#define MAXCOUNT 20000
void OutputThread(void){  // foreground thread
  UART_OutString("\n\rEE345M/EE380L, Lab 3 Preparation 4\n\r");
  while((SignalCount1+SignalCount2+SignalCount3)<100*MAXCOUNT){
    OS_Sleep(1000);   // 1 second
    UART_OutString(".");
  }       
  UART_OutString(" done\n\r");
  UART_OutString("Signalled="); UART_OutUDec(SignalCount1+SignalCount2+SignalCount3);
  UART_OutString(", Waited="); UART_OutUDec(WaitCount1+WaitCount2+WaitCount3);
  UART_OutString("\n\r");
  OS_Kill();
}
void Wait1(void){  // foreground thread
  for(;;){
    OS_Wait(&s);    // three threads waiting
    WaitCount1++; 
  }
}
void Wait2(void){  // foreground thread
  for(;;){
    OS_Wait(&s);    // three threads waiting
    WaitCount2++; 
  }
}
void Wait3(void){   // foreground thread
  for(;;){
    OS_Wait(&s);    // three threads waiting
    WaitCount3++; 
  }
}
void Signal1(void){      // called every 799us in background
  if(SignalCount1<MAXCOUNT){
    OS_Signal(&s);
    SignalCount1++;
  }
}
// edit this so it changes the periodic rate
void Signal2(void){       // called every 1111us in background
  if(SignalCount2<MAXCOUNT){
    OS_Signal(&s);
    SignalCount2++;
  }
}
void Signal3(void){       // foreground
  while(SignalCount3<98*MAXCOUNT){
    OS_Signal(&s);
    SignalCount3++;
  }
  OS_Kill();
}

long add(const long n, const long m){
static long result;
  result = m+n;
  return result;
}
int Testmain6(void){      // Testmain6  Lab 3
  volatile unsigned long delay;
  OS_Init();           // initialize, disable interrupts
	UART_Init();
  delay = add(3,4);
  PortE_Init();
  SignalCount1 = 0;   // number of times s is signaled
  SignalCount2 = 0;   // number of times s is signaled
  SignalCount3 = 0;   // number of times s is signaled
  WaitCount1 = 0;     // number of times s is successfully waited on
  WaitCount2 = 0;     // number of times s is successfully waited on
  WaitCount3 = 0;	  // number of times s is successfully waited on
  OS_InitSemaphore(&s,0);	 // this is the test semaphore
  OS_AddPeriodicThread(&Signal1,4,(799*TIME_1MS)/1000,0);   // 0.799 ms, higher priority
  OS_AddPeriodicThread(&Signal2,6,(1111*TIME_1MS)/1000,1);  // 1.111 ms, lower priority
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread6,128,6);    	// idle thread to keep from crashing
  NumCreated += OS_AddThread(&OutputThread,128,2); 	// results output thread
  NumCreated += OS_AddThread(&Signal3,128,2); 	// signalling thread
  NumCreated += OS_AddThread(&Wait1,128,2); 	// waiting thread
  NumCreated += OS_AddThread(&Wait2,128,2); 	// waiting thread
  NumCreated += OS_AddThread(&Wait3,128,2); 	// waiting thread
 
  OS_Launch(TIME_1MS);  // 1ms, doesn't return, interrupts enabled in here
  return 0;             // this never executes
}


//******************* Lab 3 Measurement of context switch time**********
// Run this to measure the time it takes to perform a task switch
// UART0 not needed 
// SYSTICK interrupts, period established by OS_Launch
// first timer not needed
// second timer not needed
// SW1 not needed, 
// SW2 not needed
// logic analyzer on PF1 for systick interrupt (in your OS)
//                on PE0 to measure context switch time
void Thread8(void){       // only thread running
  while(1){
    PE0 ^= 0x01;      // debugging profile  
  }
}
int Testmain7(void){       // Testmain7
  PortE_Init();
  OS_Init();           // initialize, disable interrupts
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread8,128,2); 
  OS_Launch(TIME_1MS/10); // 100us, doesn't return, interrupts enabled in here
  return 0;             // this never executes
}
#endif
