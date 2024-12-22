// Air Conditioner Project

// December 22, 2024
/* 
	The provided code implements a basic temperature control system using the TM4C123GH6PM microcontroller. 
	The system reads the ambient temperature, allows the user to set a target temperature, and controls heating or cooling devices to maintain the desired temperature. 
	It also displays the current status on a Nokia5110 LCD and communicates over UART.

	Team Members:
		1. Fady Youssef
		2. Abdelrahman Osman
		3. Karim Wael
	
	University: Benha University
	College: Shoubra Faculty of Engineering
	Department: Computer Engineering
 */
 
// ******* Required Hardware I/O connections*******************
// Resistor between LEDs and the microcontroller pins.
// Increase Temperature Switch on PB4, and GND
// Decrease Temperature Switch on PB5, and GND
// Red LED on PB3, and GND
// Blue LED on PB2, and GND
// LM35 Sensor on PE3, GND, and VCC

// Blue Nokia 5110
// ---------------
// Signal        (Nokia 5110) LaunchPad pin
// Reset         (RST, pin 1) connected to PA7
// SSI0Fss       (CE,  pin 2) connected to PA3
// Data/Command  (DC,  pin 3) connected to PA6
// SSI0Tx        (Din, pin 4) connected to PA5
// SSI0Clk       (Clk, pin 5) connected to PA2
// 3.3V          (Vcc, pin 6) power
// back light    (BL,  pin 7) not connected, consists of 4 white LEDs which draw ~80mA total
// Ground        (Gnd, pin 8) ground

#include <stdio.h>
#include "tm4c123gh6pm.h"
#include "Nokia5110.h"
#include "Random.h"
#include "TExaS.h"
#include "UART.c"

void DisableInterrupts(void); // Disable interrupts
void WaitForInterrupt(void);   
void EnableInterrupts(void);  // Enable interrupts
void Timer2_Init(unsigned long period); //Initialization of timer
void PortB_Init(void);	//Initialization of Port B
void PortE_Init(void); //Initialization of Port E
void getTemperature(void);
void OutCRLF(void);
void updateDisplay(void);
void calcAvgTemp(void);
void updateTempReadings(void);

unsigned long timerCount;
unsigned int checkTempFlag;
const short THRESHOLD_TEMP = 1;
const int CHECK_TIMER = 80000000 * 5; //5 seconds before checking the state of the device(80MHZ for 1 second)
short increaseTempFlag = 0; // Increase temperature flag
short decreaseTempFlag = 0; // Decrease temperature flag
short temp; // Current temperature
short avg_temp_reading; // Avg temp reading
short target = 30; // Target temperature
short currentMode = 0; // Current mode, 0 = "Turned OFF", 1 = "Cooling" and 2 = "Heating"
short temp_readings[3] = {0, 0, 0};

int main(void){
  TExaS_Init(SSI0_Real_Nokia5110_Scope);  // set system clock to 80 MHz
  Random_Init(1);
  Nokia5110_Init();
  Nokia5110_ClearBuffer();
	Nokia5110_DisplayBuffer();      // draw buffer


  //Nokia5110_Clear();
  //Nokia5110_SetCursor(2, 4);
  //Nokia5110_OutUDec(1234);
  while(1){
  }

}


// You can use this timer only if you learn how it works
void Timer2_Init(unsigned long period){ 
  unsigned long volatile delay;
  SYSCTL_RCGCTIMER_R |= 0x04;   // 0) activate timer2
  delay = SYSCTL_RCGCTIMER_R;
  TimerCount = 0;
  Semaphore = 0;
  TIMER2_CTL_R = 0x00000000;    // 1) disable timer2A during setup
  TIMER2_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER2_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
  TIMER2_TAILR_R = period-1;    // 4) reload value
  TIMER2_TAPR_R = 0;            // 5) bus clock resolution
  TIMER2_ICR_R = 0x00000001;    // 6) clear timer2A timeout flag
  TIMER2_IMR_R = 0x00000001;    // 7) arm timeout interrupt
  NVIC_PRI5_R = (NVIC_PRI5_R&0x00FFFFFF)|0x80000000; // 8) priority 4
// interrupts enabled in the main program after all devices initialized
// vector number 39, interrupt number 23
  NVIC_EN0_R = 1<<23;           // 9) enable IRQ 23 in NVIC
  TIMER2_CTL_R = 0x00000001;    // 10) enable timer2A
}

void Timer2A_Handler(void){ 
  TIMER2_ICR_R = 0x00000001;   // acknowledge timer2A timeout
  TimerCount++;
  Semaphore = 1; // trigger
}
