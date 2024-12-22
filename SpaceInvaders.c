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

void getTemperature(void) {
	ADC0_PSSI_R |= 8;           /* start a conversion sequence 3 */
	while((ADC0_RIS_R & 0x08) == 0); /* wait for conversion to complete */
	temp = ((ADC0_SSFIFO3_R * 330) / 4096);
	ADC0_ISC_R = 8;             /* clear completion flag  */
}

int main(void){
  TExaS_Init(SSI0_Real_Nokia5110_Scope);  // set system clock to 80 MHz
  Random_Init(1);
  Nokia5110_Init();
  Nokia5110_ClearBuffer();
	Nokia5110_DisplayBuffer();      // draw buffer

  while(1){
  }

}

void updateDisplay(void) {
	char buffer[10];
	char *str = " C";
	
	Nokia5110_Clear();
	Nokia5110_SetCursor(0, 0);
  Nokia5110_OutString("Target temp: ");
	Nokia5110_SetCursor(3, 2);
	
	sprintf(buffer, "%d%s", target, str);
	Nokia5110_OutString(buffer);
	Nokia5110_SetCursor(2, 4);
	if (currentMode == 0) {
		Nokia5110_OutString("Turned OFF");
	} else if (currentMode == 1) {
		Nokia5110_OutString("Cooling");
	} else if (currentMode == 2) {
		Nokia5110_OutString("Heating");
	}
}

void updateTempReadings(void) {
	temp_readings[timerCount % 3] = temp;
	calcAvgTemp();
}

void calcAvgTemp(void) {
	avg_temp_reading = (temp_readings[0] + temp_readings[1] + temp_readings[2]) / 3;
}
