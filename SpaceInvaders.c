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

void getTemperature(void) {
	ADC0_PSSI_R |= 8;           /* start a conversion sequence 3 */
	while((ADC0_RIS_R & 0x08) == 0); /* wait for conversion to complete */
	temp = ((ADC0_SSFIFO3_R * 330) / 4096);
	ADC0_ISC_R = 8;             /* clear completion flag  */
}

int main(void){
	
  TExaS_Init(SSI0_Real_Nokia5110_Scope);  // set system clock to 80 MHz
	UART_Init(); 
	PortB_Init();
	PortE_Init();
	Nokia5110_Init();
	Nokia5110_ClearBuffer();
	Nokia5110_DisplayBuffer();
	Timer2_Init(CHECK_TIMER);
	
	Nokia5110_Clear();
	
	getTemperature();
	temp_readings[0] = temp;
	temp_readings[1] = temp;
	temp_readings[2] = temp;
	calcAvgTemp();
	
	updateDisplay();
	
  while(1){
		if (increaseTempFlag == 1) {
			increaseTempFlag = 0;
			target++;
			updateDisplay();
		} else if (decreaseTempFlag == 1) {
			decreaseTempFlag = 0;
			target--;
			updateDisplay();
		}
		
		if (checkTempFlag) {
			checkTempFlag = 0;
			getTemperature();
			updateTempReadings();
			GPIO_PORTB_DATA_R &= ~(1 << 2 | 1 << 3);
			UART_OutString("\n\n");
			if (avg_temp_reading + THRESHOLD_TEMP >= target && avg_temp_reading - THRESHOLD_TEMP <= target) {
				UART_OutString("TURNED OFF");
				currentMode = 0;
				updateDisplay();
				OutCRLF();
			} else if (avg_temp_reading > target) {
				GPIO_PORTB_DATA_R |= (1 << 2);
				UART_OutString("COOLING");
				currentMode = 1;
				updateDisplay();
				OutCRLF();
			} else if (avg_temp_reading < target) {
				GPIO_PORTB_DATA_R |= (1 << 3);
				UART_OutString("HEATING");
				currentMode = 2;
				updateDisplay();
				OutCRLF();
			}
			UART_OutString("the average ambient temperature : ");
			UART_OutUDec(avg_temp_reading);  
			OutCRLF();
			UART_OutString("the target temperature : ");
			UART_OutUDec(target);		
			OutCRLF();
		}
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
void OutCRLF(void){
  UART_OutChar(CR);
  UART_OutChar(LF);
}
void GPIOPortB_Handler(void) {
	if(GPIO_PORTB_RIS_R&(1 << 4)) {
		GPIO_PORTB_ICR_R |=(1 << 4); //acknolagement interrupt in port B for pin 4
		increaseTempFlag = 1;
	} else if(GPIO_PORTB_RIS_R & (1 << 5)) {
		GPIO_PORTB_ICR_R |= (1 << 5); //acknolagement interrupt in port B for pin 5
		decreaseTempFlag = 1;
	}
}
void updateTempReadings(void) {
	temp_readings[timerCount % 3] = temp;
	calcAvgTemp();
}

void calcAvgTemp(void) {
	avg_temp_reading = (temp_readings[0] + temp_readings[1] + temp_readings[2]) / 3;
}
void Timer2_Init(unsigned long period){ 
  unsigned long volatile delay;
  SYSCTL_RCGCTIMER_R |= 0x04;   // 0) activate timer2
  delay = SYSCTL_RCGCTIMER_R;
  timerCount = 0;
  checkTempFlag = 0;
  TIMER2_CTL_R = 0x00000000;    // 1) disable timer2A during setup
  TIMER2_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER2_TAMR_R |= (1 << 1);   // 3) configure for periodic mode, default down-count settings
  TIMER2_TAILR_R = period-1;    // 4) reload value
  TIMER2_TAPR_R = 0;            // 5) bus clock resolution
  TIMER2_ICR_R |= (1 << 0);    // 6) clear timer2A timeout flag
  TIMER2_IMR_R |= (1 << 0);    // 7) arm timeout interrupt
  NVIC_EN0_R |= 1 << 23;           // 8) vector number 39, enable IRQ 23 in NVIC
  TIMER2_CTL_R |= (1 << 0);    // 9) enable timer2A
}

void PortB_Init(void) {
	volatile unsigned long delay;
	SYSCTL_RCGC2_R |= (1<<1);      // 1) B clock
  delay = SYSCTL_RCGC2_R;           // delay   
  GPIO_PORTB_LOCK_R = 0x4C4F434B;   // 2) unlock PortB  
  GPIO_PORTB_CR_R |= (1 << 2 | 1 << 3 | 1 << 4 | 1 << 5);           // allow changes to PB2-5       
  GPIO_PORTB_AMSEL_R &= ~(1 << 2 | 1 << 3 | 1 << 4 | 1 << 5);        // 3) disable analog function
  GPIO_PORTB_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL  
  GPIO_PORTB_DIR_R |= (1 << 2 | 1 << 3);      // 5) PB2, PB3 => OUTPUT LED
	GPIO_PORTB_DIR_R &= ~(1 << 4 | 1 << 5);			// 6) PB4, PB5 => INPUT SWITCHES
  GPIO_PORTB_AFSEL_R = 0x00;        // 6) no alternate function
  GPIO_PORTB_PUR_R |= (1 << 4 | 1 << 5);          // enable pullup resistors on PF4,PF0       
  GPIO_PORTB_DEN_R |= (1 << 2 | 1 << 3 | 1 << 4 | 1 << 5);          // 7) enable digital pins PF4-PF0 
	
  GPIO_PORTB_IM_R |= (1 << 4 | 1 << 5); // 8) enable interrupt on PB4-5
	GPIO_PORTB_IBE_R &= ~(1 << 4 | 1 << 5); // 9) disable both edges on PB4-5
	GPIO_PORTB_IEV_R &= ~(1 << 4 | 1 << 5); // 10) trigger on falling edges for PB4-5
	NVIC_EN0_R|=(1<<1); // 11) vector number 17, enable IRQ 1 in NVIC
}

// Configure PE3
void PortE_Init(void) {
	SYSCTL_RCGCGPIO_R |= (1 << 4);      /* enable clock to GPIO_PORTE */
	SYSCTL_RCGCADC_R |= (1 << 0);          /* enable clock to ADC0 */
	GPIO_PORTE_AFSEL_R |= (1 << 3);        /* enable alternate function */
	GPIO_PORTE_DEN_R &= ~(1 << 3);         /* disable digital function */
	GPIO_PORTE_AMSEL_R |= (1 << 3);        /* enable analog function */
	ADC0_ACTSS_R &= ~(1 << 3);             /* disable SS3 during configuration */
	ADC0_EMUX_R &= ~0xF000;         /* software trigger conversion */
	ADC0_SSMUX3_R = 0;              /* get input from channel 0 */
	ADC0_SSCTL3_R |= (1 << 1 | 1 << 2);             /* take one sample at a time, set flag at 1st sample */
	ADC0_ACTSS_R |= (1 << 3);              /* enable ADC0 sequencer 3 */
}
void Timer2A_Handler(void){ 
  TIMER2_ICR_R |= (1 << 0);   // acknowledge timer2A timeout
  timerCount++;
  checkTempFlag = 1;
}