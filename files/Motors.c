// Motors.c
// Runs on TM4C123 for CECS347 Project 2
#include "tm4c123gh6pm.h"
#include "Motors.h"

// The #define statement SYSDIV2 in PLL.h
// configure the system to get its clock from the PLL

void Motors_Init(void){
	if ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOB)==0) {
		SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB;	// Activate B clocks
		while ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOB)==0){};
	}
	
	GPIO_PORTB_AFSEL_R |= 0x30;	// enable alt funct: PB45 for PWM
  GPIO_PORTB_PCTL_R &= ~0x00FF0000; // PWM to be used
  GPIO_PORTB_PCTL_R |= 0x00440000; // PWM to be used
  GPIO_PORTB_DEN_R |= 0x30;	// enable digital I/O 
	
	// Initializes PWM settings
	SYSCTL_RCGCPWM_R |= 0x01;	// activate PWM0
	SYSCTL_RCC_R &= ~0x001E0000; // Clear any previous PWM divider values
	
	// PWM0_0 output A&B Initialization for PB76
	PWM0_1_CTL_R = 0;	// re-loading down-counting mode
	PWM0_1_GENA_R |= 0xC8;	// low on LOAD, high on CMPA down
	PWM0_1_GENB_R |= 0xC08;// low on LOAD, high on CMPB down
	PWM0_1_LOAD_R = TOTAL_PERIOD - 1;	// cycles needed to count down to 0
  PWM0_1_CMPA_R = 0;	// count value when output rises
	PWM0_1_CMPB_R = 0;	// count value when output rises
	
	PWM0_1_CTL_R |= 0x00000001;	// Enable PWM0 Generator 0 in Countdown mode
	PWM0_ENABLE_R &= ~0x0000000C;	// Disable PB76:PWM0 output 0&1 on initialization
}

void Motors_Duty(unsigned long left, unsigned long right){
	PWM0_1_CMPA_R = left - 1;	// PB6 count value when output rises
  PWM0_1_CMPB_R = right - 1;	// PB7 count value when output rises
}
