// LEDSW.c
// Runs on TM4C123 for CECS347 Project 2
#include "tm4c123gh6pm.h"
#include "LEDSW.h"

void Car_Dir_Init(void){
	if ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOB)==0) {
		SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB;	// Activate B clocks
		while ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOB)==0){};
	}
		
  GPIO_PORTB_AMSEL_R &= ~0xCC;	// disable analog function
	GPIO_PORTB_AFSEL_R &= ~0xCC;	// no alternate function
  GPIO_PORTB_PCTL_R &= ~0xFF00FF00;	// GPIO clear bit PCTL 
	GPIO_PORTB_DIR_R |= 0xCC; // output on pin(s)
  GPIO_PORTB_DEN_R |= 0xCC;	// enable digital I/O on pin(s)
}

// Port F Initialization
void LED_Init(void){
    unsigned long volatile delay;
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF; // (a) activate clock for port F
  delay = SYSCTL_RCGC2_R;
  GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY; // unlock GPIO Port F
    GPIO_PORTF_CR_R |= 0x1F;                 // allow changes to PF4-0 :11111->0x1F     
  GPIO_PORTF_DIR_R &= ~0x11;          // PF4,PF0 input   
  GPIO_PORTF_DIR_R |= 0x0E;              // PF3,PF2,PF1 output   
  GPIO_PORTF_AMSEL_R &= ~0x1F;        // disable analog function
  GPIO_PORTF_DEN_R |= 0x1F;     //     enable digital I/O on PF4->0
  GPIO_PORTF_PCTL_R &= ~0x000FFFFF;     // GPIO clear bit PCTL  
    GPIO_PORTF_AFSEL_R &= ~0x1F;        // no alternate function
    //SW Interrupt
  GPIO_PORTF_PUR_R |= 0x11;     //     enable weak pull-up on PF4,0
  GPIO_PORTF_IS_R &= ~0x11;     // (d) PF4,PF0 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;    //     PF4,PF0 is not both edges
  GPIO_PORTF_IEV_R &= ~0x11;    //     PF4,PF0 falling edge event
  GPIO_PORTF_ICR_R = 0x11;      // (e) clear flags 4,0
  GPIO_PORTF_IM_R |= 0x11;      // (f) arm interrupt on PF4,PF0
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF1FFFFF)|0x00400000; // (g) bits:23-21 for PORTF, set priority to 2
  NVIC_EN0_R |= 0x40000000;      // (h) enable interrupt 30 in NVIC
}
