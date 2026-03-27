// Motors.h
// Runs on TM4C123 for CECS347 Project 2
#define TOTAL_PERIOD 16000 //  16MHz/1000=16000
#define START_SPEED 16000*0.5
#define FASTER      16000*0.7
    
// configure the system to get its clock from the PLL
void Motors_Init(void);

void Motors_Duty(unsigned long left , unsigned long right);