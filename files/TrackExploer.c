// TrackExplorer.c
// Runs on TM4C123
// This is the starter file for CECS 347 Project 2 - A Track Explorer
// This project uses hardware PWM to control two DC Motors, 
// ADC to collect analog inputs from three Sharp IR sensors.
// The three Sharp analog IR distance sensors (GP2Y0A21YK0F) are used
// to allowthe robot to navigate through a track with two walls: 
// one mounted looking directly forward to avoid a head-on collision, 
// the other two looking forward to the left and to the right to detect  
// the distances between the car and the two walls. The goal is to 
// control power to each wheel so the left and right distances to the 
// walls are equal.
// If an object is detected too close to the robot, 
// the robot should be able to avoid it.
/*
    ------------------------------------------wall---------
                      /
                     /
                    / 
                   /
         -----------
         |         |
         | Robot   | ---> direction of motion and third sensor
         |         |
         -----------
                   \
                    \
                     \
                      \
    ------------------------------------------wall---------
*/
// The original project is designed by Dr. Daniel Valvano, Jonathan Valvano
// September 12, 2013
// Modifications are made by Dr. Min He.

// PE1 connected to forward facing IR distance sensor
// PE4 connected to right IR distance sensor
// PE5 connected to left IR distance sensor

#include "tm4c123gh6pm.h"
#include "Sensors.h"
#include "Motors.h"
#include "LEDSW.h"
#include "PLL.h"
#include "stdint.h"
#include "stdlib.h" 
#include "stdbool.h"
#define SW2  0x01
#define SW1  0x10


// basic functions defined at end of startup.s
extern void DisableInterrupts(void); // Disable interrupts
extern void EnableInterrupts(void);  // Enable interrupts
extern void WaitForInterrupt(void);  // low power mode
void Delay(void);

// You use datasheet to calculate the following ADC values
// then test your sensors to adjust the values 
#define CRASH             IR15CM// if there is less than this distance ahead of the robot, it will immediately stop
#define IR10CM            3200
#define IR15CM            2400  // ADC output for 15cm:1.8v -> (1.8/3.3)*4095=2233 
#define IR20CM            1780  // ADC output for 20cm:1.39v -> (1.39/3.3)*4095=1724
#define IR30CM            1190  // ADC output for 30cm:0.9v -> (0.9/3.3)*4095=1116
#define IR40CM            860   // ADC output for 40cm:0.74v -> (0.74/3.3)*4095=918
#define IR80CM            700   // ADC output for 80cm:0.4v -> (0.4/3.3)*4095=496
                                // with equal power to both motors (LeftH == RightH), the robot still may not drive straight
                                // due to mechanical differences in the motors, so bias the left wheel faster or slower than
                                // the constant right wheel
#define LEFTPOWER        	0.5*PERIOD   // duty cycle of left wheel 
#define RIGHTPOWER        0.5*PERIOD    // duty cycle of left wheel 

int left =0;
bool R;
int right =0;
int diff;
void System_Init(void);
void steering(uint16_t ahead_dist,uint16_t right_dist, uint16_t left_dist);
int start;

int main(void){
  uint16_t left, right, ahead;
  
  DisableInterrupts();  // disable interrupts while initializing
  System_Init();
  EnableInterrupts();   // enable after all initialization are done
	//LED_Init();
  //Car_Dir_Init();
  //Motors_Init();
	Motors_Duty(TOTAL_PERIOD, TOTAL_PERIOD);
	
	// moving forward
//////	LED = Green;
//////	WHEEL_DIR = FORWARD;
//////	PWM0_ENABLE_R |= 0x0000000C; // enable both wheels
//////	Delay();

  // TODO: Calibrate the sensors: read at least 5 times from the sensor 
	// before the car starts to move: this will allow software to filter the sensor outputs.	


	// TODO: start with moving forward, LED green 
	for(int x =0; x < 9; x++){
		ReadSensorsMedianFilter(&ahead, &right, &left);
	}
	
		while(1){
			// choose one of the following three software filter methods
			ReadSensorsMedianFilter(&ahead, &right, &left);
			steering(ahead,right,left);
			
		}
		
}

void System_Init(void) {
  PLL_Init();           // bus clock at 80 MHz
  Sensors_Init();        // initialize ADC to sample AIN2 (PE1), AIN9 (PE4), AIN8 (PE5)
  LED_Init();         // configure onboard LEDs and push buttons
	Car_Dir_Init();
  Motors_Init();         // Initialize signals for the two DC Motors
}

void GPIOPortF_Handler(void){
	  if(GPIO_PORTF_RIS_R&SW2){  // SW2 touch
			GPIO_PORTF_ICR_R = SW2;  // acknowledge flag0
			start = 0;
/*    if(MinL > 8000){
      MinL = MinL - 8000;     // slow down
      LeftL = LeftL - 8000;
      LeftH = 80000 - LeftL;  // constant period of 1ms, variable duty cycle
      RightL = RightL - 8000;
      RightH = 80000 - RightL;// constant period of 1ms, variable duty cycle
    }*/
  }
		if(GPIO_PORTF_RIS_R&SW1){  // SW1 touch
			GPIO_PORTF_ICR_R = SW1;  // acknowledge flag4
			start = 1;
/*    if(MinL < 72000){
      MinL = MinL + 8000;     // speed up
      LeftL = LeftL + 8000;
      LeftH = 80000 - LeftL;  // constant period of 1ms, variable duty cycle
      RightL = RightL + 8000;
      RightH = 80000 - RightL;// constant period of 1ms, variable duty cycle
    }*/
  }
}


void steering(uint16_t ahead_dist,uint16_t right_dist, uint16_t left_dist){
		if(start){
			bool pivot;
			
					if (ahead_dist > IR20CM){
					//LED = Dark;
					diff = (left_dist - right_dist);
					if(diff < 0){
					R = true;
					}else{
					R = false;
					}
					if(!R){
						WHEEL_DIR = LEFTPIVOT;
					}else{
						WHEEL_DIR = RIGHTPIVOT;
					}
					PWM0_ENABLE_R |= 0x0C;
					Motors_Duty(FASTER, FASTER);
					}else if(ahead_dist < IR80CM && left_dist < IR80CM && right_dist < IR80CM){
					PWM0_ENABLE_R &= ~0x0C;
					LED = Blue;
				}else if(ahead_dist < IR30CM && left_dist > IR30CM && right_dist < IR30CM){
						// Forward right turn
					//LED = Purple;
					WHEEL_DIR=FORWARD;
					PWM0_ENABLE_R &= ~0x00000008; // Disable right wheel
					PWM0_ENABLE_R |= 0x00000004; // Enable left wheel
					Motors_Duty(FASTER, FASTER);
					//Delay();
				}else if(ahead_dist < IR30CM && left_dist < IR30CM && right_dist > IR30CM){
						// Forward left turn
					//LED = Yellow;
					WHEEL_DIR=FORWARD;
					PWM0_ENABLE_R |= 0x00000008; // Enable right wheel
					PWM0_ENABLE_R &= ~0x00000004; // Disable left wheel
					Motors_Duty(FASTER, FASTER);
					//Delay();
				}else{
					diff = (left_dist - right_dist);
			if(diff < 0){
			R = true;
			}else{
			R = false;
			}
				if(abs(diff) < 500 ){
				left = START_SPEED;
				right = START_SPEED;
				}else if(abs(diff) < 1200 ){
					if(R){
						left = START_SPEED;
						right = TOTAL_PERIOD *0.6;
					}else{
						left = TOTAL_PERIOD *0.6;
						right = START_SPEED;
					}
				}else{
					if(R){
						left = START_SPEED;
						right = TOTAL_PERIOD *0.9;
					}else{
						left = TOTAL_PERIOD *0.9;
						right = START_SPEED;
					}
				}
				
				Motors_Duty(left, right);
				//LED = Green;
				WHEEL_DIR = FORWARD;
				PWM0_ENABLE_R |= 0x0000000C; // enable both wheels
				}
				
				if(ahead_dist > IR10CM || left_dist > IR10CM || right_dist > IR10CM){
					LED = Red;
				}else if(ahead_dist < IR10CM && left_dist < IR10CM && right_dist < IR10CM){
					LED = Green;
				}else if(ahead_dist < IR80CM && left_dist < IR80CM && right_dist < IR80CM){
					LED = Blue;
				}
			}
		else{
			LED = Dark;
			PWM0_ENABLE_R &= ~0x0C;
		}
}

void Delay(void){
	unsigned long volatile time;
  time = 727240*500/91;  // 0.25sec
  while(time){
		time--;
  }
}
