// TrackExplorer.c
// Runs on TM4C123
// This project uses hardware PWM to control two DC Motors, 
// ADC to collect analog inputs from three Sharp IR sensors.
// The three Sharp analog IR distance sensors (GP2Y0A21YK0F) are used
// to allow the robot to navigate through a track with two walls: 
// one mounted looking directly forward to avoid a head-on collision, 
// the other two looking forward to the left and to the right to detect  
// the distances between the car and the two walls. The goal is to 
// control power to each wheel so the left and right distances to the 
// walls are equal, if the car gets too close to a wall (cornered) find a new path
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

// Onboard push button bit masks for Port F
#define SW2  0x01   // SW2 is bit 0 of Port F (PF0)
#define SW1  0x10   // SW1 is bit 4 of Port F (PF4)

// basic functions defined at end of startup.s
extern void DisableInterrupts(void); // Disable interrupts
extern void EnableInterrupts(void);  // Enable interrupts
extern void WaitForInterrupt(void);  // low power mode
void Delay(void);

// ADC threshold values derived from the GP2Y0A21YK0F datasheet voltage outputs.
// Higher ADC value = object is closer (inverse relationship between distance and voltage).
// These were tuned experimentally to match actual sensor behavior.
#define CRASH             IR15CM    // minimum safe distance ahead before stopping
#define IR10CM            3200      // ADC value when object is ~10cm away
#define IR15CM            2400      // ADC output for 15cm: 1.8V -> (1.8/3.3)*4095=2233 
#define IR20CM            1780      // ADC output for 20cm: 1.39V -> (1.39/3.3)*4095=1724
#define IR30CM            1190      // ADC output for 30cm: 0.9V -> (0.9/3.3)*4095=1116
#define IR40CM            860       // ADC output for 40cm: 0.74V -> (0.74/3.3)*4095=918
#define IR80CM            700       // ADC output for 80cm: 0.4V -> (0.4/3.3)*4095=496

// Default 50% duty cycle for both wheels as a baseline starting point.
// Actual duty cycles are adjusted dynamically in the steering function.
#define LEFTPOWER         0.5*PERIOD    // duty cycle of left wheel 
#define RIGHTPOWER        0.5*PERIOD    // duty cycle of left wheel 

// Global variables used by both the steering function and the interrupt handler.
// Declared globally so the ISR can modify 'start' without needing to pass parameters.
int left = 0;       // computed duty cycle for left wheel
bool R;             // true = steer right, false = steer left
int right = 0;      // computed duty cycle for right wheel
int diff;           // difference between left and right sensor readings, used for centering

void System_Init(void);
void steering(uint16_t ahead_dist, uint16_t right_dist, uint16_t left_dist);

// start flag: controlled by onboard push buttons via interrupt.
// SW1 sets start=1 (run), SW2 sets start=0 (stop).
int start;

int main(void){
  uint16_t left, right, ahead;    // local sensor readings updated every loop iteration
  
  DisableInterrupts();  // disable interrupts while initializing
  System_Init();
  EnableInterrupts();   // enable after all initialization are done

  // Set both motors to full period initially (motors off until PWM is enabled)
  Motors_Duty(TOTAL_PERIOD, TOTAL_PERIOD);
	
  // Warm up the median filter by reading sensors 9 times before moving.
  // The median filter requires multiple samples to produce stable output,
  // so these dummy reads prime the filter's internal history buffers.
  for(int x = 0; x < 9; x++){
    ReadSensorsMedianFilter(&ahead, &right, &left);
  }
	
  while(1){
    // Sample all three IR sensors using a median filter to reject noise spikes.
    // The GP2Y0A21YK0F sensors are known to produce occasional erroneous readings,
    // so the median of the last three samples is used instead of raw ADC values.
    ReadSensorsMedianFilter(&ahead, &right, &left);

    // Pass filtered sensor readings to the steering controller
    steering(ahead, right, left);
  }
}

void System_Init(void) {
  PLL_Init();       // set bus clock to 80 MHz
  Sensors_Init();   // initialize ADC to sample AIN2 (PE1), AIN9 (PE4), AIN8 (PE5)
  LED_Init();       // configure onboard RGB LEDs and push buttons
  Car_Dir_Init();   // initialize GPIO pins that control motor direction
  Motors_Init();    // initialize hardware PWM signals for the two DC motors
}

// GPIO Port F interrupt handler -- fires on SW1 or SW2 button press.
// Controls the global 'start' flag to enable or disable the steering controller.
void GPIOPortF_Handler(void){
  if(GPIO_PORTF_RIS_R&SW2){     // SW2 pressed: stop the car
    GPIO_PORTF_ICR_R = SW2;     // clear the interrupt flag for SW2
    start = 0;                  // disable steering controller
  }
  if(GPIO_PORTF_RIS_R&SW1){     // SW1 pressed: start the car
    GPIO_PORTF_ICR_R = SW1;     // clear the interrupt flag for SW1
    start = 1;                  // enable steering controller
  }
}

// steering() -- main control logic for the robot.
// Called every loop iteration with fresh filtered sensor readings.
// Decides motor direction and duty cycle based on the car's current situation.
// Priority order (highest to lowest):
//   1. Lost/open space: spin to find walls
//   2. Completely boxed in: stop
//   3. Track curve right: arc right
//   4. Track curve left: arc left
//   5. Normal driving: proportional centering between two walls
void steering(uint16_t ahead_dist, uint16_t right_dist, uint16_t left_dist){
  if(start){  // only run if SW1 has been pressed to start
    bool pivot;

    // --- CASE 1: Open space detected (ahead sensor reads far) ---
    // If nothing is close in front, the car has likely lost the track
    // or entered an open area. Spin toward the closer wall to reorient.
    if(ahead_dist > IR20CM){
      diff = (left_dist - right_dist);
      // Determine which side is closer (higher ADC = closer object)
      if(diff < 0){
        R = true;   // right side is closer, spin right
      }else{
        R = false;  // left side is closer, spin left
      }
      // Set wheel directions for an in-place pivot
      if(!R){
        WHEEL_DIR = LEFTPIVOT;
      }else{
        WHEEL_DIR = RIGHTPIVOT;
      }
      PWM0_ENABLE_R |= 0x0C;          // enable both wheels
      Motors_Duty(FASTER, FASTER);    // spin at full speed to find a wall

    // --- CASE 2: Completely boxed in (all sensors detect close objects) ---
    // All three sensors are reading within IR80CM -- the car is cornered.
    // Stop both motors and indicate with blue LED.
    }else if(ahead_dist < IR80CM && left_dist < IR80CM && right_dist < IR80CM){
      PWM0_ENABLE_R &= ~0x0C;   // disable both wheels
      LED = Blue;

    // --- CASE 3: Track curves right (left wall opens up) ---
    // Ahead and right are blocked, but left is open -- this is a right-hand curve.
    // Drive only the left wheel to arc the car rightward.
    }else if(ahead_dist < IR30CM && left_dist > IR30CM && right_dist < IR30CM){
      WHEEL_DIR = FORWARD;
      PWM0_ENABLE_R &= ~0x00000008;   // disable right wheel
      PWM0_ENABLE_R |= 0x00000004;    // enable left wheel only
      Motors_Duty(FASTER, FASTER);

    // --- CASE 4: Track curves left (right wall opens up) ---
    // Ahead and left are blocked, but right is open -- this is a left-hand curve.
    // Drive only the right wheel to arc the car leftward.
    }else if(ahead_dist < IR30CM && left_dist < IR30CM && right_dist > IR30CM){
      WHEEL_DIR = FORWARD;
      PWM0_ENABLE_R |= 0x00000008;    // enable right wheel only
      PWM0_ENABLE_R &= ~0x00000004;   // disable left wheel
      Motors_Duty(FASTER, FASTER);

    // --- CASE 5: Normal driving -- proportional centering ---
    // Both walls are detected at reasonable distances.
    // Compute the difference between left and right sensor readings.
    // A larger difference means the car is off-center and needs correction.
    }else{
      diff = (left_dist - right_dist);
      // Determine which side is closer
      if(diff < 0){
        R = true;   // right side is closer, need to steer left
      }else{
        R = false;  // left side is closer, need to steer right
      }

      // Three-tier proportional correction based on how off-center the car is.
      // Small error: go straight. Medium error: gentle correction. Large error: aggressive correction.
      if(abs(diff) < 500){
        // Nearly centered -- drive straight at base speed
        left = START_SPEED;
        right = START_SPEED;
      }else if(abs(diff) < 1200){
        // Moderately off-center -- apply gentle correction (60% on the wider side)
        if(R){
          left = START_SPEED;
          right = TOTAL_PERIOD * 0.6;   // speed up right wheel to push car left
        }else{
          left = TOTAL_PERIOD * 0.6;    // speed up left wheel to push car right
          right = START_SPEED;
        }
      }else{
        // Significantly off-center -- apply aggressive correction (90% on the wider side)
        if(R){
          left = START_SPEED;
          right = TOTAL_PERIOD * 0.9;   // strong right correction
        }else{
          left = TOTAL_PERIOD * 0.9;    // strong left correction
          right = START_SPEED;
        }
      }

      Motors_Duty(left, right);
      WHEEL_DIR = FORWARD;
      PWM0_ENABLE_R |= 0x0000000C;   // enable both wheels
    }

    // --- LED status indicator ---
    // Updated after every steering decision to reflect current sensor state.
    if(ahead_dist > IR10CM || left_dist > IR10CM || right_dist > IR10CM){
      LED = Red;    // at least one sensor sees something far away (open space or wide track)
    }else if(ahead_dist < IR10CM && left_dist < IR10CM && right_dist < IR10CM){
      LED = Green;  // all sensors detect walls within 10cm -- tightly centered
    }else if(ahead_dist < IR80CM && left_dist < IR80CM && right_dist < IR80CM){
      LED = Blue;   // all walls detected but not extremely close -- normal driving range
    }

  }else{
    // Car is stopped (SW2 was pressed or never started)
    LED = Dark;             // turn off all LEDs
    PWM0_ENABLE_R &= ~0x0C; // disable both motors
  }
}

void Delay(void){
  unsigned long volatile time;
  time = 727240*500/91;   // approximately 0.25 seconds
  while(time){
    time--;
  }
}
