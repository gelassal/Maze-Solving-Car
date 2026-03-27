// LEDSW.h
// Runs on TM4C123 for CECS347 Project 2
#define LED (*((volatile unsigned long *)0x40025038))  // use onboard three LEDs: PF321
#define WHEEL_DIR (*((volatile unsigned long *)0x40005330)) // PB5432 are the four direction pins for L298

////////// Constants //////////  
// Color    LED(s) PortF
// dark     ---    0
// red      R--    0x02
// blue     --B    0x04
// green    -G-    0x08
// yellow   RG-    0x0A
// white    RGB    0x0E
// pink     R-B    0x06
// Cran     -GB    0x0C

#define Dark    	0x00
#define Red     	0x02
#define Blue    	0x04
#define Green   	0x08
#define Yellow  	0x0A
#define Cran      0x0C
#define White   	0x0E
#define Purple  	0x06

// Constant definitions based on the following hardware interface:
// PB5432 are used for direction control on L298.
// Motor 1 is connected to the left wheel, Motor 2 is connected to the right wheel.
#define FORWARD 0x88 //0x28 
#define BACKWARD 0x44 //0x14
#define LEFTPIVOT 0x48 //0x18
#define RIGHTPIVOT 0x84 //0x24

//////////////////////1. Declarations Section////////////////////////////////
////////// Function Prototypes //////////
// Dependency: None
// Inputs: None
// Outputs: None
// Description: Initializes PB5432 for use with L298N motor driver direction
void Car_Dir_Init(void);

void LED_Init(void);
