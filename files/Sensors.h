// Sensors.h
// Runs on TM4C123
// Provide functions that sample ADC inputs PE1, PE4, PE5
// using SS2 to be triggered by software and trigger three
// conversions,wait for them to finish, and return the three
// results.
// PE1, PE4, PE5 are connected to a sensor ahead, right and left, respectively.
// Starter code is by Dr.Min He on 3/14/2023.

#include "stdint.h"

// Initializes AIN2, AIN9, and AIN8 sampling three sensors: ahead, right, left
// 125k max sampling
// SS2 triggering event: software trigger, busy-wait sampling
// SS2 1st sample source: AIN2 (PE1)
// SS2 2nd sample source: AIN9 (PE4)
// SS2 3rd sample source: AIN8 (PE5)
// SS2 interrupts: enabled after 3rd sample but not promoted to controller
void Sensors_Init(void);

//------------ADC_In298------------
// Busy-wait Analog to digital conversion
// Input: none
// Output: three 12-bit result of ADC conversions
// Samples AIN8, AIN9, and AIN2
// 125k max sampling
// software trigger, busy-wait sampling
// data returned by reference
// ain2 (PE1) 0 to 4095
// ain9 (PE4) 0 to 4095
// ain8 (PE5) 0 to 4095
void Sensors_In(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8);

// Three software filter methods
void ReadSensorsFIRFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8);
void ReadSensorsIIRFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8);
void ReadSensorsMedianFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8);// This function samples AIN2 (PE1), AIN9 (PE4), AIN8 (PE5) and

