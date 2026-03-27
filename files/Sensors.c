// Sensors.c
// Runs on TM4C123
// Provide functions that sample ADC inputs PE1, PE4, PE5
// using SS2 to be triggered by software and trigger three
// conversions,wait for them to finish, and return the three
// results.
// PE1, PE4, PE5 are connected to a sensor ahead, right and left, respectively.
// Starter code is by Dr.Min He on 3/14/2023.

#include "tm4c123gh6pm.h"
#include "Sensors.h"
#include "stdint.h"

// There are many choices to make when using the ADC, and many
// different combinations of settings will all do basically the
// same thing.  For simplicity, this function makes some choices
// for you.  When calling this function, be sure that it does
// not conflict with any other software that may be running on
// the microcontroller.  Particularly, ADC0 sample sequencer 2
// is used here because it takes up to four samples, and three
// samples are needed.  Sample sequencer 2 generates a raw
// interrupt when the third conversion is complete, but it is
// not promoted to a controller interrupt.  Software triggers
// the ADC0 conversion and waits for the conversion to finish.
// If somewhat precise periodic measurements are required, the
// software trigger can occur in a periodic interrupt.  This
// approach has the advantage of being simple.  However, it does
// not guarantee real-time.
//
// A better approach would be to use a hardware timer to trigger
// the ADC conversion independently from software and generate
// an interrupt when the conversion is finished.  Then, the
// software can transfer the conversion result to memory and
// process it after all measurements are complete.

// Initializes AIN2, AIN9, and AIN8 sampling three sensors: left, center, right
// 125k max sampling
// SS2 triggering event: software trigger, busy-wait sampling
// SS2 1st sample source: AIN2 (PE1)
// SS2 2nd sample source: AIN9 (PE4)
// SS2 3rd sample source: AIN8 (PE5)
// SS2 interrupts: enabled after 3rd sample but not promoted to controller
void Sensors_Init(void){
	  volatile unsigned long delay;
//  SYSCTL_RCGC0_R |= 0x00010000; // 1) activate ADC0 (legacy code)
  SYSCTL_RCGCADC_R |= 0x00000001; // 1) activate ADC0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4; // 1) activate clock for Port E
  delay = SYSCTL_RCGCGPIO_R;      // 2) allow time for clock to stabilize
  delay = SYSCTL_RCGCGPIO_R;
  GPIO_PORTE_DIR_R &= ~0x32;      // 3) make PE1, PE4, and PE5 input
  GPIO_PORTE_AFSEL_R |= 0x32;     // 4) enable alternate function on PE1, PE4, and PE5
  GPIO_PORTE_DEN_R &= ~0x32;      // 5) disable digital I/O on PE1, PE4, and PE5
                                  // 5a) configure PE4 as ?? (skip this line because PCTL is for digital only)
  GPIO_PORTE_PCTL_R = GPIO_PORTE_PCTL_R&0xFF00FF0F;
  GPIO_PORTE_AMSEL_R |= 0x32;     // 6) enable analog functionality on PE1, PE4, and PE5
  ADC0_PC_R &= ~0xF;              // 8) clear max sample rate field
  ADC0_PC_R |= 0x1;               //    configure for 125K samples/sec
  ADC0_SSPRI_R = 0x3210;          // 9) Sequencer 3 is lowest priority
  ADC0_ACTSS_R &= ~0x0004;        // 10) disable sample sequencer 2
  ADC0_EMUX_R &= ~0x0F00;         // 11) seq2 is software trigger
  ADC0_SSMUX2_R = 0x0892;         // 12) set channels for SS2
  ADC0_SSCTL2_R = 0x0600;         // 13) no D0 END0 IE0 TS0 D1 END1 IE1 TS1 D2 TS2, yes END2 IE2
  ADC0_IM_R &= ~0x0004;           // 14) disable SS2 interrupts
  ADC0_ACTSS_R |= 0x0004;         // 15) enable sample sequencer 2
}

//------------ADC_In298------------
// Busy-wait Analog to digital conversion
// Input: none
// Output: three 12-bit result of ADC conversions
// Samples AIN8, AIN9, and AIN2 for left, center, right sensors
// 125k max sampling
// software trigger, busy-wait sampling
// data returned by reference
// ain2 (PE1) 0 to 4095
// ain9 (PE4) 0 to 4095
// ain8 (PE5) 0 to 4095
void Sensors_In(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8){
	 ADC0_PSSI_R = 0x0004;            // 1) initiate SS2
  while((ADC0_RIS_R&0x04)==0){};   // 2) wait for conversion done
  *ain2 = ADC0_SSFIFO2_R&0xFFF;    // 3A) read first result
  *ain9 = ADC0_SSFIFO2_R&0xFFF;    // 3B) read second result
  *ain8 = ADC0_SSFIFO2_R&0xFFF;    // 3C) read third result
  ADC0_ISC_R = 0x0004;             // 4) acknowledge completion
}

// Returns the results in the corresponding variables.  Some
// kind of filtering is required because the IR distance sensors
// output occasional erroneous spikes.  This is an FIR filter:
// y(n) = (x(n) + x(n-1))/2
// Assumes: ADC initialized by previously calling Sensors_Init()
void ReadSensorsFIRFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8){
  static uint16_t ain2previous=0; // after the first call, the value changed to 12
  static uint16_t ain9previous=0;
  static uint16_t ain8previous=0;
  // save some memory; these do not need to be 'static'
  //            x(n)
  uint16_t ain2newest;
  uint16_t ain9newest;
  uint16_t ain8newest;
  Sensors_In(&ain2newest, &ain9newest, &ain8newest); // sample AIN2(PE1), AIN9 (PE4), AIN8 (PE5)
  *ain2 = (ain2newest + ain2previous)/2;
  *ain9 = (ain9newest + ain9previous)/2;
  *ain8 = (ain8newest + ain8previous)/2;
  ain2previous = ain2newest; ain9previous = ain9newest; ain8previous = ain8newest;
}

// This function samples AIN2 (PE1), AIN9 (PE4), AIN8 (PE5) and
// returns the results in the corresponding variables.  Some
// kind of filtering is required because the IR distance sensors
// output occasional erroneous spikes.  This is an IIR filter:
// y(n) = (x(n) + y(n-1))/2
// Assumes: ADC initialized by previously calling Sensors_Init()
void ReadSensorsIIRFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8){
  //                   y(n-1)
  static uint16_t filter2previous=0;
  static uint16_t filter9previous=0;
  static uint16_t filter8previous=0;
  // save some memory; these do not need to be 'static'
  //            x(n)
  uint16_t ain2newest;
  uint16_t ain9newest;
  uint16_t ain8newest;
  Sensors_In(&ain2newest, &ain9newest, &ain8newest); // sample AIN2(PE1), AIN9 (PE4), AIN8 (PE5)
  *ain2 = filter2previous = (ain2newest + filter2previous)/2;
  *ain9 = filter9previous = (ain9newest + filter9previous)/2;
  *ain8 = filter8previous = (ain8newest + filter8previous)/2;
}

// Median function from EE345M Lab 7 2011; Program 5.1 from Volume 3
// helper function for ReadADCMedianFilter() but works for general use
uint16_t median(uint16_t u1, uint16_t u2, uint16_t u3){
uint16_t result;
  if(u1>u2)
    if(u2>u3)   result=u2;     // u1>u2,u2>u3       u1>u2>u3
      else
        if(u1>u3) result=u3;   // u1>u2,u3>u2,u1>u3 u1>u3>u2
        else      result=u1;   // u1>u2,u3>u2,u3>u1 u3>u1>u2
  else
    if(u3>u2)   result=u2;     // u2>u1,u3>u2       u3>u2>u1
      else
        if(u1>u3) result=u1;   // u2>u1,u2>u3,u1>u3 u2>u1>u3
        else      result=u3;   // u2>u1,u2>u3,u3>u1 u2>u3>u1
  return(result);
}

// This function samples AIN2 (PE1), AIN9 (PE4), AIN8 (PE5) and
// returns the results in the corresponding variables.  Some
// kind of filtering is required because the IR distance sensors
// output occasional erroneous spikes.  This is a median filter:
// y(n) = median(x(n), x(n-1), x(n-2))
// Assumes: ADC initialized by previously calling Sensors_Init()
void ReadSensorsMedianFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8){
  //                   x(n-2)        x(n-1)
  static uint16_t ain2oldest=0, ain2middle=0;
  static uint16_t ain9oldest=0, ain9middle=0;
  static uint16_t ain8oldest=0, ain8middle=0;
  // save some memory; these do not need to be 'static'
  //            x(n)
  uint16_t ain2newest;
  uint16_t ain9newest;
  uint16_t ain8newest;
  Sensors_In(&ain2newest, &ain9newest, &ain8newest); // sample AIN2(PE1), AIN9 (PE4), AIN8 (PE5)
  *ain2 = median(ain2newest, ain2middle, ain2oldest);
  *ain9 = median(ain9newest, ain9middle, ain9oldest);
  *ain8 = median(ain8newest, ain8middle, ain8oldest);
  ain2oldest = ain2middle; ain9oldest = ain9middle; ain8oldest = ain8middle;
  ain2middle = ain2newest; ain9middle = ain9newest; ain8middle = ain8newest;
}
