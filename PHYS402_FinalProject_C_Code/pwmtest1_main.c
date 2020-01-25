/*
 * PHYS402_PWMtest1.c
 *Purpose: This code illustrates how to set up PWM on the 
 ATXMega 256A3BU using the compare functionality of the 
 target board. MCU clock needs to be set to 2 MHZ for code 
 to run properly. PWM will be run off of Pin PC0
 * Created: 10/15/2019 11:21:39 AM
* Author : Johnathan Branch 
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
//#include <AVRXlib/AVRXClocks.h>
//#include <AVRXlib/AVRXSerial.h>

// timer overflow ISR
/*
ISR(TCC0_OVF_vect)
{
	PORTR_OUT &= ~(0x01); // toggles LED every time interrupt trigger
}

ISR(TCC0_CCA_vect)
{
	PORTR_OUT &= ~(0x02); // toggles LED every time interrupt trigger;
}
*/
void pwm_init()
{
// This sets up the timer and instead of using an interrupt for generating
// a PWM waveform, the capture/compare register(CCA), cc enable(CCAEN), and WGMODE registers
// are used to generate the PWM using mostly the internal hardware.

	//cli();
	PORTC_DIR |= 0x01; // Enable PA0 as output for CCA to trigger based on PWM mode setting
	TCC0_PER = 0x1388; // Overflow of the PWM
	TCC0_CTRLA |= 0x04; // Prescaler of 8
	TCC0_CTRLB |= 0x13; // CCAEN and Single Slope mode 
}

int main(void)
{
	PORTA_DIR = 0x0F; // Initialize PORTA for the DC Motors; PA0 -> IN1, PA1 -> IN2, PA2 -> IN3, PA3 ->IN4 
	PORTA_OUT |= 0x01;
	pwm_init();
	
	TCC0_CCA = 0x01F4; // Sets up the CCA register which for PWM SS mode is the duty cycle -> .08*TCC0_PER = 200 ticks of timer
	PORTR_DIR |= 0x03; // Enable on board LED as output for checking ISR firing
	
	while( (TCC0_INTFLAGS & 0x01) == 0); // waits for the new compare value to be loaded
	TCC0_INTFLAGS = 0x00; // clear the interrupt flag

	while(1){;} // run forever

}


