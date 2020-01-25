/*
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

#define PER 0x01388 // 5000
#define M1_PWM 0x0FA0 // 4000 -> D = 80
#define M2_PWM 0x0FA0 // 4000 -> D = 80

volatile int CCA_flag = 0; 
volatile int CCA_OVF_flag = 0;

uint16_t pin_input = 0;


//#include <AVRXlib/AVRXClocks.h>
//#include <AVRXlib/AVRXSerial.h>

// timer overflow ISR

ISR(TCC1_OVF_vect)
{
	PORTC_OUT |= 0x02; // PWM on
	PORTR_OUT ^= 0x01;
	CCA_OVF_flag = 1;
}

ISR(TCC1_CCA_vect)
{
	PORTC_OUT &= ~(0x02); // PWM off
	PORTR_OUT ^= 0x02;
	CCA_flag = 1;
}

void pwm_init()
{
// This sets up the timer and instead of using an interrupt for generating
// a PWM waveform, the capture/compare register(CCA), cc enable(CCAEN), and WGMODE registers
// are used to generate the PWM using mostly the internal hardware.

	cli();
	PORTC_DIR |= 0x01; // Enable PC0 as output for CCA to trigger based on PWM mode setting
	TCC0_PER = PER; // Overflow of the PWM
	TCC0_CTRLA |= 0x04; // Prescaler of 8 
	TCC0_CTRLB |= 0x13; // CCAEN and Single Slope mode 
	
	PORTC_DIR |= 0x02; // Enable PC1 as output for CCA to trigger based on PWM mode setting
	TCC1_PER = PER; // Overflow of the PWM
	TCC1_INTCTRLA = PMIC_MEDLVLEN_bm;
	TCC1_CTRLA |= 0x04; // Prescaler of 8
	TCC1_INTCTRLB = TC_CCAINTLVL_MED_gc | TC_WGMODE_DSTOP_gc; 
	//TCC1_CTRLB |= 0x13; // CCAEN and Single Slope mode
	PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;				// turn on all interrupt levels
	
	sei();
}

int main(void)
{
	// DC Motor 1 on PA0 and PA1
	// DC Motor 2 on PA2 and PA3
	PORTA_DIR = 0x0F; // Initialize lower nib of PORTA as output for the DC Motors; PA0 -> IN1, PA1 -> IN2, PA2 -> IN3, PA3 ->IN4 
	PORTB_DIR &= ~(0x01); // Initialize PORTB0 as input for reading IR sensor
	PORTA_OUT |= 0x0F;
	
	pwm_init();
	
	TCC0_CCA = M1_PWM; // Sets up the CCA register which for PWM SS mode is the duty cycle
	TCC1_CCA = M2_PWM; // Sets up the CCA register which for PWM SS mode is the duty cycle  
	PORTR_DIR |= 0x03; // Enable on board LED as output for checking ISR firing
	
	while( (TCC0_INTFLAGS & 0x01) == 0); // waits for the new compare value to be loaded
	TCC0_INTFLAGS = 0x00; // clear the interrupt flag

	while(1){
	
	PORTA_OUT = 0x01;
	/*	
		//PORTB_IN |= 0x01;
		pin_input = PORTB_IN & 0x03;
		
		if( pin_input == 0x0001)
			{
				PORTA_OUT = 0x00;
			}
		else
		{
			PORTA_OUT = 0x01;
		}
		} // run forever
	*/
}

}


