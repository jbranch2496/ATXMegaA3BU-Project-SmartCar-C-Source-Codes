/*
 * PHYS042_IRSense_test.c
 *
 * Created: 11/14/2019 10:13:43 AM
 * Author : Johnathan Branch
 */ 

// Libraries and included headers
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <util/delay.h>
#include <AVRXlib/AVRXClocks.h>


// Global Variable Declarations
uint16_t IR_input = 0; // input from SW0
uint8_t begin_read_flag = 0; // flag for read function debug
double counter = 0; // loop counter used in the read function
double counter_max = 100000; //max value loop counter variable will count to;play with this value should be smaller than timer period
uint8_t state = 0; // will hold the current state of the FST(either 0 for invalid, 1, 2, 3, 4 , 5 for open)
uint8_t state_flag = 1; // flag for the StateLogic function
uint8_t mode = 0; // used to change the mode of the switch case used in the StateLogic function
uint8_t led_flash_counter = 0; // used to change the number of times LED0 flashes

//Global Volatiles

//State Definitions

/*** Function Prototypes ***/
//void Timer0_init();
void ReadIR();
//void StateLogic();
//void LED0Flash();
//void LED0Control();

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
	TCC0_PER = 0x1E85; // Overflows at 2500 ticks of (2 MHZ/8)^-1 timer clock ticks  
	TCC0_CTRLA |= 0x04; // Prescaler of 8
	TCC0_CTRLB |= 0x13; // CCAEN and Single Slope mode 
}

// Function to read the input from IR sensor
void ReadIR()
{
	begin_read_flag = 1; // set high when Read function is initially called
	
	// Do-while loop captures the switch values
	do{
		IR_input = (PORTA_IN & 0x01); // stores the input coming from PB5
		counter++; // increment the counter
	}while(counter < counter_max);
	counter = 0; // resets the counter after the loop
	begin_read_flag = 0; // set low at end of Read function
}

int main(void)
{
	PORTR_DIR |= 0x03; // enable LED PORTs as outputs
	PORTR_OUT |= 0x03; // Turn off LEDs to start with if on
	PORTA_DIR &= ~(0x01); // enable PORTB5(IR sensor) as input
	
	pwm_init();
	TCC0_CCA = 0x00C8; // Sets up the CCA register which for PWM SS mode is the duty cycle -> .08*TCC0_PER = 200 ticks of timer
	PORTR_DIR |= 0x03; // Enable on board LED as output for checking ISR firing
	
	while( (TCC0_INTFLAGS & 0x01) == 0); // waits for the new compare value to be loaded
	TCC0_INTFLAGS = 0x00; // clear the interrupt flag
	
	while(1)
		{
			
			ReadIR(); // calls Read IR function
			
			if(IR_input == 0x0000)
			{
				PORTR_OUT |= 0x03; // turn LEDs off
			}
			else
			{
				PORTR_OUT &= ~(0x03); // turn LEDs on 
			}
			_delay_ms(2000);	// short delay of 2 sec
		}
	
	
}









