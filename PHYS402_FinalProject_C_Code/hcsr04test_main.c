/*
 * PHYS402_HCSR04_test1.c
 *
 * Created: 11/23/2019 6:53:37 AM
 * Author : Johnathan Branch
  * HC-SR04 Proximity Sensor Test Code( pin out goes as follows; Vcc, Trig, Echo, Gnd
  * Purpose: This code illustrates an example of how to interface the HC-SR04 proximity sensor with the ATXMega256 A3BU.
  * Obj: Use the Ultrasonic sensor to measure the distance between close objects(range 2cm ~ 40cm). This doesn't have to be exact I just 
  * need to find a feel for how well the sensor works at the distance I'm concerned with which is about 8-10inches(about 20 ~25 cm) from the sensor.
  * Instructions: To start a measurement the Trig pin must be set HIGH for at least 10us(I'll make it 15us), this will initiate the transmitter to sens out 8 cycle 
  * burst of ultrasonic signals at 40 kHz and pull the Echo pin HIGH. Once a reflected signal has been sensed on the receiver the echo pin is pulled LOW. 
  * The period of delay time between HIGH and LOW on the Echo pin is what is captured and is proportional to the distance of the sensed object. The formula given
  * is (Time in cm = delay time / 58 or Time in inches = Time / 148). I will have to adjust this on how well the sensor works.(Hopefully it works !)

  * Note: 3.3V signal logic Level used but power is +5V from ground. Echo is on PORTC pin 2 and Trig is on PORTC pin 3.
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include <util/delay.h>

// Global Variable Declarations
//float D = 0, I = 0, B = 0, X = 0, Y = 0, Z = 0;													// fuzzy state variables


// Global Volatiles
volatile int timer_count = 0;
volatile int echo_count = 0;
volatile int echo_read = 0;
volatile int flag = 0;

// State Definitions


/*** Function Prototypes ***/

ISR(PORTC_INT0_vect)
{
	echo_count = 0;
	do 
	{
		echo_count++;
	} while (PORTC_IN & 0x04);
	echo_read = echo_count;
	PORTR_OUT ^= 0x01;
}

ISR(TCF0_OVF_vect)
{
	timer_count++;
	flag = 1;
}

int main(void)
{
	
	// Port Direction Setup
	PORTC_DIR |= 0x08;													// Set PC3(0x08) as an output 
	PORTC_DIR &= ~(0x04);												// Set PC2(0x04) as input
	PORTR_DIR |= 0x03;													// enable on board LEDs
	PORTR_OUT |= 0x03;													// turn off the LEDs if on
	
	// PORTC_INT0 And TCD0 Interrupt Setup
	cli();																// disables interrupt
	PORTC_INT0MASK = 0x04;												// only want PC2 to fire interrupt
	PORTC_PIN2CTRL = 01;												// rising edge triggered on pin 2
	PORTC_INTCTRL = PMIC_MEDLVLEN_bm;									// generate an interrupt when the timer overflows
	
	TCF0_PER = 0x7530;													// set the range of the counter(50000 ticks of prescaled clock)
	TCF0_INTCTRLA = PMIC_LOLVLEN_bm;									// generate an interrupt when the timer overflows
	TCF0_CTRLA |= 0x01;													// set the timer prescaler(div by 8)
	PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;	// turn on all interrupt levels
	sei();																// enables interrupts
  
  
    /* Replace with your application code */
    while (1) 
    {
		if(timer_count > 100)
		{
			timer_count = 0;
			PORTR_OUT ^= 0x02;
			PORTC_OUT |= 0x08;		    // turn on
			PORTC_OUT |= 0x08;		    // turn on
			PORTC_OUT |= 0x08;		    // turn on
//	_delay_us(1);			    // fifteen us delay
			PORTC_OUT &= ~(0x08);		// turn off
		}
    }
}


