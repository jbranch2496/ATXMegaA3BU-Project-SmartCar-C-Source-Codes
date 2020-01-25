/*
 * PHYS402_HCSR04_test1.c
 *
 * Created: 11/23/2019 6:53:37 AM
 * Author : Johnathan Branch
  * Class: PHYS 402, Fall 2019
  * HC-SR04 Proximity Sensor Test Code( pin out goes as follows; Vcc, Trig, Echo, Gnd
  * Credit:Yes
  * Purpose: This code illustrates an example of how to interface the HC-SR04 proximity sensor with the ATXMega256 A3BU.
  * Obj: Use the Ultrasonic sensor to measure the distance between close objects(range 2cm ~ 40cm). This doesn't have to be exact I just 
  * need to find a feel for how well the sensor works at the distance I'm concerned with which is about 8-10inches(about 20~25 cm) from the sensor.
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


// State Definitions


/*** Function Prototypes ***/



int main(void)
{
	
	// Port Direction Setup
	PORTC_DIR |= 0x08; // Set PC3(0x08) as an output 
	
    /* Replace with your application code */
    while (1) 
    {
		PORTC_OUT &= (0x08); // turn off
		_delay_us(5);		 // five microsecond delay to make sure pin output is low
		PORTC_OUT |= 0x08;	 // turn on
		_delay_us(15);		 // fifteen microsecond delay to give a good trigger
		PORTC_OUT &= (0x08); // turn off
    }
}

