/*
 * PHY402_FinalProject.c
 * Created: 12/05/2019 11:00:37 AM
 * Author : Johnathan Branch
 * Purpose: This code is the application code for controlling the smart robotic vehicle. This code illustrates how to interface the HC-SR04 proximity sensor, 
 * L298 H-bridge driver, and optical detection networks(load and motor control) with the ATXMega256 A3BU.
 
 * Obj: This code integrates all the sensors and external circuit networks to control the behavior of the vehicle. The behavior of the vehicle is defined as follows,
		the ATXMega will poll the various sensors through the main program loop and if the input from the sensors is appropriate the motor configuration function will
		control the output sent from the A3BU to the motor drivers controlling the motion of the vehicle. For the control logic there are three functions; Dist_Detect(), 
		Load_Detect(), and IR_Read(). If the distance received from the HC-SR04 is outside of a defined range and not null then the distance function will set a flag HIGH, 
		otherwise the flag is set low. In the load read function if the load detection senses a load(PB2 input is LOW) then a flag is set HIGH, otherwise the flag is set LOW. 
		In the IR read function logic is set in the IR read function based on what the input of the two IR sensors read when polled.
		Once the flags have been set and the control of the motors have been determined a function called Config_Set(), will set the
		port values connected to the two motor drivers. (Hopefully it works !)
 
 * Abbreviations: FML(front motor left), FMR(front motor right), BML(back motor left), BMR(back motor right)
 the DC motors have the following associations on the two 293 motor drivers; FML(IN1,IN2), FMR(IN3,IN4), BML(IN3,IN4), and BMR(IN1, IN2) 		

 * Pin Mappings: IR-left: PB1(0x02)
			   	 IR-right: PB0(0x01)
				 Load: PB2(0x04)
				 Trig: PC3(0x08)
				 Echo: PC2(0x04)
				 FML_IN1: PA0(0x01)
				 FML_IN2: PA1(0x02)
				 FMR_IN3: PA2(0x04)
				 FMR_IN4: PA3(0x08)
				 BML_IN3: PA6(0x40)
				 BML_IN4: PA7(0x80)
				 BMR_IN1: PA4(0x10)
				 BMR_IN2: PA5(0x20)
				 FML_PWM: PC0(0x01)
				 FMR_PWM: PC1(0x02)
				 BML_PWM: PC7(0x80)
				 BMR_PWM: PC6(0x40)
				 		
	* Commits: Car does not turn well, need to be able to to decide more accurately when to turn and when to stop turning and go back to driving.	 
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

// Hex Definitions
#define PWM_PER 0x09C4				// 2500 PWM timer period for approx 10 ms period with CLK/8
#define FML_PWM 0x02EE				// 750 -> D = 30 , increase D to 30 for turning 
#define FMR_PWM 0x02EE				// 750 -> D = 30 , increase D to 30 for turning
#define BMR_PWM 0x02D5				// 725 -> D = 29, increase D to 35 for turning 
#define BML_PWM 0x02D5				// 725 -> D = 299, increase D to 35 for turning 
#define TURN_BM_PWM 0x07D0			// 1600 -> D = 64 of back motors for turning 
#define TURN_FM_PWM 0x0640			// 1500 -> D = 60 on front motors for turning

#define ALL_FWD  0x01 | 0x04 | 0x20 | 0x80
#define REAR_FWD 0x20 | 0x80
#define FRONT_FWD 0x01 | 0x04
#define LEFT_ONLY 0x01 | 0x80
#define RIGHT_ONLY 0x04 | 0x20
#define ALL_OFF 0x00

// Global Variables
volatile int ir_left = 0;
volatile int ir_right = 0;
volatile int load_detect = 0;
volatile double ir_counter = 0;			// loop counter used in the ir read function
volatile double load_counter = 0;       	// loop counter used in the load_detect function
const double counter_max = 10;			// max value loop counter variable will count to;play with this value should be smaller than timer period
volatile uint8_t car_direction = 0;		// 0-> stop, 1-> forward, 2-> turn left, 3 -> turn right  
volatile int echo_count = 0;
volatile int echo_read = 0;
volatile int flag = 0;
volatile int timer_count = 0;
/*
int turn_latch_left = 0;				// turn latches on the left motors to make sure the car turns correctly
int turn_latch_right = 0;				// turn latches on the right motors to make sure car turns correctly 
int turn_count_left = 0;				// left turn latch counter to balance how long to take turns
int turn_count_right = 0;				// right turn latch counter to balance how long to take turns
*/

// timer overflow ISRs

// PWM ISRs ************************//
ISR(TCC1_OVF_vect)					// FMR
{
	PORTC_OUT |= 0x02;				// PWM on
	PORTR_OUT ^= 0x01;
}
ISR(TCC1_CCA_vect)
{
	PORTC_OUT &= ~(0x02);				// PWM off
	PORTR_OUT ^= 0x02;
}


ISR(TCE0_OVF_vect)					// BML 
{
	PORTC_OUT |= 0x80;				// PWM on
	PORTR_OUT ^= 0x01;
}
ISR(TCE0_CCA_vect)			 
{
	PORTC_OUT &= ~(0x80);				// PWM off
	PORTR_OUT ^= 0x02;
}


ISR(TCE1_OVF_vect)					// BMR
{
	PORTC_OUT |= 0x40;				// PWM on
	PORTR_OUT ^= 0x01;
}
ISR(TCE1_CCA_vect)
{
	PORTC_OUT &= ~(0x40);				// PWM off
	PORTR_OUT ^= 0x02;
}
//**********************************//
// HCSR-04 ISRs *******************//
ISR(TCD1_OVF_vect)
{
			flag = 1;
			timer_count = (timer_count+1)%101;
			
			if(timer_count > 99)
			{
			timer_count = 0;	
			PORTC_OUT |= 0x08;		    // turn on
			PORTC_OUT |= 0x08;		    // turn on repeated to get timing right 
			PORTC_OUT |= 0x08;		    // turn on repeated to get timing right
			PORTC_OUT |= 0x08;		    // turn on repeated to get timing right
			//	_delay_us(1);		    // fifteen us delay
			PORTC_OUT &= ~(0x08);	 	    // turn off, total pulse should last about 15us
			}
}
ISR(PORTC_INT0_vect)
{
	echo_count = 0;
	do
	{
		echo_count++;
	} while (PORTC_IN & 0x04);
	echo_read = echo_count;
}
//**********************************//
// Sensory Functions ***************//
void IR_Read()
{
		// Do-while loop captures the IR sensor readings
		do{
			ir_right = (PORTB_IN & 0x01); // stores the input coming from PB0
			ir_left = (PORTB_IN & 0x02);  // stores the input coming from PB1
			ir_counter++;		      // increment the counter
		}while(ir_counter < counter_max);
		ir_counter = 0;		   	      // resets the counter after the loop
		
		if( (ir_left == 0) && (ir_right == 0) )						// Both sensors over the white
			car_direction = 1;
	
		if( (ir_left == 0) && (ir_right == 1) )						// Only right sensor over the black(need to turn right)
			{
			 car_direction = 2;
			}
		if( (ir_left == 2) && (ir_right == 0) )						// Only left sensor over the black(need to turn left)
			{
			car_direction = 3;
			}
		if( (ir_left == 2) && (ir_right == 1) )						// Both sensors over the black
			car_direction = 0;
					
}

void Load_Detect()
{
		// Do-while loop captures the load detection readings  
		do{
			load_detect = (PORTB_IN & 0x04);	// stores the input coming from PB0
			load_counter++;				// increment the counter
		}while(load_counter < counter_max);
		load_counter = 0;					// resets the counter after the loop
		
		if(load_detect != 0x04)					// if the reading is LOW on PB2 then there is NOT a Load 
			car_direction = 0;				// car ignores previous IR sensor configuration for the motors if there is not a Load 					
			
}

void Dist_Detect()
{
	if(echo_read < 0x0090)						// this corresponds to a hand distance of approx. 8 inches w/hand lined up with sensor
		car_direction = 0;	
}
//**********************************//
// Motor Confuguration *************//
void Config_Set()
{
	if(car_direction == 0)
		PORTA_OUT = ALL_OFF;
	else if(car_direction == 1)
	{
		TCC0_CCA = FML_PWM;						 // Sets up the CCA register which for PWM SS mode is the duty cycle
		TCC1_CCA = FMR_PWM;						 // Sets up the CCA register which for PWM SS mode is the duty cycle
		TCE0_CCA = BML_PWM;						 // Sets up the CCA register which for PWM SS mode is the duty cycle
		TCE1_CCA = BMR_PWM;						 // Sets up the CCA register which for PWM SS mode is the duty cycle
		PORTA_OUT = ALL_FWD;
	}
	else if(car_direction == 2)											// Turns the car to the left 
	{
		TCC0_CCA = TURN_FM_PWM;						 // Sets up the CCA register which for PWM SS mode is the duty cycle
		TCC1_CCA = TURN_FM_PWM;						 // Sets up the CCA register which for PWM SS mode is the duty cycle
		TCE0_CCA = TURN_BM_PWM;						 // Sets up the CCA register which for PWM SS mode is the duty cycle
		TCE1_CCA = TURN_BM_PWM;						 // Sets up the CCA register which for PWM SS mode is the duty cycle
		PORTA_OUT = LEFT_ONLY;
	}
	else if(car_direction == 3)											// Turns the car to the right
	{
		TCC0_CCA = TURN_FM_PWM;						 // Sets up the CCA register which for PWM SS mode is the duty cycle
		TCC1_CCA = TURN_FM_PWM;						 // Sets up the CCA register which for PWM SS mode is the duty cycle
		TCE0_CCA = TURN_BM_PWM;						 // Sets up the CCA register which for PWM SS mode is the duty cycle
		TCE1_CCA = TURN_BM_PWM;						 // Sets up the CCA register which for PWM SS mode is the duty cycle 
		PORTA_OUT = RIGHT_ONLY;
	}
}

void pwm_init()
{
																		// This sets up the timer and instead of using an interrupt for generating
																		// a PWM waveform, the capture/compare register(CCA), cc enable(CCAEN), and WGMODE registers
																		// are used to generate the PWM using mostly the internal hardware.

	PORTC_DIR |= 0x01;													// Enable PC0 as output for CCA to trigger based on PWM mode setting
	TCC0_PER = PWM_PER;													// Overflow of the PWM
	TCC0_CTRLA |= 0x04;													// Prescaler of 8 
	TCC0_CTRLB |= 0x13;													// CCAEN and Single Slope mode 
	
																		// The other PWM are created using interrupts
	
	PORTC_DIR |= 0x02;													// Enable PC1 as output for CCA to trigger based on PWM mode setting
	TCC1_PER = PWM_PER;													// Overflow of the PWM
	TCC1_INTCTRLA = PMIC_MEDLVLEN_bm;
	TCC1_CTRLA |= 0x04;													// Prescaler of 8
	TCC1_INTCTRLB = TC_CCAINTLVL_MED_gc | TC_WGMODE_DSTOP_gc; 
	
	
	PORTC_DIR |= 0x80;													// Enable PC7 as output for CCA to trigger based on PWM mode setting
	TCE0_PER = PWM_PER;													// Overflow of the PWM
	TCE0_INTCTRLA = PMIC_MEDLVLEN_bm;
	TCE0_CTRLA |= 0x04;													// Prescaler of 8
	TCE0_INTCTRLB = TC_CCAINTLVL_MED_gc | TC_WGMODE_DSTOP_gc;
	
		
	PORTC_DIR |= 0x40;													// Enable PC6 as output for CCA to trigger based on PWM mode setting
	TCE1_PER = PWM_PER;													// Overflow of the PWM
	TCE1_INTCTRLA = PMIC_MEDLVLEN_bm;
	TCE1_CTRLA |= 0x04;													// Prescaler of 8
	TCE1_INTCTRLB = TC_CCAINTLVL_MED_gc | TC_WGMODE_DSTOP_gc;
	//PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;	// turn on all interrupt levels
	
}

void HCSR04_init()
{
	PORTC_INT0MASK = 0x04;											// only want PC2 to fire interrupt
	PORTC_PIN2CTRL = 01;											// rising edge triggered on pin 2
	PORTC_INTCTRL = PMIC_MEDLVLEN_bm;									// generate an interrupt on rising edge of PC2 
		
	TCD1_PER = 0x7530;											// set the range of the counter(30000 ticks of prescaled clock)
	TCD1_INTCTRLA = PMIC_MEDLVLEN_bm;									// generate an interrupt when the timer overflows
	TCD1_CTRLA |= 0x01;											// set the timer prescaler(div by 1)	
	PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;	// turn on all interrupt levels
}
//**********************************//
// Main Function *******************//
int main(void)
{
											// FML on PA0 and PA1
											// FMR on PA2 and PA3
											// BML on PB6 and PB7
											// BMR on PB4 and PB5
	
	PORTA_DIR |= 0xFF;					// Initialize lower and upper nib of PORTA as outputs for the DC Motors
	PORTB_DIR &= ~(0x07);					// Initialize PB0 and PB1 as inputs for reading the IR sensors and PB2 as input for reading load detection network
	PORTC_DIR |= 0x08;					// Initialize PC3 as output for Trig for distance reading
	PORTC_DIR &= ~(0x04);					// Initialize PC2 and input for Echo for distance reading
	
	cli();								// Disables all global interrupts
	pwm_init();							// Initialize Timers for the PWM channels
	HCSR04_init();							// Initialize Timer for the HCSR-04 proximity sensor
	sei();								// Enables all global interrupts
	
	TCC0_CCA = FML_PWM;						 // Sets up the CCA register which for PWM SS mode is the duty cycle
	TCC1_CCA = FMR_PWM;						 // Sets up the CCA register which for PWM SS mode is the duty cycle  
	TCE0_CCA = BML_PWM;						 // Sets up the CCA register which for PWM SS mode is the duty cycle  
	TCE1_CCA = BMR_PWM;						 // Sets up the CCA register which for PWM SS mode is the duty cycle  
	
	PORTR_DIR |= 0x03;						 // Enable on board LED as output for checking ISR firing
	
	while( (TCC0_INTFLAGS & 0x01) == 0);	 			// waits for the new compare value to be loaded
	TCC0_INTFLAGS = 0x00;					 	// clear the interrupt flag

	PORTA_OUT = 0x00;						 // set the DC motors to null to begin with before the infinite while loop 
	
	while(1){
	
				
				IR_Read();				// Call to function will read the IR sensors and set car direction flag
				Load_Detect();				// Call to function will read the Load circuit network and reassign the car direction flag if needed
				Dist_Detect();				// Call to function will read the distance sensor and reassign the car direction flag if needed
				Config_Set();				// Configures the output to the motor driver to control the car's motion
				
			}

}
