/*
 * main.cpp
 *
 *  Created on: 25-05-2012
 *      Author: Luke
 */

#include <avr/io.h>
#include "snuffie.h"

/*
 * SENSORS
 * PA0..7 PC7..0
 */

snuffie Snuffie; //don't know how to change data from interrupt -> made this global variable

ISR(INT0_vect){
	SBI(PORTD,7);
	PORTB |= 0b10000000;
}
ISR(INT1_vect){
	CBI(PORTD,7);
	PORTB &= 0b01111111;
}
ISR(TIMER1_OVF_vect){
	Snuffie.calculate_error();
	Snuffie.calculate_speed();
	Snuffie.set_motor_speed();
	//OCR1A = 1023;
	//OCR1B = 1023;
}
ISR(TIMER2_COMP_vect){
	Snuffie.sensors_scan();
}

int main(){
	while(1){
	}

	return 0;
}
