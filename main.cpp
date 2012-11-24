/*
 * main.cpp
 *
 *  Created on: 25-05-2012
 *      Author: Luke
 */

#include <avr/io.h>
#include "snuffie.h"

snuffie Snuffie; //don't know how to change data from interrupt -> made this global variable

ISR(INT0_vect){
	SBI(PORTD,7);
	SBI(HBRIDGE_PORT, HBRIDGE_STBY);
}
ISR(INT1_vect){
	CBI(PORTD,7);
	CBI(HBRIDGE_PORT, HBRIDGE_STBY);
}
ISR(TIMER1_OVF_vect){
	SBI(LED_PORT,GREEN_C_LED);
	Snuffie.calculate_speed();
	Snuffie.set_motor_speed();
	CBI(LED_PORT,GREEN_C_LED);
}
ISR(TIMER2_COMP_vect){
	SBI(LED_PORT,GREEN_E_LED);
	Snuffie.sensors_scan();
	CBI(LED_PORT,GREEN_E_LED);
}

int main(){
	while(1){}
	return 0;
}
