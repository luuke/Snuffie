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
	Snuffie.enable = 0;
}
ISR(INT1_vect){
	Snuffie.enable = 1;
}
ISR(TIMER1_OVF_vect){
	Snuffie.calculate_speed();
	Snuffie.set_speed();
}
ISR(TIMER2_COMP_vect){
	Snuffie.sensors_scan();
}

int main(){
	while(1){
	}

	return 0;
}
