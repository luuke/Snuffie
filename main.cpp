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
int i=0;
ISR(INT0_vect){
	Snuffie.enable = 0;
	//CBI(PORTD,6);
}
ISR(INT1_vect){
	Snuffie.enable = 1;
	//SBI(PORTD,6);
}
ISR(TIMER1_OVF_vect){
	Snuffie.calculate_speed();
	Snuffie.set_speed();
	//TBI(PORTD,7);
}
ISR(TIMER2_COMP_vect){
	Snuffie.sensors_scan();
	/*
	i++;
	if(i==50){
		TBI(PORTD,5);
		CBI(PORTD,6);
		i=0;
	}
	*/
}

int main(){
	while(1){
	}

	return 0;
}
