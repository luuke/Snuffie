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
}
ISR(INT1_vect){
	SBI(PORTD,6);
}

int main(){
	while(1){
	}

	return 0;
}
