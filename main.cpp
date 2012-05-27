/*
 * main.cpp
 *
 *  Created on: 25-05-2012
 *      Author: Luke
 */

//#include <avr/io.h>
#include "snuffie.h"

int main(){
	snuffie autko;
	//Snuffie.init();
	SBI(DDRD,7);
	while(1){
		//Snuffie.led_on();
		SBI(PORTD,7);
		_delay_ms(400);
		//Snuffie.led_off();
		CBI(PORTD,7);
		_delay_ms(400);
	}
	return 0;
}


