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
	Snuffie.eng1 = 0;
	Snuffie.eng2 = 0;

}
ISR(INT1_vect){
	Snuffie.eng1 = 1023;
	Snuffie.eng2 = 1023;
}
ISR(TIMER1_OVF_vect){
	OCR1A = Snuffie.eng1;
	OCR1B = Snuffie.eng2;
}
ISR(TIMER2_COMP_vect){
	for(int i=0;i<8;i++){
		Snuffie.sensors[i]=CHBI(PINA,i);
	}
	for(int j=8;j<16;j++){
		Snuffie.sensors[j]=CHBI(PINC,(15-j));
	}
	if((Snuffie.sensors[0] == false) || (Snuffie.sensors[15]==false)){
		SBI(PORTD,7);
	}
	else CBI(PORTD,7);
}

int main(){
	while(1){
	}

	return 0;
}
