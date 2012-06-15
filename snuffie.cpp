/*
 * snuffie.cpp
 *
 *  Created on: 25-05-2012
 *      Author: Luke
 */


/*
 * INT0 -> toggle allow flag
 * or
 * INT0 -> setting ON/OFF all functions (sensors check freq timer, PWM itd.)
 */

#include "snuffie.h"

#define LED3 7 //Red LED
#define LED2 6 //Green center LED
#define LED1 5 //Green edge LED

snuffie::snuffie(){
	/*
	 * 		I/O init
	 * Inputs:
	 * PA0..7 - ground sensors
	 * PC0..7 - ground sensors
	 * PD0..1 - switches (Pull-Up)
	 * Outputs:
	 * PB0 & PB2..7 - H-Bridge control (direction, PWM, standby)
	 * PD5..7 - LEDs
	 *
	 * All DDRx and PORTx initial value = 0 (Input, Tri-State)
	 */
	DDRC = 0b00000000;
	DDRB = 0b11111101;
	DDRD = 0b11100000;
	PORTD = 0b00000011;

	/*
	 *		INT0..1 init
	 */
	EIMSK |= (1<<INT0) | (1<<INT1);

	/*
	 *		PWM init
	 *	Both engines using PWM generated with Timer1.
	 *
	 *		TCCR1A register:
	 *
	 *	Clear OCnA/OCnB/OCnC on compare match
	 *	when up-counting. Set OCnA/OCnB/OCnC on
	 *	compare match when downcounting.
	 *	Channels A and B:
	 *		COM1A0, COM1B0 = 0
	 *		COM1A1, COM1B1 = 1
	 *	Channel C not used!
	 *		COM1C0, COM1B1 = 0
	 *
	 *	PWM, Phase Correct, 10-bit
	 *		WGM10, WGM11 = 1;
	 *
	 *		TCCR1B register:
	 *
	 *	PWM, Phase Correct, 10-bit
	 *		WGM12, WGM13 = 0;
	 *
	 *	TB6612FNG H-Bridge max PWM frequency = 100kHz
	 *	While writing this code CPU clock is 1MHz.
	 *	16MHz external oscillator prepared to use.
	 *	Will change later.
	 *	Because of it I use 256 prescaler (16Mhz/256=62.5kHz 1MHz/256=3906Hz)
	 *	instead of 64 prescaler (>>>16MHz/64=250kHz<<< 1MHz/64=15625MHz).
	 *	clkI/O/256 (From prescaler):
	 *		CS12 = 1
	 *		CS11, CS10 = 0
	 *
	 *	Input Capture not used!
	 *		ICNC1, ICES1 = 0
	 */
	TCCR1A |= (1<<COM1A1) | (1<<COM1B1) | (1<<WGM10) | (1<<WGM11);
	TCCR1B |= (1<<CS12);

	/*
	 * temp variable init
	 */
	flaga = 0;

	/*
	 * Global Interrupt Flag in SREG register set
	 */
	sei();

	/*
	 * Wait for action after completing init
	 */
	wait();
}

void snuffie::wait(){
	PORTB = 0b11110001;
	while(1){
		OCR1A = 1023;
		OCR1B = 1023;

	}
}
