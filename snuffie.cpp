/*
 * snuffie.cpp
 *
 *  Created on: 25-05-2012
 *      Author: Luke
 */

/*
 * FUSE BITS
 * low: 0xFF
 * high: 0x89
 * ext: 0xFF
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
	 *	Using external oscillator 16MHz.
	 *	256 prescaler (16Mhz/256=62.5kHz 1MHz/256=3906Hz)
	 *	64 prescaler (>>>16MHz/64=250kHz<<< 1MHz/64=15625MHz).
	 *	clkI/O/256 (From prescaler):
	 *		CS12 = 1
	 *		CS11, CS10 = 0
	 *
	 *	Input Capture not used!
	 *		ICNC1, ICES1 = 0
	 *
	 *		TIMSK register:
	 *
	 *	PWM Timer1 Overflow interrupt enable
	 *		TOIE1 = 1;
	 *
	 *	Engines stop
	 *		OCR1A, OCR1B = 0
	 */
	TCCR1A |= (1<<COM1A1) | (1<<COM1B1) | (1<<WGM10) | (1<<WGM11);
	TCCR1B |= (1<<CS12);
	TIMSK |= (1<<TOIE1);
	OCR1A = 0;
	OCR1B = 0;

	/*
	 *		Sensors init
	 *	Sensors read in Timer2 overflow interrupt
	 *
	 *		TCCR2 register:
	 *
	 *	CTC mode
	 *		WGM21 = 1
	 *		WGM20 = 0
	 *
	 *	Normal port operation, OC2 disconnected
	 *		COM21, COM20 = 0
	 *
	 *	Frequency equation:
	 *	fOC2 = fclk_I/O / 2 / N / (1 + OCR2)
	 *	Sensor scan period ~5ms = 200Hz
	 *	fclk_I/O = 16MHz
	 *	prescaler N = 256
	 *	OCR2 = 155
	 *	fOC2 -> 200.32Hz
	 *
	 *	Prescaler N = 256
	 *		CS22 = 1
	 *		CS21, CS20 = 0
	 *
	 *		TIMSK register:
	 *
	 *	Timer2 Output Compare Match interrupt enable
	 *		OCIE2 = 1
	 */
	TCCR2 |= (1<<WGM21) | (1<<CS22);
	TIMSK |= (1<<OCIE2);
	OCR2 = 155;

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

	}
}
