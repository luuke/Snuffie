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

/*
 * What have any influence on robot's ride quality?
 * - sensors scan frequency
 * - PID regulator values
 *
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
	 *
	 *		OCR1A - left motor
	 *		OCR1B - right motor
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
	OCR2 = 155; //add here something to easy change scan freq

	/*
	 *		Sensors factors init
	 *	|-7|-6|-5|-4|-3|-2|-1|0|0|+1|+2|+3|+4|+5|+6|+7|
	 */
	for(int i=0;i<8;i++){
		factor[i] = (-7+i);
		factor[8+i] = i;
	}

	factor[0] = -20; //temp
	factor[15] = 20; //temp

	/*
	 * 		PID regulator values set
	 * 	PID_P >= 1
	 */
	PIDreg_P = 35;

	/*
	 * Global Interrupt Flag in SREG register set
	 */
	sei();

	/*
	 *
	 * PORTB description
	 * PB0 - AIN2
	 * PB1 - SCK
	 * PB2 - AIN1
	 * PB3 - BIN1
	 * PB4 - BIN2
	 * PB5 - PWMA
	 * PB6 - PWMB
	 * PB7 - /STBY
	 *
	 *		Motors directions init
	 * |------|------|-----------|
	 * |      LEFT ENGINE        |
	 * |------|------|-----------|
	 * | AIN1 | AIN2 | direction |
	 * |------|------|-----------|
	 * |  0   |  0   | STOP      |
	 * |  1   |  0   | BACKWARD  |
	 * |  0   |  1   | FORWARD   |
	 * |  1   |  1   | BRAKE     |
	 * |------|------|-----------|
	 * |      RIGHT ENGINE       |
	 * |------|------|-----------|
	 * | BIN1 | BIN2 | direction |
	 * |------|------|-----------|
	 * |  0   |  0   | STOP      |
	 * |  1   |  0   | BACKWARD  |
	 * |  0   |  1   | FORWARD   |
	 * |  1   |  1   | BRAKE     |
	 * |------|------|-----------|
	 *
	 * Both engines FORWARD init
	 * 		AIN2, BIN2 = 1
	 * 		AIN1, BIN1 = 0
	 *
	 * H-Bridge standby (off)
	 * 		PB7 = 0
	 */
	PORTB = 0b00010001;
	left_motor_dir = FORWARD;
	right_motor_dir = FORWARD;

	/*
	 * Wait for action after completing init
	 */
	wait();
}

void snuffie::wait(){
	while(1){

	}
}

void snuffie::sensors_scan(){
	for(int i=0; i<8; i++){
		sensor_status[i] = CHBI(PINA,i);
		sensor_status[15-i] = CHBI(PINC,i);
	}
}

void snuffie::calculate_error(){
	PID_error = 0;
	//uint8_t counter = 0;
	for(int i=0; i<16; i++){
		if(sensor_status[i] != 0){
			PID_error += factor[i];
			//HERE CAN ADD HOW MANY SENSORS SEE LINE
			//counter++;
		}
	}
	//PID_error /= counter;
}

void snuffie::calculate_speed(){
	int16_t PID_output = 0;

	//P
	PID_output += PIDreg_P * PID_error;
	//I

	//D

	//output
	left_motor_speed = 1023 + PID_output;
	right_motor_speed = 1023 - PID_output;

	//setting speed mode
	if(left_motor_speed > 1023){
		if(right_motor_speed >= 0){
			speed_mode = 1;
		}
		else if(right_motor_speed >= -1023){
			speed_mode = 2;
		}
		else if(right_motor_speed < -1023){
			speed_mode = 3;
		}
	}
	else if(left_motor_speed >= 0){
		if(right_motor_speed > 1023){
			speed_mode = 4;
		}
		else if(right_motor_speed >= 0){
			speed_mode = 5;
		}
		else if(right_motor_speed >= -1023){
			speed_mode = 6;
		}
		else if(right_motor_speed < -1023){
			speed_mode = 7;
		}
	}
	else if(left_motor_speed >= -1023){
		if(right_motor_speed > 1023){
			speed_mode = 8;
		}
		else if(right_motor_speed >= 0){
			speed_mode = 9;
		}
		else if(right_motor_speed >= -1023){
			speed_mode = 10;
		}
		else if(right_motor_speed < -1023){
			speed_mode = 11;
		}
	}
	else if(left_motor_speed < -1023){ //need 'else if'? maybe 'if' only?
		if(right_motor_speed > 1023){
			speed_mode = 12;
		}
		else if(right_motor_speed >= 0){
			speed_mode = 13;
		}
		else if(right_motor_speed >= -1023){
			speed_mode = 14;
		}
	}


#if HALF_POWER
	left_motor_speed = (left_motor_speed>>1);
	right_motor_speed = (right_motor_speed>>1);
#endif //HALF_POWER
#if QUARTER_POWER
	left_motor_speed = (left_motor_speed>>2);
	right_motor_speed = (right_motor_speed>>2);
#endif //QUARTER_POWER
}

void snuffie::set_motor_speed(){
	uint16_t temp;
	uint8_t new_left_dir, new_right_dir;

	//Setting speed
	switch(speed_mode){
	case 1:

		break;
	case 2:

		break;
	case 3:

		break;
	case 4:

		break;
	case 5:

		break;
	case 6:

		break;
	case 7:

		break;
	case 8:

		break;
	case 9:

		break;
	case 10:

		break;
	case 11:

		break;
	case 12:

		break;
	case 13:

		break;
	case 14:

		break;
	}

	//Setting direction        FREE TRASH!
	if(left_motor_dir != new_left_dir){
		PORTB ^= 0b00011000;
		left_motor_dir = new_left_dir;
	}
	if(right_motor_dir != new_right_dir){
		PORTB ^= 0b00000101;
		left_motor_dir = new_right_dir;
	}
}

void snuffie::test(){

}
