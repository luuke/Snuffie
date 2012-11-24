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
 * - sensors factors
 *
 */

#include "snuffie.h"

#define LED3 7 //Red LED
#define LED2 6 //Green center LED
#define LED1 5 //Green edge LED

snuffie::snuffie(){
	name[0] = 'S';
	name[1] = 'n';
	name[2] = 'u';
	name[3] = 'f';
	name[4] = 'f';
	name[5] = 'i';
	name[6] = 'e';
	name[7] = '\0';

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
	 *	256 prescaler (>>>16Mhz/256=62.5kHz<<< 1MHz/256=3906Hz)
	 *	64 prescaler (16MHz/64=250kHz 1MHz/64=15625Hz).
	 *	clkI/O/256 (From prescaler):
	 *		CS12 = 1
	 *		CS11, CS10 = 0
	 *
	 *		>>>THINK OF LOWERING PWM FREQ<<<
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
	 *		Sensors init (NEED CORRECTION)
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
	 *	prescaler N = 1024
	 *	OCR2 = 155
	 *	fOC2 -> 200.32Hz
	 *
	 *	Prescaler N = 1024
	 *		CS22, CS20 = 1
	 *		CS21 = 0
	 *
	 *		TIMSK register:
	 *
	 *	Timer2 Output Compare Match interrupt enable
	 *		OCIE2 = 1
	 */
	TCCR2 |= (1<<WGM21) | (1<<CS22) | (1<<CS20);
	TIMSK |= (1<<OCIE2);
#define SENSOR_SCAN_FREQUENCY 50
	OCR2 = 16000000 / 1024 / 2 / SENSOR_SCAN_FREQUENCY - 1; //add here something to easy change scan freq
#undef SENSOR_SCAN_FREQUENCY
	/*
	 *		Sensors factors init
	 *	|-7|-6|-5|-4|-3|-2|-1|0|0|+1|+2|+3|+4|+5|+6|+7|
	 */
	for(int i=0;i<8;i++){
		factor[i] = (-7+i);
		factor[8+i] = i;
	}

	/*
	 * 		PID regulator values set
	 * 	PID_P >= 1
	 */
	PIDreg_P = 100;

	/*
	 * 		UART init (NEED COMPLETION)
	 *	Using UART0 - asynchronous operation
	 *
	 *		USCR0A register - not used for UART init
	 *
	 *		USCR0B register:
	 *
	 *	Receiver enable
	 *		RXEN0 = 1
	 *
	 *	Transmitter enable
	 *		TXEN0 = 1
	 *
	 *		USCR0C register:
	 *
	 *	Asynchronous operation
	 *		UMSEL0 = 0
	 *
	 *	Parity mode disable
	 *		UPM00, UPM01 = 0
	 *
	 *	2 stop bits
	 *		USBS0 = 1
	 *
	 *	8-bit character size
	 *		UCSZ00, UCSZ01 = 1
	 *		UCSZ02 = 0 (USCR0B register)
	 *
	 *		UBRR0 register:
	 *
	 *	Baud rate = 9600
	 *		UBRR0 = 103
	 *
	 *
	 */
#if DEBUG
	UCSR0B |= (1<<RXEN)|(1<<TXEN);
	UCSR0C |= (1<<USBS)|(3<<UCSZ0);
	UART_ubrr = 103;
	UBRR0H = (unsigned char)(UART_ubrr>>8);
	UBRR0L = (unsigned char)UART_ubrr;
#endif //DEBUG

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
		//UART_monitor();
	}
}

void snuffie::sensors_scan(){


	for(int i=0; i<8; i++){
		//scan
		new_sensor_status[i] = CHBI(PINA,i);
		new_sensor_status[15-i] = CHBI(PINC,i);

#if OLD
		//if not yet seen the line
		seen_line = 0;
		if(seen_line == 0) seen_line = new_sensor_status[i] != 0 ? 1 : 0;
		if(seen_line == 0) seen_line = new_sensor_status[15-i] != 0 ? 1 : 0;
#endif //OLD
	}
}

void snuffie::calculate_speed(){
	PID_error = 0;
	int16_t PID_output = 0;

	//new sensor scan to be used
	if(seen_line != 0){
		for(int i=0; i<8; i++){
			sensor_status[i] = new_sensor_status[i];
			sensor_status[15-i] = new_sensor_status[15-i];
		}
	}
	//uint8_t counter = 0;
	for(int i=0; i<16; i++){
		if(sensor_status[i] != 0){
			PID_error += factor[i];
			//HERE CAN ADD HOW MANY SENSORS SEE LINE
			//counter++;
		}
	}
	//PID_error /= counter;

	//P
	PID_output += PIDreg_P * PID_error;
	//I

	//D

	//output
	left_motor_speed = 1023 + PID_output;
	right_motor_speed = 1023 - PID_output;

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

	//Speed values must stay in range of <-1023;1023>
	if(left_motor_speed > 1023){
		temp = left_motor_speed - 1023;
		left_motor_speed = 1023;
		right_motor_speed -= temp;
		if(right_motor_speed < -1023) right_motor_speed = -1023;
	}
	else if(right_motor_speed > 1023){
		temp = right_motor_speed - 1023;
		right_motor_speed = 1023;
		left_motor_speed -= temp;
		if(left_motor_speed < -1023) left_motor_speed = -1023;
	}

	//
	if(left_motor_speed >= 0) new_left_dir = FORWARD;
	else{
		left_motor_speed *= (-1);
		new_left_dir = BACKWARD;
	}
	if(right_motor_speed >= 0) new_right_dir = FORWARD;
	else{
		right_motor_speed *= (-1);
		new_right_dir = BACKWARD;
	}

	//Setting PWM values
	OCR1A = left_motor_speed;
	OCR1B = right_motor_speed;

	//Changing directions in PORTB register        FREE TRASH!
	if(left_motor_dir != new_left_dir){
		PORTB ^= 0b00000101;
		left_motor_dir = new_left_dir;
	}
	if(right_motor_dir != new_right_dir){
		PORTB ^= 0b00011000;
		right_motor_dir = new_right_dir;
	}
}

#if DEBUG
void snuffie::UART_transmit_char(uint8_t char_to_transmit){
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = char_to_transmit;
}

void snuffie::UART_transmit_string(uint8_t* string_to_transmit){
	while(*string_to_transmit){
		while(!(UCSR0A & (1<<UDRE0)));
		UDR0 = *string_to_transmit++;
	}
}

void snuffie::UART_transmit_number(int16_t number_to_transmit){
	char number[6];
	char *pnumber = &number[0];

	itoa(number_to_transmit, pnumber, 10);
	while(*pnumber){
		while(!(UCSR0A & (1<<UDRE0)));
		UDR0 = *pnumber++;
	}
}

uint8_t snuffie::UART_receive_char(){
	uint8_t incoming_char;

	while(!(UCSR0A & (1<<RXC0)));
	incoming_char = UDR0;
	return incoming_char;
}

void snuffie::UART_receive_string(uint8_t* string_array){
	uint8_t *pstring = string_array;

	do{
		while(!(UCSR0A & (1<<RXC0)));
		*pstring = UDR0;
	}while('\0' != *pstring++);
}

void snuffie::UART_monitor(){
	uint8_t request_sign = UART_receive_char();

	switch(request_sign){
	case 'd': //prepare for incoming data
		request_sign = UART_receive_char();
		switch(request_sign){
		case 'P':
			uint8_t received_string[6];
			UART_receive_string(received_string);
			PIDreg_P = atoi((char*)received_string);
			break;
		case 'I':

			break;
		case 'D':

			break;
		}
		break;
	case 'l':
		UART_transmit_number(left_motor_speed);
		break;
	case 'n': //name request
		UART_transmit_string(name);
		break;
	case 'p': //
		UART_transmit_number(PIDreg_P);
		break;
	case 'r': //
		UART_transmit_number(right_motor_speed);
		break;
	}
}
#endif //DEBUG

void snuffie::test(){

}
