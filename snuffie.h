/*
 * snuffie.h
 *
 *  Created on: 25-05-2012
 *      Author: Luke
 */

/*
 * SENSORS
 * PA0..7 PC7..0
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define SBI(reg,bit) reg|=1<<bit //set bit
#define CBI(reg,bit) reg&=~(1<<bit) //clear bit
#define TBI(reg,bit) reg^=1<<bit //toggle bit
#define CHBI(reg,bit) reg&(1<<bit) //check bit

/*
 * direction defines
 */
#define FORWARD 2
#define STOP 1
#define BACKWARD 0

/*
 * sensor status defines
 */
#define SEE 1
#define NSEE 0

/*
 * engine power define
 */
#define HALF_POWER 1
#define QUARTER_POWER 0

#ifndef SNUFFIE_H_
#define SNUFFIE_H_

class snuffie{
private:
	volatile uint8_t sensor_status[16]; //TODO: change for 16bit bitfield -> easier to detect many unique cases by value of variable
	int8_t factor[16];
	uint8_t left_motor_dir, right_motor_dir;
	int16_t left_motor_speed, right_motor_speed;
	int16_t PID_error, PID_output;
	uint8_t PIDreg_P;
	uint8_t speed_mode;
public:
	snuffie();

	void wait();

	void sensors_scan(); //ready
	void calculate_error(); //ready
	void calculate_speed(); //wrong, rewrite this shit!
	void set_motor_speed(); //ready

	void test();
};

#endif /* SNUFFIE_H_ */
