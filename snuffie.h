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

#ifndef SNUFFIE_H_
#define SNUFFIE_H_

class snuffie{
private:
	volatile uint8_t sensor_status[16];
	int8_t factor[16];
	uint16_t motor_left, motor_right;
	uint16_t reg_P, reg_I, reg_D;
	int8_t dir_left, dir_right; //need init!
	int16_t last_scan;
	uint8_t if_see;

public:
	uint8_t enable;

	snuffie();
	void wait();

	void sensors_scan(); //OK
	void calculate_speed();
	void set_speed(int8_t dir_left_temp = 1, int8_t dir_right_temp = 1);
	void test();
};

#endif /* SNUFFIE_H_ */
