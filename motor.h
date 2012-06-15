/*
 * motor.h
 *
 *  Created on: 25-05-2012
 *      Author: Luke
 */

#include <stdint.h>

#ifndef MOTOR_H_
#define MOTOR_H_

#define FORWARD 1
#define BACKWARD 0

class motor{
public:
	void init();
	void speed(uint16_t speed, uint8_t direction = FORWARD);

	motor();
	~motor();
};


#endif /* MOTOR_H_ */
