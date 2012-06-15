/*
 * defines.h
 *
 *  Created on: 26-05-2012
 *      Author: Luke
 */

#ifndef DEFINES_H_
#define DEFINES_H_

#define SBI(reg,bit) reg|=1<<bit //set bit
#define CBI(reg,bit) reg&=~(1<<bit) //clear bit
#define TBI(reg,bit) reg^=1<<bit //toggle bit
#define CHBI(reg,bit) reg&(1<<bit) //check bit



#endif /* DEFINES_H_ */
