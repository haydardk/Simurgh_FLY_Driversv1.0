/*
 * servo_driver.h
 *
 *  Created on: Jul 17, 2023
 *      Author: CASPER
 */

#ifndef INC_SERVO_DRIVER_H_
#define INC_SERVO_DRIVER_H_

typedef enum Servo_CHANNELS{
	ServoCHANNEL1 =0, //RUDDER
	ServoCHANNEL2 =1, //ELEVATOR
	ServoCHANNEL3 =2, //AILERON RIGHT
	ServoCHANNEL4 =3, //AILERON LEFT
}Servo_Channels_e;

void servo_init(void);

void servo_enable(void);
void servo_disable(void);

void servo_set_duty_cycle(uint32_t duty, Servo_Channels_e channel);

void servo1 (double angel);
void servo2 (double angel);
void servo3 (double angel);
void servo4 (double angel);

#endif /* INC_SERVO_DRIVER_H_ */
