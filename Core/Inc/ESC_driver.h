/*
 * ESC_driver.h
 *
 *  Created on: Aug 3, 2023
 *      Author: CASPER
 */

#ifndef INC_ESC_DRIVER_H_
#define INC_ESC_DRIVER_H_

#include "stdint.h"


void ESC_init(void);
void ESC_enable(void);
void ESC_disable(void);
void ESC_set_duty_cycle(uint32_t duty, int channel);
void ESC1_Run(uint32_t ratio);
void ESC_Calibrate(void);


#endif /* INC_ESC_DRIVER_H_ */
