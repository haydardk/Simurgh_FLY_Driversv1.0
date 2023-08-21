/*
 * sbusd1.h
 *
 *  Created on: Jul 14, 2023
 *      Author: CASPER
 */

#ifndef INC_SBUS_DRIVER_H_
#define INC_SBUS_DRIVER_H_

#include <Sbus_Driver.h>
#include "main.h"
#include "stdint.h"

extern UART_HandleTypeDef huart2 ;

#define SBUS_PACKET_SIZE 25
#define DATA_START_BIT 10
#define DATA_LENGTH 120

#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03

void SBUS_Init(void);


#endif /* INC_SBUS_DRIVER_H_ */
