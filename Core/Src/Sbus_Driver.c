/*
 * sbusd1.c
 *
 *  Created on: Jul 14, 2023
 *      Author: CASPER
 */
#include <Sbus_Driver.h>
#include "main.h"
#include "stdint.h"

extern UART_HandleTypeDef huart1 ;

#define SBUS_PACKET_SIZE 25
#define DATA_START_BIT 10
#define DATA_LENGTH 120

#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03


uint8_t buffer[25];
uint16_t channels[18];

extern uint8_t Sbus_Buffer[25];
extern uint16_t Command_Channels[18];

int lenght=0;
uint16_t USB_Send_Data[]={0};

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	//if (buffer[0] == 0x0F) {
	/* channels[0]  = ((buffer[1] << 8) | buffer[2]) >> 3 & 0x07FF;
	    channels[1]  = ((buffer[3] << 8) | buffer[4]) >> 3 & 0x07FF;
	    channels[2]  = ((buffer[5] << 8) | buffer[6]) >> 3 & 0x07FF;
	    channels[3]  = ((buffer[7] << 8) | buffer[8]) >> 3 & 0x07FF;
	    channels[4]  = ((buffer[9] << 8) | buffer[10]) >> 3 & 0x07FF;
	    channels[5]  = ((buffer[11] << 8) | buffer[12]) >> 3 & 0x07FF;
	    channels[6]  = ((buffer[13] << 8) | buffer[14]) >> 3 & 0x07FF;
	    channels[7]  = ((buffer[15] << 8) | buffer[16]) >> 3 & 0x07FF;
	    channels[8]  = ((buffer[17] << 8) | buffer[18]) >> 3 & 0x07FF;
	    channels[9]  = ((buffer[19] << 8) | buffer[20]) >> 3 & 0x07FF;
	    channels[10] = ((buffer[21] << 8) | buffer[22]) >> 3 & 0x07FF;
	    channels[11] = ((buffer[23] << 8) | buffer[24]) >> 3 & 0x07FF;  */
	// ALINAN KOD AŞAĞIDA
	Command_Channels[0] = (Sbus_Buffer[1] >> 0 | (Sbus_Buffer[2] << 8)) & 0x07FF;
	Command_Channels[1] = (Sbus_Buffer[2] >> 3 | (Sbus_Buffer[3] << 5)) & 0x07FF;
	Command_Channels[2] = (Sbus_Buffer[3] >> 6 | (Sbus_Buffer[4] << 2) | Sbus_Buffer[5] << 10) & 0x07FF;
	Command_Channels[3] = (Sbus_Buffer[5] >> 1 | (Sbus_Buffer[6] << 7)) & 0x07FF;
	Command_Channels[4] = (Sbus_Buffer[6] >> 4 | (Sbus_Buffer[7] << 4)) & 0x07FF;
	Command_Channels[5] = (Sbus_Buffer[7] >> 7 | (Sbus_Buffer[8] << 1) | Sbus_Buffer[9] << 9) & 0x07FF;
	Command_Channels[6] = (Sbus_Buffer[9] >> 2 | (Sbus_Buffer[10] << 6)) & 0x07FF;
	Command_Channels[7] = (Sbus_Buffer[10] >> 5 | (Sbus_Buffer[11] << 3)) & 0x07FF;
	Command_Channels[8] = (Sbus_Buffer[12] << 0 | (Sbus_Buffer[13] << 8)) & 0x07FF;
	Command_Channels[9] = (Sbus_Buffer[13] >> 3 | (Sbus_Buffer[14] << 5)) & 0x07FF;
	Command_Channels[10] = (Sbus_Buffer[14] >> 6 | (Sbus_Buffer[15] << 2) | Sbus_Buffer[16] << 10) & 0x07FF;
	Command_Channels[11] = (Sbus_Buffer[16] >> 1 | (Sbus_Buffer[17] << 7)) & 0x07FF;
	Command_Channels[12] = (Sbus_Buffer[17] >> 4 | (Sbus_Buffer[18] << 4)) & 0x07FF;
	Command_Channels[13] = (Sbus_Buffer[18] >> 7 | (Sbus_Buffer[19] << 1) | Sbus_Buffer[20] << 9) & 0x07FF;
	Command_Channels[14] = (Sbus_Buffer[20] >> 2 | (Sbus_Buffer[21] << 6)) & 0x07FF;
	Command_Channels[15] = (Sbus_Buffer[21] >> 5 | (Sbus_Buffer[22] << 3)) & 0x07FF;

}


void SBUS_Init(void){
	HAL_UART_Receive_DMA(&huart1, Sbus_Buffer, 25);
	HAL_UART_RxCpltCallback(&huart1);
}


