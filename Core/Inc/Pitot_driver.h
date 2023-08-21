/*
 * Pitot_driver.h
 *
 *  Created on: Aug 4, 2023
 *      Author: CASPER
 */

#ifndef INC_PITOT_DRIVER_H_
#define INC_PITOT_DRIVER_H_

#define PITOTADDR 0x00  // I DONT EVEN KNOW AMK
#define WHOAMI 0x00

#define I_ADDR 0x28
#define J_ADDR 0x36
#define K_ADDR 0x46

#define ZERO_ADDR 0x48

void Pitot_ADC_Read (void);

double Air_Speed_Calc(void);
uint16_t MS4525_Read(uint8_t regAddr);
int32_t MS4525DO_ReadPressure(void);
int16_t MS4525DO_ReadTemperature(void);


typedef enum {
	SENSOR_OK,
	SENSOR_ERROR
}sensor_status_e;


sensor_status_e sensor_test_device( uint8_t chipAdd );
uint8_t sensor_read_register8(uint8_t chipAdd, uint8_t regAdd);
uint16_t sensor_read_register16(uint8_t chipAdd, uint8_t regAdd);
sensor_status_e sensor_write_register16(uint8_t chipAdd, uint8_t regAdd, uint16_t value);
sensor_status_e sensor_write_register8(uint8_t chipAdd, uint8_t regAdd, uint8_t value);
sensor_status_e sensor_read_bytes(uint8_t chipAdd, uint8_t regAdd, uint8_t *pBuffer, uint8_t size);
sensor_status_e sensor_write_bytes(uint8_t chipAdd, uint8_t regAdd, uint8_t *pBuffer, uint8_t size);


#endif /* INC_PITOT_DRIVER_H_ */
