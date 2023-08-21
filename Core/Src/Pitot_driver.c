/*
 * Pitot_drviver.c
 *
 *  Created on: Aug 4, 2023
 *      Author: CASPER
 */


#include "stm32f4xx.h"
#include "Pitot_driver.h"
#include "stdint.h"
#include "math.h"

extern ADC_HandleTypeDef hadc1 ;

uint32_t adc_buffer[200];

extern float  air_speed, voltage ;
extern uint32_t raw_Volt;
extern float kpascal_pitot ;
extern float pitot_velocity ;

void Pitot_ADC_Read(void){
	HAL_ADC_Start(&hadc1);

	raw_Volt = HAL_ADC_GetValue(&hadc1);

	voltage = raw_Volt * 5 / 4095; // 5V and 4095 !!!!
	air_speed = ((voltage - 0.5) / 4.5) * 1000;

	//Vout = VS x (0.2 x P(kPa) + 0.5) ± 6.25% VFSS FROM DATSHEET

	kpascal_pitot = (raw_Volt - 0.5 )/5/(0.2) ;
    pitot_velocity = sqrt(2* (kpascal_pitot-2900) / 1.225 ); //   m/s !!!
}
/* extern I2C_HandleTypeDef hi2c2;

//#define PITOTADDR 0x00  // I DONT EVEN KNOW AMK
//#define WHOAMI 0x00


#define MS4525DO_I2C_ADDRESS 0x28
#define I_ADDR 0x28
#define J_ADDR 0x36
#define K_ADDR 0x46

#define referencePressure 101325
int32_t pressure = 0;

#define ZERO_ADDR 0x48

double Air_Speed_Calc(void){
	 float deltaPressure = pressure - referencePressure;
	 float airDensity = 1.225;
	 float airSpeed = sqrt(2 * deltaPressure / airDensity);
	 return airSpeed;
}

uint16_t MS4525_Read(uint8_t regAddr) {
    uint8_t data[2];
    if (HAL_I2C_Mem_Read(&hi2c2, MS4525DO_I2C_ADDRESS, regAddr, I2C_MEMADD_SIZE_8BIT, data, 2, 100) != HAL_OK) {
        // Error Handling

    }
    return ((uint16_t)data[0] << 8) | data[1];
}

int32_t MS4525DO_ReadPressure(void) {

    uint16_t rawPressure = MS4525_Read(0x28);

    // Combine the 2 bytes into a signed 16-bit integer (big-endian format)
    pressure = (int32_t)rawPressure;

    // Convert pressure to hPa (hPa = Pa / 100)
    pressure = pressure / 100;

    return pressure;
}

int16_t MS4525DO_ReadTemperature(void) {
    int16_t temperature;
    uint16_t rawTemperature = MS4525_Read(0x32);

    // Combine the 2 bytes into a signed 16-bit integer (big-endian format)
    temperature = (int16_t)rawTemperature;

    // Convert temperature to Celsius (Celsius = °C * 0.01)
    temperature = temperature * 0.01;

    return temperature;
}  */

/* uint8_t Pitot_Buffer[4];

int16_t Pitot_Test (void) {
    HAL_StatusTypeDef pitot_durum;

    pitot_durum = HAL_I2C_IsDeviceReady(&hi2c2, MS4525DO_I2C_ADDRESS, 1, 100);

    if (pitot_durum == HAL_OK) {
        return 1;
    } else {
        return 0;
    }
}
void Pitot_Init(void){
	// MX_I2C2_Init();
	 *
	 *
	void Pitot_ReadData(void)
{
	HAL_I2C_Mem_Read(&hi2c2, MS4525DO_I2C_ADDRESS, 0x00, 8, Pitot_Buffer, 4, 100);
}

void Process_Pitot(void){
	int16_t ProData[2];
	ProData[0] = (Pitot_Buffer[0]<<8) | Pitot_Buffer[1];
	ProData[1] = (Pitot_Buffer[2]<<8) | Pitot_Buffer[3];

}
} */




//*************** SENSOR GENERAL PURPOSE FUNCTIONS ! THIS PART WILL NOT BE USED   ********************

/*
#define I2C_TIMEOUT ( 100 )
#define I2C_NUMBER_OF_TRIALS ( 4 )
#define I2C_REG_ADD_SIZE_1_BYTE ( 1 )
#define I2C_DATA_SIZE_1_BYTE  ( 1 )
#define I2C_DATA_SIZE_2_BYTES ( 2 )

uint16_t bytesToUint16_LittleEndian(uint8_t *pdata);
uint16_t bytesToUint16_BigEndian(uint8_t *pdata);

sensor_status_e sensor_test_device( uint8_t chipAdd )
{
	HAL_StatusTypeDef status;

    status = HAL_I2C_IsDeviceReady( &hi2c2, chipAdd, I2C_NUMBER_OF_TRIALS, I2C_TIMEOUT );

	if ( HAL_OK == status ) {
		return SENSOR_OK;
	}
	else {
		return SENSOR_ERROR;
	}
}

uint8_t sensor_read_register8(uint8_t chipAdd, uint8_t regAdd)
{
	uint8_t data;
	HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read( &hi2c2, chipAdd, regAdd, I2C_REG_ADD_SIZE_1_BYTE, &data, I2C_DATA_SIZE_1_BYTE, I2C_TIMEOUT );

	if (HAL_OK != status) {
		return 0;
	} else {
		return data;
	}
}

uint16_t sensor_read_register16(uint8_t chipAdd, uint8_t regAdd)
{
	uint8_t data[2];
	uint16_t retVal;
	HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read( &hi2c2, chipAdd, regAdd, I2C_REG_ADD_SIZE_1_BYTE, data, I2C_DATA_SIZE_2_BYTES, I2C_TIMEOUT );

	if (HAL_OK != status) {
		return 0;
	} else {
		retVal = (data[1]<<8) | data[0];
		return retVal;
	}
}

sensor_status_e sensor_write_register8(uint8_t chipAdd, uint8_t regAdd, uint8_t value)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Write( &hi2c2, chipAdd, regAdd, I2C_REG_ADD_SIZE_1_BYTE, &value, I2C_DATA_SIZE_1_BYTE, I2C_TIMEOUT );

    if (HAL_OK != status) {
        return SENSOR_ERROR;
    } else {
        return SENSOR_OK;
    }
}

sensor_status_e sensor_write_register16(uint8_t chipAdd, uint8_t regAdd, uint16_t value)
{
	uint8_t data[2];
	HAL_StatusTypeDef status;

	data[0] = (uint8_t) ((value >> 8) & 0xFF); // High Byte
	data[1] = (uint8_t) (value & 0xFF); // Low Byte

    status = HAL_I2C_Mem_Write( &hi2c2, chipAdd, regAdd, I2C_REG_ADD_SIZE_1_BYTE, data, I2C_DATA_SIZE_2_BYTES, I2C_TIMEOUT );

	if (HAL_OK != status) {
		return SENSOR_ERROR;
	} else {
		return SENSOR_OK;
	}
}


sensor_status_e sensor_read_bytes(uint8_t chipAdd, uint8_t regAdd, uint8_t *pBuffer, uint8_t size)
{
	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Read(&hi2c2, chipAdd, regAdd, I2C_REG_ADD_SIZE_1_BYTE, pBuffer, size, I2C_TIMEOUT);

	if (HAL_OK != status) {
		return SENSOR_ERROR;
	} else {
		return SENSOR_OK;
	}
}

sensor_status_e sensor_write_bytes(uint8_t chipAdd, uint8_t regAdd, uint8_t *pBuffer, uint8_t size)
{
	HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Write( &hi2c2, chipAdd, regAdd, I2C_REG_ADD_SIZE_1_BYTE, pBuffer, size, I2C_TIMEOUT );

	if (HAL_OK != status) {
		return SENSOR_ERROR;
	} else {
		return SENSOR_OK;
	}
}

uint16_t bytesToUint16_LittleEndian(uint8_t *pdata)
{
    return ( (pdata[1]<<8) | pdata[0] );
}

uint16_t bytesToUint16_BigEndian(uint8_t *pdata)
{
    return ( (pdata[0]<<8) | pdata[1] );
}
*/

