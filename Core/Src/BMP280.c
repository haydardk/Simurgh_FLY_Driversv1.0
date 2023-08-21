/*
 * H-BME280.c
 *
 *  Created on: Jun 21, 2023
 *      Author: CASPER
 */

/* BME280 Library */

#include "BMP280.h"
#include "math.h"
#define PRESS_MSB_REG   0xF7
extern I2C_HandleTypeDef hi2c1;
#define BME280_I2C &hi2c1

#define SUPPORT_64BIT 1

#define BME280_ADDRESS (0x76<<1) // 1 1 1 0 1 1 X 0 , x=SDD => 0

extern float Temperature_BMP, Pressure_BMP, Humidity, Altitude_BMP;
float Temperature_BMP, Pressure_BMP, Humidity,Altitude_BMP;

// ****** //

uint8_t chipID;

uint8_t TrimParam[36];

int32_t tRaw, pRaw, hRaw;

uint16_t dig_T1, dig_P1; //from datasheet

int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8,
		dig_P9;

void TrimRead(void) {

	uint8_t trimdata[32];

	HAL_I2C_Mem_Read( BME280_I2C, BME280_ADDRESS, 0x88, 1, trimdata, 25,
			HAL_MAX_DELAY);

	dig_T1 = (trimdata[1] << 8) | trimdata[0];
	dig_T2 = (trimdata[3] << 8) | trimdata[2];
	dig_T3 = (trimdata[5] << 8) | trimdata[4];
	dig_P1 = (trimdata[7] << 8) | trimdata[5];
	dig_P2 = (trimdata[9] << 8) | trimdata[6];
	dig_P3 = (trimdata[11] << 8) | trimdata[10];
	dig_P4 = (trimdata[13] << 8) | trimdata[12];
	dig_P5 = (trimdata[15] << 8) | trimdata[14];
	dig_P6 = (trimdata[17] << 8) | trimdata[16];
	dig_P7 = (trimdata[19] << 8) | trimdata[18];
	dig_P8 = (trimdata[21] << 8) | trimdata[20];
	dig_P9 = (trimdata[23] << 8) | trimdata[22];
}
//RAW 0xF7 to 0xFC
int BMERawRead(void) {

	uint8_t RawData[8];
	HAL_I2C_Mem_Read(&hi2c1, BME280_ADDRESS, ID_REG, 1, &chipID, 1, 1000);

	if (chipID == 0x60) { // 0x60 forBME  0x58 forBMP

		HAL_I2C_Mem_Read(BME280_I2C, BME280_ADDRESS, PRESS_MSB_REG, 1, RawData,8, HAL_MAX_DELAY);

		pRaw = (RawData[0] << 12) | (RawData[1] << 4) | (RawData[2] >> 4);
		tRaw = (RawData[3] << 12) | (RawData[4] << 4) | (RawData[5] >> 4);
		// hRaw = (RawData[6]<<8) | (RawData[7]) ;

		return 0;
	} else
		return -1;
 }

// Returns Temperature_BMP in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine Temperature_BMP as global value
int32_t t_fine;
int32_t BME280_compensate_T_int32(int32_t adc_T) {
	int32_t var1, var2, T;
	var1 = ((((adc_T >> 3) - ((int32_t) dig_T1 << 1))) * ((int32_t) dig_T2))
			>> 11;
	var2 = (((((adc_T >> 4) - ((int32_t) dig_T1))
			* ((adc_T >> 4) - ((int32_t) dig_T1))) >> 12) * ((int32_t) dig_T3))
			>> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}
#if SUPPORT_64BIT
/* Returns Pressure_BMP in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
 Output value of â€œ24674867â€ represents 24674867/256 = 96386.2 Pa = 963.862 hPa
 */
uint32_t BME280_compensate_P_int64(int32_t adc_P) {
	int64_t var1, var2, p;
	var1 = ((int64_t) t_fine) - 128000;
	var2 = var1 * var1 * (int64_t) dig_P6;
	var2 = var2 + ((var1 * (int64_t) dig_P5) << 17);
	var2 = var2 + (((int64_t) dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t) dig_P3) >> 8)
			+ ((var1 * (int64_t) dig_P2) << 12);
	var1 = (((((int64_t) 1) << 47) + var1)) * ((int64_t) dig_P1) >> 33;
	if (var1 == 0) {
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t) dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t) dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t) dig_P7) << 4);
	return (uint32_t) p;
}
#endif

// Returns Pressure_BMP in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa


int BME280_Config(uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h, uint8_t mode,
		uint8_t t_sb, uint8_t filter) {
	// Read the Trimming parameters
	TrimRead();

	uint8_t datatowrite = 0;
	uint8_t datacheck = 0;

	// Reset the device
	datatowrite = 0xB6;  // reset sequence
	if (HAL_I2C_Mem_Write(BME280_I2C, BME280_ADDRESS, RESET_REG, 1,
			&datatowrite, 1, 1000) != HAL_OK) {
		return -1;
	}

	HAL_Delay(100);

	// write the humidity oversampling to 0xF2
	datatowrite = osrs_h;
	if (HAL_I2C_Mem_Write(BME280_I2C, BME280_ADDRESS, CTRL_HUM_REG, 1,
			&datatowrite, 1, 1000) != HAL_OK) {
		return -1;
	}
	HAL_Delay(100);
	HAL_I2C_Mem_Read(BME280_I2C, BME280_ADDRESS, CTRL_HUM_REG, 1, &datacheck, 1,
			1000);
	if (datacheck != datatowrite) {
		return -1;
	}

	// write the standby time and IIR filter coeff to 0xF5
	datatowrite = (t_sb << 5) | (filter << 2);
	if (HAL_I2C_Mem_Write(BME280_I2C, BME280_ADDRESS, CONFIG_REG, 1,
			&datatowrite, 1, 1000) != HAL_OK) {
		return -1;
	}
	HAL_Delay(100);
	HAL_I2C_Mem_Read(BME280_I2C, BME280_ADDRESS, CONFIG_REG, 1, &datacheck, 1,
			1000);
	if (datacheck != datatowrite) {
		return -1;
	}

	// write the Pressure_BMP and temp oversampling along with mode to 0xF4
	datatowrite = (osrs_t << 5) | (osrs_p << 2) | mode;
	if (HAL_I2C_Mem_Write(BME280_I2C, BME280_ADDRESS, CTRL_MEAS_REG, 1,
			&datatowrite, 1, 1000) != HAL_OK) {
		return -1;
	}
	HAL_Delay(100);
	HAL_I2C_Mem_Read(BME280_I2C, BME280_ADDRESS, CTRL_MEAS_REG, 1, &datacheck,
			1, 1000);
	if (datacheck != datatowrite) {
		return -1;
	}

	return 0;
}
void BME280_WakeUP(void) {
	uint8_t datatowrite = 0;

	// first read the register
	HAL_I2C_Mem_Read(BME280_I2C, BME280_ADDRESS, CTRL_MEAS_REG, 1, &datatowrite,
			1, 1000);

	// modify the data with the forced mode
	datatowrite = datatowrite | MODE_FORCED;

	// write the new data to the register
	HAL_I2C_Mem_Write(BME280_I2C, BME280_ADDRESS, CTRL_MEAS_REG, 1,
			&datatowrite, 1, 1000);

	HAL_Delay(100);
}
void BMP280_Altitude(void){
	/* Altitude_BMP= (-log(Pressure_BMP/101325))/(0.0000514892218924277);
	Altitude_BMP=Altitude_BMP/(3.28); // feet to meter */

	//H = (R * T0) / g * ln(P0 / P)
	Altitude_BMP= (287.05 * Temperature_BMP)/(9.81 * ((log(101.325/Pressure_BMP)/log(exp(1)))));
}



void BME280_Measure(void) {
	if (BMERawRead() == 0) {
		if (tRaw == 0x800000)
			Temperature_BMP = 0; // value in case temp measurement was disabled
		else {
			Temperature_BMP = (BME280_compensate_T_int32(tRaw)) / 100.0; // as per datasheet, the temp is x100
		}

		if (pRaw == 0x800000)
			Pressure_BMP = 0; // value in case temp measurement was disabled
		else {
#if SUPPORT_64BIT
			Pressure_BMP = (BME280_compensate_P_int64(pRaw)) / 256.0; // as per datasheet, the Pressure_BMP is x256

#elif SUPPORT_32BIT
			  Pressure_BMP = (BME280_compensate_P_int32 (pRaw));  // as per datasheet, the Pressure_BMP is Pa

#endif
		}

		/* if (hRaw == 0x8000)
			Humidity = 0; // value in case temp measurement was disabled
		else {
			Humidity = (bme280_compensate_H_int32(hRaw)) / 1024.0; // as per datasheet, the temp is x1024
		}*/
	}

	// if the device is detached
	else {
		Temperature_BMP = 0;
		Pressure_BMP = 0; // ADD Humidity
	}
}

