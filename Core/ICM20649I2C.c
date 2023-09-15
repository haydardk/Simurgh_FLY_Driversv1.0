#include "ICM20649_I2C_Driver.h"

#include <stdint.h>
#include <stdio.h>

static void ICM20649_chipSelectSet(bool select);

/** @endcond */

/***************************************************************************//**
 * @brief
 *    Initializes the ICM20649 sensor. Enables the power supply and SPI lines,
 *    sets up the host SPI controller, configures the chip control interface,
 *    clock generator and interrupt line.
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20649_init(void)
{
  uint8_t data;

  /* Enable and setup I2C */
  ICM20649_i2cInit();

  /* Issue reset */
  ICM20649_reset();

  /* Disable SPI interface, use I2C */
  ICM20649_registerWrite(ICM20649_REG_USER_CTRL, ICM20649_BIT_I2C_IF_DIS);

  /* Read Who am I register, should get 0xe1 */
  ICM20649_registerRead(ICM20649_REG_WHO_AM_I, 1, &data);

  /* If not - return */
  if ( (data != ICM20649_DEVICE_ID) && (data != ICM20948_DEVICE_ID) ) {
    return ICM20649_ERROR_INVALID_DEVICE_ID;
  }

  /* Auto selects the best available clock source ? PLL if ready, else use the Internal oscillator */
  ICM20649_registerWrite(ICM20649_REG_PWR_MGMT_1, ICM20649_BIT_CLK_PLL);

  /* PLL startup time - maybe it is too long but better be on the safe side, no spec in the datasheet */
  UTIL_delay(30);

  /* INT pin: active low, open drain, IT status read clears. It seems that latched mode does not work, the INT pin cannot be cleared if set */
  ICM20649_registerWrite(ICM20649_REG_INT_PIN_CFG, ICM20649_BIT_INT_ACTL | ICM20649_BIT_INT_OPEN);

  return ICM20649_OK;
}

/***************************************************************************//**
 * @brief
 *    Performs soft reset on the ICM20649 chip
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20649_reset(void)
{
  /* Set H_RESET bit to initiate soft reset */
  ICM20649_registerWrite(ICM20649_REG_PWR_MGMT_1, ICM20649_BIT_H_RESET);

  /* Wait 100ms to complete the reset sequence */
  UTIL_delay(100);

  return ICM20649_OK;
}

/***************************************************************************//**
 * @brief
 *    Reads the raw gyroscope value and converts to deg/sec value based on
 *    the actual resolution
 *
 * @param[out] gyro
 *    A 3-element array of float numbers containing the gyroscope values
 *    for the x, y and z axes in deg/sec units.
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20649_gyroDataRead(float *gyro)
{
  uint8_t rawData[2];
  float gyroRes = 0.f;
  int16_t temp;

  /* Retrieve the current resolution */
  ICM20649_gyroResolutionGet(&gyroRes);

  /* Read the six raw data registers into data array */
  ICM20649_registerRead(ICM20649_REG_GYRO_XOUT_H_SH, 2, &rawData[0]);

  /* Convert the MSB and LSB into a signed 16-bit value and multiply by the resolution to get the dps value */
  temp = ( (int16_t) rawData[0] << 8) | rawData[1];
  gyro[0] = (float) temp * gyroRes;

  return ICM20649_OK;
}

/***************************************************************************//**
 * @brief
 *    Gets the actual resolution of the gyroscope
 *
 * @param[out] gyroRes
 *    The actual resolution in (deg/sec)/bit units
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20649_gyroResolutionGet(float *gyroRes)
{
  uint8_t reg;

  /* Read the actual gyroscope full scale setting */
  ICM20649_registerRead(ICM20649_REG_GYRO_CONFIG_1, 1, &reg);
  reg &= ICM20649_MASK_GYRO_FULLSCALE;

  /* Calculate the resolution */
  switch ( reg ) {
    case ICM20649_GYRO_FULLSCALE_500DPS:
      *gyroRes = 500.0 / 32768.0;
      break;

    case ICM20649_GYRO_FULLSCALE_1000DPS:
      *gyroRes = 1000.0 / 32768.0;
      break;

    case ICM20649_GYRO_FULLSCALE_2000DPS:
      *gyroRes = 2000.0 / 32768.0;
      break;

    case ICM20649_GYRO_FULLSCALE_4000DPS:
      *gyroRes = 4000.0 / 32768.0;
      break;
  }

  return ICM20649_OK;
}


/***************************************************************************//**
 * @brief
 *    Sets the full scale value of the gyroscope
 *
 * @param[in] gyroFs
 *    The desired full scale value. Use the ICM20649_GYRO_FULLSCALE_yDPS
 *    macros, which are defined in the ICM20649.h file. The value of y can be
 *    500, 1000, 2000 or 4000.
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20649_gyroFullscaleSet(uint8_t gyroFs)
{
  uint8_t reg;

  gyroFs &= ICM20649_MASK_GYRO_FULLSCALE;
  ICM20649_registerRead(ICM20649_REG_GYRO_CONFIG_1, 1, &reg);
  reg &= ~(ICM20649_MASK_GYRO_FULLSCALE);
  reg |= gyroFs;
  ICM20649_registerWrite(ICM20649_REG_GYRO_CONFIG_1, reg);

  return ICM20649_OK;
}

uint32_t ICM20649_sampleRateSet(float sampleRate)
{
  ICM20649_gyroSampleRateSet(sampleRate);

  return ICM20649_OK;
}

/***************************************************************************//**
 * @brief
 *    Sets the sample rate of the gyroscope
 *
 * @param[in] sampleRate
 *    The desired sample rate in Hz
 *
 * @return
 *    The actual sample rate. May be different from the desired value because
 *    of the finite and discrete number of divider settings
 ******************************************************************************/
float ICM20649_gyroSampleRateSet(float sampleRate)
{
  uint8_t gyroDiv;
  float gyroSampleRate;

  /* Calculate the sample rate divider */
  gyroSampleRate = (1125.0 / sampleRate) - 1.0;

  /* Check if it fits in the divider register */
  if ( gyroSampleRate > 255.0 ) {
    gyroSampleRate = 255.0;
  }

  if ( gyroSampleRate < 0 ) {
    gyroSampleRate = 0.0;
  }

  /* Write the value to the register */
  gyroDiv = (uint8_t) gyroSampleRate;
  ICM20649_registerWrite(ICM20649_REG_GYRO_SMPLRT_DIV, gyroDiv);

  /* Calculate the actual sample rate from the divider value */
  gyroSampleRate = 1125.0 / (gyroDiv + 1);

  return gyroSampleRate;
}


/***************************************************************************//**
 * @brief
 *    Sets the bandwidth of the gyroscope
 *
 * @param[in] gyroBw
 *    The desired bandwidth value. Use the ICM20649_GYRO_BW_xHZ macros, which
 *    are defined in the ICM20649.h file. The value of x can be
 *    6, 12, 24, 51, 120, 150, 200, 360 or 12100.
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20649_gyroBandwidthSet(uint8_t gyroBw)
{
  uint8_t reg;

  /* Read the GYRO_CONFIG_1 register */
  ICM20649_registerRead(ICM20649_REG_GYRO_CONFIG_1, 1, &reg);
  reg &= ~(ICM20649_MASK_GYRO_BW);

  /* Write the new bandwidth value to the gyro config register */
  reg |= (gyroBw & ICM20649_MASK_GYRO_BW);
  ICM20649_registerWrite(ICM20649_REG_GYRO_CONFIG_1, reg);

  return ICM20649_OK;
}


/***************************************************************************//**
 * @brief
 *    Enables or disables the sleep mode of the device
 *
 * @param[in] enable
 *    If true, sleep mode is enabled. Set to false to disable sleep mode.
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20649_sleepModeEnable(bool enable)
{
  uint8_t reg;

  ICM20649_registerRead(ICM20649_REG_PWR_MGMT_1, 1, &reg);

  if ( enable ) {
    /* Sleep: set the SLEEP bit */
    reg |= ICM20649_BIT_SLEEP;
  } else {
    /* Wake up: clear the SLEEP bit */
    reg &= ~(ICM20649_BIT_SLEEP);
  }

  ICM20649_registerWrite(ICM20649_REG_PWR_MGMT_1, reg);

  return ICM20649_OK;
}

/***************************************************************************//**
 * @brief
 *    Enables or disables the cycle mode operation of the accel and gyro
 *
 * @param[in] enable
 *    If true both the accel and gyro sensors will operate in cycle mode. If
 *    false the senors working in continuous mode.
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20649_cycleModeEnable(bool enable)
{
  uint8_t reg;

  reg = 0x00;

  if ( enable ) {
    reg = ICM20649_BIT_GYRO_CYCLE;
  }

  ICM20649_registerWrite(ICM20649_REG_LP_CONFIG, reg);

  return ICM20649_OK;
}

/***************************************************************************//**
 * @brief
 *    Enables or disables the sensors in the ICM20649 chip
 *
 * @param[in] accel
 *    If true enables the acceleration sensor
 *
 * @param[in] gyro
 *    If true enables the gyroscope sensor
 *
 * @param[in] temp
 *    If true enables the temperature sensor
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20649_sensorEnable(bool accel, bool gyro, bool temp)
{
  uint8_t pwrManagement1;
  uint8_t pwrManagement2;

  ICM20649_registerRead(ICM20649_REG_PWR_MGMT_1, 1, &pwrManagement1);
  pwrManagement2 = 0;

  /* To enable the accelerometer clear the DISABLE_ACCEL bits in PWR_MGMT_2 */
  if ( accel ) {
    pwrManagement2 &= ~(ICM20649_BIT_PWR_ACCEL_STBY);
  } else {
    pwrManagement2 |= ICM20649_BIT_PWR_ACCEL_STBY;
  }

  /* To enable gyro clear the DISABLE_GYRO bits in PWR_MGMT_2 */
  if ( gyro ) {
    pwrManagement2 &= ~(ICM20649_BIT_PWR_GYRO_STBY);
  } else {
    pwrManagement2 |= ICM20649_BIT_PWR_GYRO_STBY;
  }

  /* To enable the temperature sensor clear the TEMP_DIS bit in PWR_MGMT_1 */
  if ( temp ) {
    pwrManagement1 &= ~(ICM20649_BIT_TEMP_DIS);
  } else {
    pwrManagement1 |= ICM20649_BIT_TEMP_DIS;
  }

  /* Write back the modified values */
  ICM20649_registerWrite(ICM20649_REG_PWR_MGMT_1, pwrManagement1);
  ICM20649_registerWrite(ICM20649_REG_PWR_MGMT_2, pwrManagement2);

  return ICM20649_OK;
}

/***************************************************************************//**
 * @brief
 *    Enables or disables the sensors in low power mode in the ICM20649 chip
 *
 * @param[in] enAccel
 *    If true enables the acceleration sensor in low power mode
 *
 * @param[in] enGyro
 *    If true enables the gyroscope sensor in low power mode
 *
 * @param[in] enTemp
 *    If true enables the temperature sensor in low power mode
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20649_lowPowerModeEnter(bool enAccel, bool enGyro, bool enTemp)
{
  uint8_t data;

  ICM20649_registerRead(ICM20649_REG_PWR_MGMT_1, 1, &data);

  if ( enAccel || enGyro || enTemp ) {
    /* Make sure that the chip is not in sleep */
    ICM20649_sleepModeEnable(false);

    /* And in continuous mode */
    ICM20649_cycleModeEnable(false);

    /* Enable the accelerometer and the gyroscope*/
    ICM20649_sensorEnable(enAccel, enGyro, enTemp);
    UTIL_delay(50);

    /* Enable cycle mode */
    ICM20649_cycleModeEnable(true);

    /* Set the LP_EN bit to enable low power mode */
    data |= ICM20649_BIT_LP_EN;
  } else {
    /* Enable continuous mode */
    ICM20649_cycleModeEnable(false);

    /* Clear the LP_EN bit to disable low power mode */
    data &= ~ICM20649_BIT_LP_EN;
  }

  /* Write the updated value to the PWR_MGNT_1 register */
  ICM20649_registerWrite(ICM20649_REG_PWR_MGMT_1, data);

  return ICM20649_OK;
}

/***************************************************************************//**
 * @brief
 *    Enables or disables the interrupts in the ICM20649 chip
 *
 * @param[in] dataReadyEnable
 *    If true enables the Raw Data Ready interrupt, otherwise disables.
 *
 * @param[in] womEnable
 *    If true enables the Wake-up On Motion interrupt, otherwise disables.
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20649_interruptEnable(bool dataReadyEnable, bool womEnable)
{
  uint8_t intEnable;

  /* All interrupts disabled by default */
  intEnable = 0;

  /* Enable one or both of the interrupt sources if required */
  if ( womEnable ) {
    intEnable = ICM20649_BIT_WOM_INT_EN;
  }
  /* Write value to register */
  ICM20649_registerWrite(ICM20649_REG_INT_ENABLE, intEnable);

  /* All interrupts disabled by default */
  intEnable = 0;

  if ( dataReadyEnable ) {
    intEnable = ICM20649_BIT_RAW_DATA_0_RDY_EN;
  }

  /* Write value to register */
  ICM20649_registerWrite(ICM20649_REG_INT_ENABLE_1, intEnable);

  return ICM20649_OK;
}

/***************************************************************************//**
 * @brief
 *    Reads the interrupt status registers of the ICM20649 chip
 *
 * @param[out] intStatus
 *    The content the four interrupt registers. LSByte is INT_STATUS, MSByte is
 *    INT_STATUS_3
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20649_interruptStatusRead(uint32_t *intStatus)
{
  uint8_t reg[4];

  ICM20649_registerRead(ICM20649_REG_INT_STATUS, 4, reg);
  *intStatus = (uint32_t) reg[0];
  *intStatus |= ( ( (uint32_t) reg[1]) << 8);
  *intStatus |= ( ( (uint32_t) reg[2]) << 16);
  *intStatus |= ( ( (uint32_t) reg[3]) << 24);

  return ICM20649_OK;
}

/***************************************************************************//**
 * @brief
 *    Checks if new data is available for read
 *
 * @return
 *    Returns true if the Raw Data Ready interrupt bit set, false otherwise
 ******************************************************************************/
bool ICM20649_isDataReady(void)
{
  uint8_t status;
  bool ret;

  ret = false;
  ICM20649_registerRead(ICM20649_REG_INT_STATUS_1, 1, &status);

  if ( status & ICM20649_BIT_RAW_DATA_0_RDY_INT ) {
    ret = true;
  }

  return ret;
}

/***************************************************************************//**
 * @brief
 *    Gyroscope calibration function. Reads the gyroscope
 *    values while the device is at rest and in level. The
 *    resulting values are loaded to the gyro bias registers to cancel
 *    the static offset error.
 *
 * @param[out] gyroBiasScaled
 *    The mesured gyro sensor bias in deg/sec
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20649_gyroCalibrate(float *gyroBiasScaled)
{
  uint8_t data[12];
  uint16_t i, packetCount, fifoCount;
  int32_t gyroBias[1] = { 0 };
  int32_t gyroTemp[1];
  int32_t gyroBiasStored[1];
  float gyroRes;

  /* Enable the gyro */
  ICM20649_sensorEnable(false, true, false);

  /* Set 1kHz sample rate */
  ICM20649_sampleRateSet(1100.0);

  /* Configure bandwidth for gyroscope to 12Hz */
  ICM20649_gyroBandwidthSet(ICM20649_GYRO_BW_12HZ);

  /* Configure sensitivity to 250dps full scale */
  ICM20649_gyroFullscaleSet(ICM20649_GYRO_FULLSCALE_500DPS);

  /* Retrieve the resolution per bit */
  ICM20649_gyroResolutionGet(&gyroRes);

  /* The accel sensor needs max 30ms, the gyro max 35ms to fully start */
  /* Experiments show that the gyro needs more time to get reliable results */
  UTIL_delay(50);

  /* Disable the FIFO */
  ICM20649_registerWrite(ICM20649_REG_USER_CTRL, ICM20649_BIT_FIFO_EN);
  ICM20649_registerWrite(ICM20649_REG_FIFO_MODE, 0x0F);

  /* Enable gyro to store the data in FIFO */
  ICM20649_registerWrite(ICM20649_REG_FIFO_EN_2, ICM20649_BITS_GYRO_FIFO_EN);

  /* Reset the FIFO */
  ICM20649_registerWrite(ICM20649_REG_FIFO_RST, 0x0F);
  ICM20649_registerWrite(ICM20649_REG_FIFO_RST, 0x00);

  /* Enable the FIFO */
  ICM20649_registerWrite(ICM20649_REG_USER_CTRL, ICM20649_BIT_FIFO_EN);

  /* The max FIFO size is 4096 bytes, one set of measurements takes 12 bytes */
  /* (3 axes, 2 sensors, 2 bytes each value ) 340 samples use 4080 bytes of FIFO */
  /* Loop until at least 4080 samples gathered */
  fifoCount = 0;
  while ( fifoCount < 4080 ) {
    UTIL_delay(5);

    /* Read FIFO sample count */
    ICM20649_registerRead(ICM20649_REG_FIFO_COUNT_H, 2, &data[0]);

    /* Convert to a 16 bit value */
    fifoCount = ( (uint16_t) (data[0] << 8) | data[1]);
  }

  /* Disable accelerometer and gyro to store the data in FIFO */
  ICM20649_registerWrite(ICM20649_REG_FIFO_EN_2, 0x00);

  /* Read FIFO sample count */
  ICM20649_registerRead(ICM20649_REG_FIFO_COUNT_H, 2, &data[0]);

  /* Convert to a 16 bit value */
  fifoCount = ( (uint16_t) (data[0] << 8) | data[1]);

  /* Calculate the number of data sets (3 axis of accel an gyro, two bytes each = 12 bytes) */
  packetCount = fifoCount / 12;

  /* Retrieve the data from the FIFO */
  for ( i = 0; i < packetCount; i++ ) {
    ICM20649_registerRead(ICM20649_REG_FIFO_R_W, 12, &data[0]);
    /* Convert to 16 bit gyro x values */
    gyroTemp[0] = ( (int16_t) (data[6] << 8) | data[7]);

    /* Sum the values */
    gyroBias[0] += gyroTemp[0];
  }

  /* Divide by packet count to get the average */
  gyroBias[0] /= packetCount;

  /* Convert the values to degrees per sec for displaying */
  gyroBiasScaled[0] = (float) gyroBias[0] * gyroRes;

  /* Read stored gyro trim values. After reset these values are all 0 */
  ICM20649_registerRead(ICM20649_REG_XG_OFFS_USRH, 2, &data[0]);
  gyroBiasStored[0] = ( (int16_t) (data[0] << 8) | data[1]);

  /* The gyro bias should be stored in 1000dps full scaled format. We measured in 500dps to get */
  /* the best sensitivity, so need to divide by 2 */
  /* Substract from the stored calibration value */
  gyroBiasStored[0] -= gyroBias[0] / 2;


  /* Split the values into two bytes */
  data[0] = (gyroBiasStored[0] >> 8) & 0xFF;
  data[1] = (gyroBiasStored[0]) & 0xFF;

  /* Write the  gyro bias values to the chip */
  ICM20649_registerWrite(ICM20649_REG_XG_OFFS_USRH, data[0]);
  ICM20649_registerWrite(ICM20649_REG_XG_OFFS_USRL, data[1]);

  /* Turn off FIFO */
  ICM20649_registerWrite(ICM20649_REG_USER_CTRL, 0x00);

  /* Disable all sensors */
  ICM20649_sensorEnable(false, false, false);

  return ICM20649_OK;
}

/***************************************************************************//**
 * @brief
 *    Reads the device ID of the ICM20649
 *
 * @param[out] devID
 *    The ID of the device read from teh WHO_AM_I register. Expected value? 0xE0
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20649_getDeviceID(uint8_t *devID)
{
  ICM20649_registerRead(ICM20649_REG_WHO_AM_I, 1, devID);

  return ICM20649_OK;
}

/***************************************************************************//**
 * @brief
 *    Initializes I2C pins in order to communicate with the ICM20649
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20649_i2cInit(void)
{
  /* IO configuration */
  /* GPIO_PinModeSet(sdaPin, gpioModePushPull, 0);     SDA set
  GPIO_PinModeSet(sclPin, gpioModePushPull, 0);     */

  return ICM20649_OK;
}

/***************************************************************************//**
 * @brief
 *    Reads register from the ICM20649 device
 *
 * @param[in] addr
 *    The register address to read from in the sensor
 *    Bit[8:7] - bank address
 *    Bit[6:0] - register address
 *
 * @param[in] numBytes
 *    The number of bytes to read
 *
 * @param[out] data
 *    The data read from the register
 *
 * @return
 *    None
 ******************************************************************************/
void ICM20649_registerRead(uint16_t addr, int numBytes, uint8_t *data)
{
  uint8_t regAddr;
  uint8_t bank;

  regAddr = (uint8_t) (addr & 0x7F);
  bank = (uint8_t) (addr >> 7);

  ICM20649_bankSelect(bank);

  /* Enable chip select */
  ICM20649_chipSelectSet(true);

  /* Set R/W bit to 1 - read */
  USART_Tx(ICM20649_SPI_USART, (regAddr | 0x80) );
  USART_Rx(ICM20649_SPI_USART);
  /* Transmit 0's to provide clock and read the data */
  while ( numBytes-- ) {
    USART_Tx(ICM20649_SPI_USART, 0x00);
    *data++ = USART_Rx(ICM20649_SPI_USART);
  }

  /* Disable chip select */
  ICM20649_chipSelectSet(false);

  return;
}

/***************************************************************************//**
 * @brief
 *    Writes a register in the ICM20649 device
 *
 * @param[in] addr
 *    The register address to write
 *    Bit[8:7] - bank address
 *    Bit[6:0] - register address
 *
 * @param[in] data
 *    The data to write to the register
 *
 * @return
 *    None
 ******************************************************************************/
void ICM20649_registerWrite(uint16_t addr, uint8_t data)
{
  uint8_t regAddr;
  uint8_t bank;

  regAddr = (uint8_t) (addr & 0x7F);
  bank = (uint8_t) (addr >> 7);

  ICM20649_bankSelect(bank);

  /* Enable chip select */
  ICM20649_chipSelectSet(true);

  /* clear R/W bit - write, send the address */
  USART_Tx(ICM20649_SPI_USART, (regAddr & 0x7F) );
  USART_Rx(ICM20649_SPI_USART);

  /* Send the data */
  USART_Tx(ICM20649_SPI_USART, data);
  USART_Rx(ICM20649_SPI_USART);

  /* Disable chip select */
  ICM20649_chipSelectSet(false);

  return;
}

/***************************************************************************//**
 * @brief
 *    Select the desired register bank
 *
 * @param[in] bank
 *    The address of the register bank (0..3)
 *
 * @return
 *    None
 ******************************************************************************/
void ICM20649_bankSelect(uint8_t bank)
{
  /* Enable chip select */
  ICM20649_chipSelectSet(true);

  /* Select the Bank Select register */
  USART_Tx(ICM20649_SPI_USART, ICM20649_REG_BANK_SEL);
  USART_Rx(ICM20649_SPI_USART);

  /* Write the desired bank address 0..3 */
  USART_Tx(ICM20649_SPI_USART, (bank << 4) );
  USART_Rx(ICM20649_SPI_USART);

  /* Disable chip select */
  ICM20649_chipSelectSet(false);

  return;
}

/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */

/***************************************************************************//**
 * @brief
 *    Controls the state of the ICM20649 SPI chip select pin
 *
 * @param[in] select
 *    The desired state of the select pin. True: asserted (logic L)
 *
 * @return
 *    None
 ******************************************************************************/
static void ICM20649_chipSelectSet(bool select)
{
  if ( select ) {
    GPIO_PinOutClear(sclPin, sdaPin);
  } else {
    GPIO_PinOutSet(sclPin, sdaPin);
  }

  return;
}

/** @endcond */

/** @} (end defgroup ICM20649) */
/** @} {end addtogroup TBSense_BSP} */
