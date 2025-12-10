/*
 * HTU21DF.cpp
 *
 *  Created on: Mar 20, 2024
 *      Author: coryg
 */

#include "../../htu21df/HTU21DF.h"

#include "stm32l4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

/*!
 * @file HTU21DF.cpp
 *
 * @mainpage Adafruit HTU21DF Sensor
 *
 * @section intro_sec Introduction
 *
 * This is a library for the HTU21DF Humidity & Temp Sensor
 *
 * Designed specifically to work with the HTU21DF sensor from Adafruit
 * ----> https://www.adafruit.com/products/1899
 *
 * These displays use I2C to communicate, 2 pins are required to
 * interface
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section author Author
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text above must be included in any redistribution
 */

/**
 * Constructor for the HTU21DF driver.
 */
HTU21DF::HTU21DF(I2C_HandleTypeDef* _hi2c, uint8_t _devAddress) {

    this->_hi2c = _hi2c;
    this->state = htu21df_init_state;
    this->_devAddress = _devAddress<<1;
  /* Assign default values to internal tracking variables. */
  _last_humidity = 0.0f;
  _last_temp = 0.0f;

}

/**
 * Initialises the I2C transport, and configures the IC for normal operation.
 * @param theWire Pointer to TwoWire I2C object, uses &Wire by default
 * @return true (1) if the device was successfully initialised, otherwise
 *         false (0).
 */
/*
bool HTU21DF::begin(uint8_t _devAddress = HTU21DF_I2CADDR) {
  uint8_t readbuffer;
  uint8_t cmd = HTU21DF_READREG;

  reset();

  HAL_I2C_Master_Transmit(this->_hi2c,this->_devAddress,&cmd,1,HAL_MAX_DELAY);
  HAL_I2C_Master_Receive(this->_hi2c,this->_devAddress,&readbuffer,1,HAL_MAX_DELAY);

  return (readbuffer == 0x2);
}
*/

/**
 * Sends a 'reset' request to the HTU21DF, followed by a 15ms delay.
 */
void HTU21DF::reset(void) {
  uint8_t cmd = HTU21DF_RESET;
  HAL_I2C_Master_Transmit(this->_hi2c,this->_devAddress,&cmd,1,HAL_MAX_DELAY);
}


void HTU21DF::_startTempMeasurement(){
	uint8_t cmd;
	cmd = HTU21DF_READTEMP;
	HAL_I2C_Master_Transmit(this->_hi2c,this->_devAddress,&cmd,1,HAL_MAX_DELAY);
}

void HTU21DF::_startHumidityMeasurement(){
	uint8_t cmd;
	cmd = HTU21DF_READHUM;
	HAL_I2C_Master_Transmit(this->_hi2c,this->_devAddress,&cmd,1,HAL_MAX_DELAY);
}

void HTU21DF::startTempMeasurement(){

}

void HTU21DF::startHumidityMeasurement(){

}

/**
 *
 * Performs a single temperature conversion in degrees Celsius.
 *
 * @return a single-precision (32-bit) float value indicating the measured
 *         temperature in degrees Celsius or NAN on failure.
 *
 */
bool HTU21DF::readTemperature(float *temperature) {
  //OK lets ready!
  //uint8_t cmd = HTU21DF_READTEMP;
  //if(!HAL_I2C_Master_Transmit(this->_hi2c,this->_devAddress,&cmd,1,HAL_MAX_DELAY)==HAL_OK) return NAN;

//  HAL_Delay(50);// add delay between request and actual read!

  uint8_t buf[3];

  if(!HAL_I2C_Master_Receive(this->_hi2c,this->_devAddress,buf,3,HAL_MAX_DELAY)==HAL_OK) return false;

  /* Read 16 bits of data, dropping the last two status bits. */
  uint16_t t = buf[0];
  t <<= 8;
  t |= buf[1] & 0b11111100;

  // 3rd byte is the CRC

  float temp = t;
  temp *= 175.72f;
  temp /= 65536.0f;
  temp -= 46.85f;

  /* Track the value internally in case we need to access it later. */
  _last_temp = temp;

  *temperature = temp;
  return true;
}

/**
 * Performs a single relative humidity conversion.
 *
 * @return A single-precision (32-bit) float value indicating the relative
 *         humidity in percent (0..100.0%).
 */
bool HTU21DF::readHumidity(float *humidity) {
  /* Prepare the I2C request. */
  //uint8_t cmd = HTU21DF_READHUM;

  //HAL_I2C_Master_Transmit(this->_hi2c,this->_devAddress,&cmd,1,HAL_MAX_DELAY);
  /* Wait a bit for the conversion to complete. */
  //sleep_us(50000);
  //HAL_Delay(50);

  uint8_t buf[3];

  if(!HAL_I2C_Master_Receive(this->_hi2c,this->_devAddress,buf,3,HAL_MAX_DELAY)==HAL_OK) return false;

  /* Read 16 bits of data, dropping the last two status bits. */
  uint16_t h = buf[0];
  h <<= 8;
  h |= buf[1] & 0b11111100;

  // 3rd byte is the CRC

  float hum = h;
  hum *= 125.0f;
  hum /= 65536.0f;
  hum -= 6.0f;

  /* Track the value internally in case we need to access it later. */
  _last_humidity = hum;

  *humidity=hum;
  return true;
}

void HTU21DF::main(){
	switch(state){
	case htu21df_init_state:
		if(checkDevice()){ // check if device exists
			reset(); // if device exists send reset.
			last_update=HAL_GetTick();
			state=htu21df_reset_state;
		} else{
			state=htu21df_error_state;
		}
		break;
	case htu21df_reset_state:
		if(HAL_GetTick()>(last_update+15)){ // wait 15ms for soft reset.

			last_update=HAL_GetTick();
			state=htu21df_sleep_state;
		}
		break;
	case htu21df_sleep_state:
		if(HAL_GetTick()>(last_update+1000)){
			state=htu21df_measure_temp_state;
			last_update=HAL_GetTick();
		}
		break;
	case htu21df_measure_temp_state:
		startTempMeasurement();
		last_update=HAL_GetTick();
		state=htu21df_read_temp_state;
		break;
	case htu21df_measure_humidity_state:
		startHumidityMeasurement();
		last_update=HAL_GetTick();
		state=htu21df_read_humidity_state;
		break;
	case htu21df_read_temp_state:
		if(HAL_GetTick()>(last_update+50)){
			last_update=HAL_GetTick();
			state=htu21df_measure_humidity_state;
		}
		break;
	case htu21df_read_humidity_state:
		if(HAL_GetTick()>(last_update+50)){
			last_update=HAL_GetTick();
			state=htu21df_sleep_state;
		}
		break;
	case htu21df_error_state:
		if(HAL_GetTick()>(last_update+100)){
			last_update=HAL_GetTick();
			state=htu21df_init_state;
		}
		break;
	default:
		state=htu21df_init_state;
		break;
	}
}

bool HTU21DF::checkDevice(){
	HAL_StatusTypeDef I2CStatus = HAL_I2C_IsDeviceReady(_hi2c,_devAddress,1,HAL_MAX_DELAY);
	return (HAL_OK==I2CStatus);
}




