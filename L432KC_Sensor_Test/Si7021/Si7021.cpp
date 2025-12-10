/*
 * Si7021.cpp
 *
 *  Created on: Feb 15, 2025
 *      Author: coryg
 */

#include <Si7021.h>
#include <cstring>

Si7021::Si7021(I2C_HandleTypeDef *hi2c) {
	// TODO Auto-generated constructor stub
	_hi2c = hi2c;
	_DevAddress = SI7021_I2C_ADDR<<1;
}

Si7021::~Si7021() {
	// TODO Auto-generated destructor stub
	delete this;
}

void Si7021::startMeasurement(){
	uint8_t buf[1];
	buf[0] = Measure_RH_No_Hold;
	// send measurement command
	HAL_I2C_Master_Transmit(this->_hi2c,this->_DevAddress,buf,1,HAL_MAX_DELAY);
}

/**
 * Performs a CRC8 calculation on the supplied values.
 *
 * @param data  Pointer to the data to use when calculating the CRC8.
 * @param len   The number of bytes in 'data'.
 *
 * @return The computed CRC8 value.
 */
static uint8_t crc8(const uint8_t *data, int len) {
  /*
   *
   * CRC-8 formula from page 14 of SHT spec pdf
   *
   * Test data 0xBE, 0xEF should yield 0x92
   *
   * Initialization data 0xFF
   * Polynomial 0x31 (x8 + x5 +x4 +1)
   * Final XOR 0x00
   */

  const uint8_t POLYNOMIAL(0x31);
  uint8_t crc(0xFF);

  for (int j = len; j; --j) {
    crc ^= *data++;

    for (int i = 8; i; --i) {
      crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
    }
  }
  return crc;
}

bool Si7021::readHumidity(float *h){
	uint8_t buf[3];
	uint16_t hm;

	// read humidity measurement
	HAL_I2C_Master_Receive(this->_hi2c,this->_DevAddress,buf,3,HAL_MAX_DELAY);

	if(crc8(buf,2)!=buf[2]) {
			//sprintf((char*)error,"Did not pass CRC check");
			//  HAL_UART_Transmit(&huart2,error,strlen((char*)error),HAL_MAX_DELAY);
			//  HAL_Delay(100);
				  return false; // check crc

		}
	hm = buf[0];
	hm<<=8;
	hm|= buf[1];

	*h = (float(hm)*125)/65536-6;
	return true;
}

bool Si7021::readTemp(float *t){
	uint8_t buf[3];
	buf[0] = Read_T_From_RH;
	// send measurement command
	HAL_I2C_Master_Transmit(this->_hi2c,this->_DevAddress,buf,1,HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(this->_hi2c,this->_DevAddress,buf,2,HAL_MAX_DELAY);
  // no CRC8 available
	*t = 0;
	return true;
}

void Si7021::readRegister(){

}

void Si7021::main(){
switch (state){
case Si7021_init_state:
	if(checkDevice()){
		state=Si7021_reset_state;
	} else {
		state=Si7021_err_state;
	}
	break;
case Si7021_reset_state:
	if(HAL_GetTick()>(last_update+100)){
		state=Si7021_initializing;
	}
	break;
case Si7021_done_state:

	break;
case Si7021_err_state:

	break;
}

}

bool Si7021::checkDevice(){
	I2CStatus = HAL_I2C_IsDeviceReady(_hi2c,_DevAddress,1,HAL_MAX_DELAY);
	return (I2CStatus == HAL_OK);
}

void Si7021::readUserRegister(){
	uint8_t buf[3];
	buf[0] = Read_RHT_User_Register_1;
	// send measurement command
	HAL_I2C_Master_Transmit(this->_hi2c,this->_DevAddress,buf,1,HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(this->_hi2c,this->_DevAddress,buf,1,HAL_MAX_DELAY);
}

void Si7021::writeUserRegister(){

}

