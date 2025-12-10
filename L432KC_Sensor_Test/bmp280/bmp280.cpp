/*
 * bmp280.cpp
 *
 *  Created on: Aug 9, 2025
 *      Author: coryg
 */

#include "bmp280.h"
#include <stm32l4xx_hal.h>

extern UART_HandleTypeDef huart2;

namespace BMP280 {

uint8_t charBuffer[255];

bmp280::bmp280() {
	// TODO Auto-generated constructor stub
	bmp280_state=bmp280_init;
}

bmp280::~bmp280() {
	// TODO Auto-generated destructor stub
	delete this;
}

bool bmp280::init(){ return false;}

bool bmp280i2c::init(){
	HAL_StatusTypeDef I2CStatus;
	// check if device exists
	I2CStatus = HAL_I2C_IsDeviceReady(this->_hi2c,this->_DevAddress,1,HAL_MAX_DELAY);
	return (I2CStatus==HAL_OK);
}

void bmp280::main(){
	switch (bmp280_state){
		case bmp280_init:
			reset();
			_newData=false;
			last_update=HAL_GetTick(); // get system time &  wait 100ms before continuing
			bmp280_state = bmp280_reset;
			break;
		case bmp280_reset:
			// wait 100ms after issuing Rest
			if(HAL_GetTick()>(last_update+100)){
				bmp280_state=bmp280_initializing;
			}
			break;
		case bmp280_initializing:
			if(readIDReg()) {
				readCal();
				osrs_p_t = bmp280_osrs_p_1x;
				osrs_t_t = bmp280_osrs_t_1x;
				mode_t = bmp280_forced_mode;
				setConfigReg(bmp280_sb_500,bmp280_filter_off,bmp280_spi_en_off);
				setCtrlMeasReg(osrs_t_t,osrs_p_t,mode_t);
				last_update=HAL_GetTick();
				bmp280_state=bmp280_sleeping;
			} else{
           	    bmp280_state = bmp280_error;
			}
			break;
		case bmp280_start_measurement:
			if(startMeasurement()){
				last_update=HAL_GetTick();
				bmp280_state=bmp280_measuring;
			} else {
			}
			break;
		case bmp280_measuring:
			if(HAL_GetTick()>(last_update+100)){
				if(readTempPressure(&_temp,&_pressure)){
					last_update=HAL_GetTick();
					bmp280_state=bmp280_sleeping;}
			}
			break;
		case bmp280_sleeping:
			if(HAL_GetTick()>(last_update+100)){
					bmp280_state=bmp280_start_measurement;
				}
			break;
		case bmp280_error:
			if(HAL_GetTick()>(last_update+100)){
						bmp280_state=bmp280_init;
					}
			break;
		default: // if we don't know what state we're in, re-init
			bmp280_state=bmp280_init;
			break;
	}
}

/*
 * virtual functions overridden in bmp280i2c and bmp280spi child classes
 * */
uint8_t bmp280::readReg8(bmp280_reg8_t *bmp280_reg_addr){
	return 0;
}

uint8_t bmp280::writeReg8(bmp280_reg8_t *bmp280_reg_addr){
	return 0;
}

void bmp280::readRegMulti(bmp280_reg_addr_t addr_t,uint8_t *buf,uint8_t numRegs){
}

void bmp280::writeRegMulti(){
}

void bmp280::reset(){
	bmp280_reg8_t bmp280_reset_reg = {bmp280_reg_reset_addr,bmp280_reset_value};
	writeReg8(&bmp280_reset_reg);
}

/* Taken from BMP280 Datasheet */
double bmp280::bmp280_compensate_T_double(BMP280_S32_t adc_T){
	double var1, var2, T;
	var1 = (((double)adc_T)/16384.0 - ((double)dig_T1)/1024.0) *((double)dig_T2);
	var2 = ((((double)adc_T)/131072.0 - ((double)dig_T1)/8192.0)*(((double)adc_T)/13172.0 - ((double)dig_T1)/8192.0))*((double)dig_T3);
	t_fine = (BMP280_S32_t)(var1+var2);
	T=(var1+var2)/5120.0;
	return T;
}

/* Taken from BMP280 Datasheet */
double bmp280::bmp280_compensate_P_double(BMP280_S32_t adc_P){
	double var1, var2, P;
	var1 = ((double)t_fine/2.0)-64000.0;
	var2 = var1*var1*((double)dig_P6)/32768.0;
	var2 = var2+var1 * ((double)dig_P5)*2.0;
	var2 = (var2/4.0)+(((double)dig_P4)*65536.0);
	var1 = (((double)dig_P3)*var1*var1/524288.0+((double)dig_P2)*var1)/524288.0;
	var1 = (1.0+var1/32768.0)*((double)dig_P1);
	if(var1 == 0.0){
		return 0;
	}
	P=1048576.0 - (double)adc_P;
	P=(P-(var2/4096.0))*6250.0/var1;
	var1 = ((double)dig_P9)*P*P/2147483648.0;
	return P;
}

bool bmp280::readIDReg(){
	bmp280_reg8_t bmp280_id_reg = {bmp280_reg_id_addr,0};
	readReg8(&bmp280_id_reg);
	if(bmp280_id_reg.reg_value==bmp280_id){
		this->id.reg_value=bmp280_id_reg.reg_value;
		return true;
	}
	return false;
}

void bmp280::readConfigReg(){
	config.addr=bmp280_reg_config_addr;
	readReg8(&config);
}

void bmp280::readCtrlMeas(){
	ctrl_meas.addr=bmp280_reg_ctrl_meas_addr;
	readReg8(&ctrl_meas);
}

void bmp280::readStatusReg(){
	status.addr=bmp280_reg_status_addr;
	readReg8(&status);
}

void bmp280::readCal(){
	uint8_t calReg[24];
	readRegMulti(bmp280_reg_cal_addr,calReg,24);
	dig_T1=calReg[1]<<8;
	dig_T1|=calReg[0];
	dig_T2=calReg[3]<<8;
	dig_T2|=calReg[2];
	dig_T3=calReg[5]<<8;
	dig_T3|=calReg[4];

	dig_P1=calReg[7]<<8;
	dig_P1|=calReg[6];
	dig_P2=calReg[9]<<8;
	dig_P2|=calReg[8];
	dig_P3=calReg[11]<<8;
	dig_P3|=calReg[10];
	dig_P4=calReg[13]<<8;
	dig_P4|=calReg[12];
	dig_P5=calReg[15]<<8;
	dig_P5|=calReg[14];
	dig_P6=calReg[17]<<8;
	dig_P6|=calReg[16];
	dig_P7=calReg[19]<<8;
	dig_P7|=calReg[18];
	dig_P8=calReg[21]<<8;
	dig_P8|=calReg[20];
	dig_P9=calReg[23]<<8;
	dig_P9|=calReg[22];
}

void bmp280::setCtrlMeasReg(bmp280_osrs_t_t osrs_t_t, bmp280_osrs_p_t osrs_p_t, bmp280_mode_t mode_t){
	ctrl_meas.addr=bmp280_reg_ctrl_meas_addr;
	ctrl_meas.reg_value=(osrs_t_t|osrs_p_t|mode_t);
	writeReg8(&ctrl_meas);
}

void bmp280::setConfigReg(bmp280_sb_t sb_t, bmp280_filter_t filter_t, bmp280_spi_en spi_en){
	this->_sb_t=sb_t;this->_filter_t=filter_t;this->_spi_en=spi_en;
	config.addr=bmp280_reg_config_addr;
	config.reg_value=(sb_t|filter_t|spi_en);
	writeReg8(&config);
}

bool bmp280::startMeasurement(){
	readStatusReg();
	if(status.reg_value&&bmp_status_measuring){ // bmp80 is already measuring
		return false;
	}
	else {// start forced measurement
		mode_t=bmp280_forced_mode;
		setCtrlMeasReg(osrs_t_t, osrs_p_t, mode_t);
		return true;
	}
}

bool bmp280::readTempPressure(double *t, double *p){
	uint8_t buf[6];
	readStatusReg();
	if((status.reg_value&bmp_status_measuring)!=bmp_status_measuring){ // bmp80 is already measuring
		readRegMulti(bmp280_reg_press_msb_addr,buf,6); // burst read temp/pressure data

		this->adc_p = 0;
		this->adc_p = buf[0];
		this->adc_p=this->adc_p<<8;
		this->adc_p |= buf[1];
		this->adc_p=this->adc_p<<8;
		this->adc_p |= buf[2];
		this->adc_p=this->adc_p>>4;

		this->adc_t = 0;
		this->adc_t = buf[3];
		this->adc_t=this->adc_t<<8;
		this->adc_t |= buf[4];
		this->adc_t=this->adc_t<<8;
		this->adc_t |= buf[5];
		this->adc_t=this->adc_t>>4;

		*t = bmp280_compensate_T_double(this->adc_t);
		*p = bmp280_compensate_P_double(this->adc_p);

		_newData = true;

		return true;
	}
	return false;
}

bool bmp280::newData(){
	return _newData;
}


bool bmp280::GetTemperature(double *t){
	*t = _temp;
	return true;
}
bool bmp280::GetPressure(double *p){
	*p = _pressure;
	return true;
}

// I2C BMP280

// bmp280 constructor
bmp280i2c::bmp280i2c(I2C_HandleTypeDef *hi2c, bmp280_i2c_addr_t dev_addr){
	this->_DevAddress = dev_addr<<1;
	this->_hi2c = hi2c;
	bmp280_state=bmp280_init;
}

bmp280i2c::~bmp280i2c() {
	// TODO Auto-generated destructor stub
	delete this;
}


uint8_t bmp280i2c::readReg8(bmp280_reg8_t *bmp280_reg){
	uint8_t buf[1];
	buf[0]=bmp280_reg->addr;
	buf[1]=0;
	HAL_I2C_Master_Transmit(this->_hi2c,this->_DevAddress,buf,1,HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(this->_hi2c,this->_DevAddress,&buf[1],1,HAL_MAX_DELAY);
	bmp280_reg->reg_value=buf[1];
	return 0;
}

uint8_t bmp280i2c::writeReg8(bmp280_reg8_t *bmp280_reg){
	uint8_t buf[2];
	buf[0]=bmp280_reg->addr;
	buf[1]=bmp280_reg->reg_value;
	HAL_I2C_Master_Transmit(this->_hi2c,this->_DevAddress,buf,2,HAL_MAX_DELAY);
	return 0;
}

void bmp280i2c::readRegMulti(bmp280_reg_addr_t addr_t,uint8_t *buf,uint8_t numRegs){
	buf[0]=addr_t;
	HAL_I2C_Master_Transmit(this->_hi2c,this->_DevAddress,buf,1,HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(this->_hi2c,this->_DevAddress,&buf[0],numRegs,HAL_MAX_DELAY);
}

void bmp280i2c::writeRegMulti(){
}


} /* namespace BMP280 */




