/*
 * mpl3115a2.cpp
 *
 *  Created on: Dec 29, 2025
 *      Author: coryg
 */

#include "mpl3115a2.h"
#include <cstring>
#include <stdio.h>

namespace mpl3115a2 {

mpl3115a2::mpl3115a2(mpl3115a2_param_t param) {
	// TODO Auto-generated constructor stub
	memcpy(&this->_param,&param,sizeof(mpl3115a2_param_t));
	_state=mpl3115a2_init_state;
}

mpl3115a2::~mpl3115a2() {
	// TODO Auto-generated destructor stub
}

void mpl3115a2::main(){
	switch(_state){
	case mpl3115a2_init_state:
		break;
	case mpl3115a2_error_state:
		break;
	default:
		break;
	}
}

bool mpl3115a2::checkDevice(){
	if(HAL_OK==HAL_I2C_IsDeviceReady(_param._hi2c,_param._DevAddress,1,HAL_MAX_DELAY)){
		return true;
	}
	return false;
}


bool mpl3115a2::_readReg8(mpl3115a2_reg_t *reg){
	uint8_t buf[1];
	buf[0]=reg->addr;
	HAL_I2C_Master_Transmit(_param._hi2c,_param._DevAddress,buf,1,HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(_param._hi2c,_param._DevAddress,buf,1,HAL_MAX_DELAY);
	reg->reg_value=buf[0];
	return true;
}

bool mpl3115a2::who_am_i(){
	mpl3115a2_reg_t whoami = {mpl3115a2_who_am_i_addr,0};
	_readReg8(&whoami);
	if(whoami.reg_value==mpl3115a2_who_am_i) return true;
	return false;
}



} /* namespace mpl3115a2 */
