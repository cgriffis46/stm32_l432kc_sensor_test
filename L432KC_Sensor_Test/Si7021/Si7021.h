/*
 * Si7021.h
 *
 *  Created on: Feb 15, 2025
 *      Author: coryg
 */

#ifndef SI7021_H_
#define SI7021_H_

#include <stm32l4xx_hal.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#define SI7021_I2C_ADDR 0x40

typedef enum Si7021_Cmd_t {
	Measure_RH_Hold = 0xE5,
	Measure_RH_No_Hold = 0xF5,
	Measure_T_Hold = 0xE3,
	Measure_T_No_Hold = 0xF3,
	Read_T_From_RH = 0xE0,
	Reset = 0xFE,
	Write_RHT_User_Register_1 = 0xE6,
	Read_RHT_User_Register_1 = 0xE7,
	Write_Heater_Control_Reg = 0x51,
	Read_Heater_Control_Reg = 0x11
}Si7021_Cmd_t;

typedef bool Si7021_heater_enable;

typedef enum Si7021_state_t{
	Si7021_init_state,
	Si7021_reset_state,
	Si7021_done_state,
	Si7021_err_state
}Si7021_state_t;

typedef enum Si7021_reg_addr_t{

}Si7021_reg_addr_t;

struct Si7021_reg_t {
	Si7021_reg_addr_t reg_addr;
};

class Si7021 {
public:
	Si7021(I2C_HandleTypeDef *hi2c);
	virtual ~Si7021();
	void readTempHumidity(float *t, float *h);
	void readRegister();
	void startMeasurement();
	void main();
	void readUserRegister();
	void writeUserRegister();
	bool checkDevice();
private:
	I2C_HandleTypeDef *_hi2c;
	uint16_t _DevAddress;
	Si7021_state_t state;
	Si7021_heater_enable enable_heater;
	float temp,hudity;
	bool readTemp(float *t);
	bool readHumidity(float *h);
};

#endif /* SI7021_H_ */
