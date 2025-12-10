/*
 * bmp280.h
 *
 *  Created on: Aug 9, 2025
 *      Author: coryg
 */

#ifndef BMP280_H_
#define BMP280_H_

#include <stdint.h>
#include <stm32l4xx_hal.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

namespace BMP280 {

#define bmp280_id 0x58

//i2c addresses
//#define bmp280_i2c_addr_1
//#define bmp280_i2c_addr_2 0x77

typedef enum bmp280_i2c_addr_t{
	bmp280_i2c_addr_1 = 0x76,
	bmp280_i2c_addr_2 = 0x77,
}bmp280_i2c_addr_t;

// defines for register addresses

typedef enum bmp280_reg_addr_t{
	bmp280_reg_id_addr = 0xD0,
	bmp280_reg_reset_addr = 0xE0,
	bmp280_reg_status_addr = 0xF3,
	bmp280_reg_ctrl_meas_addr = 0xF4,
	bmp280_reg_config_addr = 0xF5,
	bmp280_reg_press_msb_addr = 0xF7,
	bmp280_reg_press_lsb_addr = 0xF8,
	bmp280_reg_press_xlsb_addr = 0xF9,
	bmp280_reg_temp_msb_addr = 0xFA,
	bmp280_reg_temp_lsb_addr = 0xFB,
	bmp280_reg_temp_xlsb_addr = 0xFC,
	bmp280_reg_cal_addr = 0x88
}bmp280_reg_addr_t;

typedef uint8_t bmp280_reg8_val_t;
typedef long signed int BMP280_S32_t;
typedef long unsigned int BMP280_U32_t;
const bmp280_reg8_val_t bmp280_reset_value = 0xB6;

typedef struct bmp280_reg8_t{
	bmp280_reg_addr_t addr;
	bmp280_reg8_val_t reg_value;
}bmp280_reg8_t;

// defines for register masks
#define bmp280_t_sb_mask 0b11100000
#define bmp280_filter_mask 0b00011100
#define bmp_osrs_t_mask 0b11100000
#define bmp_osrs_p_mask 0b00011100
#define bmp_spi3w_en_mask 0b00000001
#define bmp_status_measuring 0b00001000
#define bmp_status_im_update 0b00000001

// pressure oversampling register settings
typedef enum bmp280_osrs_p_t{
 bmp280_osrs_p_skip = 0b00000000,
 bmp280_osrs_p_1x = 0b00000100,
 bmp280_osrs_p_2x = 0b00001000,
 bmp280_osrs_p_4x = 0b00001100,
 bmp280_osrs_p_8x = 0b00010000,
 bmp280_osrs_p_16x = 0b00010100
}bmp280_osrs_p_t;

// temperature oversampling register settings
typedef enum bmp280_osrs_t_t {
 bmp280_osrs_t_skip = 0b00000000,
 bmp280_osrs_t_1x = 0b00100000,
 bmp280_osrs_t_2x = 0b01000000,
 bmp280_osrs_t_4x = 0b01100000,
 bmp280_osrs_t_8x = 0b10000000,
 bmp280_osrs_t_16x = 0b10100000
}bmp280_osrs_t_t;

typedef enum bmp280_mode_t{
	bmp280_sleep_mode = 0b00000000,
	bmp280_forced_mode = 0b00000010,
	bmp280_normal_mode = 0b00000011
}bmp280_mode_t;

typedef enum bmp280_sb_t{
	bmp280_sb_05 = 0b00000000,
	bmp280_sb_62 = 0b00100000,
	bmp280_sb_125 = 0b01000000,
	bmp280_sb_250 = 0b01100000,
	bmp280_sb_500 = 0b10000000,
	bmp280_sb_1000 = 0b10100000,
	bmp280_sb_2000 = 0b11000000,
	bmp280_sb_4000 = 0b11100000
}bmp280_sb_t;

typedef enum bmp280_filter_t{
	bmp280_filter_off = 0b00000000,
	bmp280_filter_x2 = 0b00000100,
	bmp280_filter_x4 = 0b00001000,
	bmp280_filter_x8 = 0b00001100,
	bmp280_filter_x16 = 0b00010000
}bmp280_filter_t;

typedef enum bmp280_spi_en{
	bmp280_spi_en_off = 0b00000000,
	bmp280_spi_en_on = 0b00000001
}bmp280_spi_en;

typedef enum bmp280_err_t{
	bmp280_err_busy,
	bmp280_err_done,
	bmp280_err_not_init,
	bmp280_err_bus
}bmp280_err_t;

typedef enum bmp280_state_t{
	bmp280_init,
	bmp280_reset,
	bmp280_initializing,
	bmp280_sleeping,
	bmp280_error,
	bmp280_start_measurement,
	bmp280_measuring,
	bmp280_convert_measurement,
	bmp280_done
}bmp280_state_t;

typedef struct bmp280_ret_t {

}bmp280_ret_t;

class bmp280 {
public:
	bmp280();
	virtual ~bmp280();
	virtual bool init();
	void reset();
	void main();
	bool startMeasurement();
	bool readTempPressure(double *t, double *p);

	bool newData();

	bool GetTemperature(double *t);
	bool GetPressure(double *p);

protected:
	bmp280_reg8_t temp_xlsb;
	bmp280_reg8_t temp_lsb;
	bmp280_reg8_t temp_msb;
	bmp280_reg8_t press_xlsb;
	bmp280_reg8_t press_lsb;
	bmp280_reg8_t press_msb;
	bmp280_reg8_t config;
	bmp280_reg8_t ctrl_meas;
	bmp280_reg8_t status;
	bmp280_reg8_t id;
	BMP280_S32_t t_fine;

	unsigned short dig_T1;
	signed short dig_T2, dig_T3;
	unsigned short dig_P1;
	signed short dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
	bmp280_state_t bmp280_state;

	uint32_t last_update;

	bool _newData;

	virtual uint8_t readReg8(bmp280_reg8_t *bmp280_reg_addr);
	virtual uint8_t writeReg8(bmp280_reg8_t *bmp280_reg_addr);
	virtual void readRegMulti(bmp280_reg_addr_t addr_t,uint8_t *buf,uint8_t numRegs);
	virtual void writeRegMulti();

	void readConfigReg();
	void readCtrlMeas();
	void readStatusReg();
	bool readIDReg();
	void readCal();

	void setCtrlMeasReg(bmp280_osrs_t_t osrs_t_t, bmp280_osrs_p_t osrs_p_t, bmp280_mode_t mode_t);
	void setConfigReg(bmp280_sb_t sb_t, bmp280_filter_t filter_t, bmp280_spi_en spi_en);

	double _temp,_pressure;
	bmp280_osrs_p_t osrs_p_t;
	bmp280_osrs_t_t osrs_t_t;
	bmp280_mode_t mode_t;
	bmp280_sb_t _sb_t;
	bmp280_filter_t _filter_t;
	bmp280_spi_en _spi_en;
	BMP280_S32_t adc_t, adc_p;

private:


	double bmp280_compensate_T_double(BMP280_S32_t adc_T);
	double bmp280_compensate_P_double(BMP280_S32_t adc_P);
};

class bmp280i2c : public bmp280{
public:
	bmp280i2c(I2C_HandleTypeDef *hi2c, bmp280_i2c_addr_t dev_addr);
	~bmp280i2c();
	bool init();
private:
	I2C_HandleTypeDef *_hi2c;
	uint16_t _DevAddress;
	uint8_t readReg8(bmp280_reg8_t *bmp280_reg_addr);
	uint8_t writeReg8(bmp280_reg8_t *bmp280_reg_addr);

	void readRegMulti(bmp280_reg_addr_t addr_t,uint8_t *buf,uint8_t numRegs);
	void writeRegMulti();
};

} /* namespace BMP280 */

#endif /* BMP280_H_ */
