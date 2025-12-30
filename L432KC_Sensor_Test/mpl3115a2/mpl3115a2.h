/*
 * mpl3115a2.h
 *
 *  Created on: Dec 29, 2025
 *      Author: coryg
 */

#ifndef MPL3115A2_H_
#define MPL3115A2_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

namespace mpl3115a2 {

#define mpl3115a2_who_am_i 0xC4

typedef enum mpl3115a2_reg_addr_t{
	mpl3115a2_status_addr = 0x00,
	mpl3115a2_out_p_msb_addr = 0x01,
	mpl3115a2_out_p_csb_addr = 0x02,
	mpl3115a2_out_p_lsb_addr = 0x03,
	mpl3115a2_out_t_msb_addr = 0x04,
	mpl3115a2_out_t_lsb_addr = 0x05,
	mpl3115a2_dr_status_addr = 0x06,
	mpl3115a2_out_p_delta_msb_addr = 0x07,
	mpl3115a2_out_p_delta_csb_addr = 0x08,
	mpl3115a2_out_p_delta_lsb_addr = 0x09,
	mpl3115a2_out_t_delta_msb_addr = 0x0A,
	mpl3115a2_out_t_delta_lsb_addr = 0x0B,
	mpl3115a2_who_am_i_addr = 0x0C,
	mpl3115a2_f_status_addr = 0x0D,
	mpl3115a2_f_data_addr = 0x0E,
	mpl3115a2_f_setup = 0x0F,
	mpl3115a2_time_dly_addr = 0x10,
	mpl3115a2_sysmod_addr = 0x11,
	mpl3115a2_int_source_addr = 0x12,
	mpl3115a2_pt_data_cfg_addr = 0x13,
	mpl3115a2_bar_in_msb_addr = 0x14,
	mpl3115a2_bar_in_lsb_addr = 0x15,
	mpl3115a2_p_tgt_msb_addr = 0x16,
	mpl3115a2_p_tgt_lsb_addr = 0x17,
	mpl3115a2_t_tgt_addr = 0x18,
	mpl3115a2_p_wnd_msb_addr = 0x19,
	mpl3115a2_p_wnd_lsb_addr = 0x1A,
	mpl3115a2_t_wnd_addr = 0x1B,
	mpl3115a2_p_min_msb_addr = 0x1C,
	mpl3115a2_p_min_csb_addr = 0x1D,
	mpl3115a2_p_min_lsb_addr = 0x1E,
	mpl3115a2_t_min_msb_addr = 0x1F,
	mpl3115a2_t_min_lsb_addr = 0x20,
	mpl3115a2_p_max_msb_addr = 0x21,
	mpl3115a2_p_max_csb_addr = 0x22,
	mpl3115a2_p_max_lsb_addr = 0x23,
	mpl3115a2_t_max_msb_addr = 0x24,
	mpl3115a2_t_max_lsb_addr = 0x25,
	mpl3115a2_ctrl_reg1_addr = 0x26,
	mpl3115a2_ctrl_reg2_addr = 0x27,
	mpl3115a2_ctrl_reg3_addr = 0x28,
	mpl3115a2_ctrl_reg4_addr = 0x29,
	mpl3115a2_ctrl_reg5_addr = 0x2A,
	mpl3115a2_off_p_addr = 0x2B,
	mpl3115a2_off_t_addr = 0x2C,
	mpl3115a2_off_h_addr = 0x2D
}mpl3115a2_reg_addr_t;

typedef uint8_t mpl3115a2_reg_value_t;

typedef struct mpl3115a2_reg_t{
	mpl3115a2_reg_addr_t addr;
	mpl3115a2_reg_value_t reg_value;
}mpl3115a2_reg_t;

typedef enum mpl3115a2_state_t{
	mpl3115a2_init_state,
	mpl3115a2_error_state
}mpl3115a2_state_t;

typedef enum mpl3115a2_mode_t{
	mpl3115a2_altimeter_mode,
	mpl3115a2_barometer_mode
}mpl3115a2_mode_t;

typedef struct mpl3115a2_param_t{
	I2C_HandleTypeDef *_hi2c;
	uint16_t _DevAddress;
	mpl3115a2_mode_t mode;
}mpl3115a2_param_t;

class mpl3115a2 {
public:
	mpl3115a2(mpl3115a2_param_t param);
	virtual ~mpl3115a2();
	void main();
protected:

private:
	mpl3115a2_state_t _state;
	mpl3115a2_param_t _param;

	bool checkDevice();
	bool who_am_i();
	bool _readReg8(mpl3115a2_reg_t *reg);
};

} /* namespace mpl3115a2 */

#endif /* MPL3115A2_H_ */
