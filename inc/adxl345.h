/***************************************************************************//**
 *   @file   adxl345.h
 *   @brief  Header file of ADXL345 Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#ifndef __ADXL345_H__
#define __ADXL345_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "adxl345_reg.h"
#include <rtthread.h>
#include <rtdevice.h>


/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/*! Reads the value of a register. */
rt_uint8_t adxl345_get_register_value(rt_uint8_t register_address);

/*! Writes data into a register. */
void adxl345_set_register_value(
	         rt_uint8_t register_address,
             rt_uint8_t register_value);

/*! Init. the comm. peripheral and checks if the ADXL345 part is present. */
rt_err_t adxl345_init(void);

/*! Places the device into standby/measure mode. */
void adxl345_set_power_mode(rt_uint8_t pwr_mode);

/*! Reads the raw output data of each axis. */
void adxl345_get_xyz(
		     rt_int16_t* x,
		     rt_int16_t* y,
		     rt_int16_t* z);

/*! Reads the raw output data of each axis and converts it to g. */
void adxl345_get_g_xyz(
		       float* x,
		       float* y,
		       float* z);

/*! Enables/disables the tap detection. */
void adxl345_set_tap_detection(
			       rt_uint8_t tap_type,
			       rt_uint8_t tap_axes,
			       rt_uint8_t tap_dur,
			       rt_uint8_t tap_latent,
			       rt_uint8_t tap_window,
			       rt_uint8_t tap_thresh,
			       rt_uint8_t tap_int);

/*! Enables/disables the activity detection. */
void adxl345_set_activity_detection(
				    rt_uint8_t act_on_off,
				    rt_uint8_t act_axes,
				    rt_uint8_t act_ac_dc,
				    rt_uint8_t act_thresh,
				    rt_uint8_t act_int);

/*! Enables/disables the inactivity detection. */
void adxl345_set_inactivity_detection(
				      rt_uint8_t inact_on_off,
				      rt_uint8_t inact_axes,
				      rt_uint8_t inact_ac_dc,
				      rt_uint8_t inact_thresh,
				      rt_uint8_t inact_time,
				      rt_uint8_t inact_int);

/*! Enables/disables the free-fall detection. */
void adxl345_set_free_fall_detection(
					rt_uint8_t ff_on_off,
					rt_uint8_t ff_thresh,
					rt_uint8_t ff_time,
					rt_uint8_t ff_int);

/*! Sets an offset value for each axis (Offset Calibration). */
void adxl345_set_offset(
			rt_uint8_t x_offset,
			rt_uint8_t y_offset,
			rt_uint8_t z_offset);

/*! Selects the measurement range. */
void adxl345_set_range_resolution(
				  rt_uint8_t g_range,
				  rt_uint8_t full_res);

#endif	/* __ADXL345_H__ */

