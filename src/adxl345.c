/***************************************************************************//**
 *   @file   adxl345.c
 *   @brief  Implementation of ADXL345 Driver.
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

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "adxl345.h"
#include <drv_spi.h>
/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/
#define SPI_BUS_NAME				"spi2"
#define SPI_ADXL345_DEVICE_NAME 	"spi21"


static struct rt_spi_device *spi_dev_adxl345;

struct adxl345_dev {
	/** Measurement range */
	rt_uint8_t		selected_range;
	/** Enable/Disable Full Resolution */
	rt_uint8_t		full_resolution_set;
}dev;

/***************************************************************************//**
 * @brief Reads the value of a register.
 *
 * @param dev              - The device structure.
 * @param register_address - Address of the register.
 *
 * @return register_value  - Value of the register.
*******************************************************************************/
rt_uint8_t adxl345_get_register_value(rt_uint8_t register_address)
{
    rt_uint8_t send,recv;
	send = ADXL345_SPI_READ | register_address;
	rt_spi_send_then_recv(spi_dev_adxl345,&send,1,&recv,1);
	return recv;
}

/***************************************************************************//**
 * @brief Writes data into a register.
 *
 * @param dev              - The device structure.
 * @param register_address - Address of the register.
 * @param register_value   - Data value to write.
 *
 * @return None.
*******************************************************************************/
void adxl345_set_register_value(rt_uint8_t register_address,rt_uint8_t register_value)
{
	rt_uint8_t data_buffer[2] = {0, 0};
	data_buffer[0] = ADXL345_SPI_WRITE | register_address;
	data_buffer[1] = register_value;
	rt_spi_send(spi_dev_adxl345,data_buffer,2);
}

/***************************************************************************//**
 * @brief Initializes the communication peripheral and checks if the ADXL345
 *        part is present.
 *
 * @param device     - The device structure.
 * @param init_param - The structure that contains the device initial
 * 		       parameters.
 *
 * @return status    - Result of the initialization procedure.
 *                     Example: -1 - I2C/SPI peripheral was not initialized or
 *                                   ADXL345 part is not present.
 *                               0 - I2C/SPI peripheral is initialized and
 *                                   ADXL345 part is present.
*******************************************************************************/
rt_err_t adxl345_init()
{
	rt_err_t res;

	struct rt_spi_configuration cfg;

    res = rt_hw_spi_device_attach(SPI_BUS_NAME, SPI_ADXL345_DEVICE_NAME, GPIOA, GPIO_PIN_10);
    if (res != RT_EOK)
        return res;
    {
		cfg.data_width = 8;
		cfg.mode       = RT_SPI_MASTER | RT_SPI_MODE_3 | RT_SPI_MSB;
		cfg.max_hz     = 1* 1000 *1000; /* 5M,SPI max 5MHz,ADXL362 4-wire spi */
	}
     spi_dev_adxl345 = (struct rt_spi_device *)rt_device_find(SPI_ADXL345_DEVICE_NAME);
	if (spi_dev_adxl345)
	{
		if(rt_spi_configure(spi_dev_adxl345, &cfg) != RT_EOK)
			return -RT_ERROR;
	}
 	else
      return -RT_ERROR;

	dev.selected_range       = 2; // Measurement Range: +/- 2g (reset default).
	dev.full_resolution_set  = 0;
	rt_kprintf("ADXL345ID:%x\r\n",adxl345_get_register_value( ADXL345_DEVID));
	if (adxl345_get_register_value( ADXL345_DEVID) != ADXL345_ID)
          return -RT_ERROR;

	return RT_EOK;
}
//INIT_COMPONENT_EXPORT(adxl345_init);


/***************************************************************************//**
 * @brief Places the device into standby/measure mode.
 *
 * @param dev      - The device structure.
 * @param pwr_mode - Power mode.
 *                   Example: 0x0 - standby mode.
 *                            0x1 - measure mode.
 *
 * @return None.
*******************************************************************************/
void adxl345_set_power_mode(rt_uint8_t pwr_mode)
{
	rt_uint8_t old_power_ctl = 0;
	rt_uint8_t new_power_ctl = 0;

	old_power_ctl = adxl345_get_register_value(ADXL345_POWER_CTL);
	new_power_ctl = old_power_ctl & ~ADXL345_PCTL_MEASURE;
	new_power_ctl = new_power_ctl | (pwr_mode * ADXL345_PCTL_MEASURE);
	adxl345_set_register_value(ADXL345_POWER_CTL,new_power_ctl);
}

/***************************************************************************//**
 * @brief Reads the raw output data of each axis.
 *
 * @param dev - The device structure.
 * @param x   - X-axis's output data.
 * @param y   - Y-axis's output data.
 * @param z   - Z-axis's output data.
 *
 * @return None.
*******************************************************************************/
void adxl345_get_xyz(rt_int16_t* x,rt_int16_t* y,rt_int16_t* z)
{
	rt_uint8_t read_buffer[6];
	rt_uint8_t send = ADXL345_SPI_READ |ADXL345_SPI_MB |ADXL345_DATAX0;
	
	 rt_spi_send_then_recv(spi_dev_adxl345,&send,1,read_buffer,6);

		/* x = ((ADXL345_DATAX1) << 8) + ADXL345_DATAX0 */
		*x = ((rt_int16_t)read_buffer[1] << 8) + read_buffer[0];
		/* y = ((ADXL345_DATAY1) << 8) + ADXL345_DATAY0 */
		*y = ((rt_int16_t)read_buffer[3] << 8) + read_buffer[2];
		/* z = ((ADXL345_DATAZ1) << 8) + ADXL345_DATAZ0 */
		*z = ((rt_int16_t)read_buffer[5] << 8) + read_buffer[4];
}


/***************************************************************************//**
 * @brief Reads the raw output data of each axis and converts it to g.
 *
 * @param dev - The device structure.
 * @param x   - X-axis's output data.
 * @param y   - Y-axis's output data.
 * @param z   - Z-axis's output data.
 *
 * @return None.
*******************************************************************************/
void adxl345_get_g_xyz(float* x,float* y,float* z)
{
	rt_int16_t x_data = 0;  // X-axis's output data.
	rt_int16_t y_data = 0;  // Y-axis's output data.
	rt_int16_t z_data = 0;  // Z-axis's output data.

	adxl345_get_xyz(&x_data, &y_data, &z_data);
	*x = (float)(dev.full_resolution_set ? (x_data * ADXL345_SCALE_FACTOR) :
		     (x_data * ADXL345_SCALE_FACTOR * (dev.selected_range >> 1)));
	*y = (float)(dev.full_resolution_set ? (y_data * ADXL345_SCALE_FACTOR) :
		     (y_data * ADXL345_SCALE_FACTOR * (dev.selected_range >> 1)));
	*z = (float)(dev.full_resolution_set ? (z_data * ADXL345_SCALE_FACTOR) :
		     (z_data * ADXL345_SCALE_FACTOR * (dev.selected_range >> 1)));
}

/***************************************************************************//**
 * @brief Enables/disables the tap detection.
 *
 * @param dev        - The device structure.
 * @param tap_type   - Tap type (none, single, double).
 *                     Example: 0x0 - disables tap detection.
 *				ADXL345_SINGLE_TAP - enables single tap
 *                                                   detection.
 *				ADXL345_DOUBLE_TAP - enables double tap
 *                                                   detection.
 * @param tap_axes   - Axes which participate in tap detection.
 *                     Example: 0x0 - disables axes participation.
 *				ADXL345_TAP_X_EN - enables x-axis participation.
 *				ADXL345_TAP_Y_EN - enables y-axis participation.
 *				ADXL345_TAP_Z_EN - enables z-axis participation.
 * @param tap_dur    - Tap duration. The scale factor is 625us is/LSB.
 * @param tap_latent - Tap latency. The scale factor is 1.25 ms/LSB.
 * @param tap_window - Tap window. The scale factor is 1.25 ms/LSB.
 * @param tap_thresh - Tap threshold. The scale factor is 62.5 mg/LSB.
 * @param tap_int    - Interrupts pin.
 *                     Example: 0x0 - interrupts on INT1 pin.
 *				ADXL345_SINGLE_TAP - single tap interrupts on
 *						     INT2 pin.
 *				ADXL345_DOUBLE_TAP - double tap interrupts on
 *						     INT2 pin.
 *
 * @return None.
*******************************************************************************/
void adxl345_set_tap_detection(
			       rt_uint8_t tap_type,
			       rt_uint8_t tap_axes,
			       rt_uint8_t tap_dur,
			       rt_uint8_t tap_latent,
			       rt_uint8_t tap_window,
			       rt_uint8_t tap_thresh,
			       rt_uint8_t tap_int)
{
	rt_uint8_t old_tap_axes = 0;
	rt_uint8_t new_tap_axes = 0;
	rt_uint8_t old_int_map = 0;
	rt_uint8_t new_int_map = 0;
	rt_uint8_t old_int_enable = 0;
	rt_uint8_t new_int_enable = 0;

	old_tap_axes = adxl345_get_register_value(ADXL345_TAP_AXES);
	new_tap_axes = old_tap_axes & ~(ADXL345_TAP_X_EN |ADXL345_TAP_Y_EN |ADXL345_TAP_Z_EN);
	new_tap_axes = new_tap_axes | tap_axes;
	adxl345_set_register_value(ADXL345_TAP_AXES,new_tap_axes);
	
	adxl345_set_register_value(ADXL345_DUR,tap_dur);
	adxl345_set_register_value(ADXL345_LATENT,tap_latent);
	adxl345_set_register_value(ADXL345_WINDOW,tap_window);
	adxl345_set_register_value(ADXL345_THRESH_TAP, tap_thresh);
	
	old_int_map = adxl345_get_register_value(ADXL345_INT_MAP);
	new_int_map = old_int_map & ~(ADXL345_SINGLE_TAP | ADXL345_DOUBLE_TAP);
	new_int_map = new_int_map | tap_int;
	adxl345_set_register_value(ADXL345_INT_MAP, new_int_map);
	
	old_int_enable = adxl345_get_register_value(ADXL345_INT_ENABLE);
	new_int_enable = old_int_enable & ~(ADXL345_SINGLE_TAP | ADXL345_DOUBLE_TAP);
	new_int_enable = new_int_enable | tap_type;
	adxl345_set_register_value(ADXL345_INT_ENABLE, new_int_enable);
}

/***************************************************************************//**
 * @brief Enables/disables the activity detection.
 *
 * @param dev        - The device structure.
 * @param act_on_off - Enables/disables the activity detection.
 *                    Example: 0x0 - disables the activity detection.
 *                             0x1 - enables the activity detection.
 * @param act_axes   - Axes which participate in detecting activity.
 *                    Example: 0x0 - disables axes participation.
 *                             ADXL345_ACT_X_EN - enables x-axis participation.
 *                             ADXL345_ACT_Y_EN - enables y-axis participation.
 *                             ADXL345_ACT_Z_EN - enables z-axis participation.
 * @param act_ac_dc  - Selects dc-coupled or ac-coupled operation.
 *                    Example: 0x0 - dc-coupled operation.
 *                             ADXL345_ACT_ACDC - ac-coupled operation.
 * @param act_thresh - Threshold value for detecting activity. The scale factor
                      is 62.5 mg/LSB.
 * @param act_int    - Interrupts pin.
 *                    Example: 0x0 - activity interrupts on INT1 pin.
 *                             ADXL345_ACTIVITY - activity interrupts on INT2
 *                                                pin.
 *
 * @return None.
*******************************************************************************/
void adxl345_set_activity_detection(
				    rt_uint8_t act_on_off,
				    rt_uint8_t act_axes,
				    rt_uint8_t act_ac_dc,
				    rt_uint8_t act_thresh,
				    rt_uint8_t act_int)
{
	rt_uint8_t old_act_inact_ctl = 0;
	rt_uint8_t new_act_inact_ctl = 0;
	rt_uint8_t old_int_map       = 0;
	rt_uint8_t new_int_map       = 0;
	rt_uint8_t old_int_enable    = 0;
	rt_uint8_t new_int_enable    = 0;

	old_act_inact_ctl = adxl345_get_register_value(ADXL345_INT_ENABLE);
	new_act_inact_ctl = old_act_inact_ctl & ~(ADXL345_ACT_ACDC | ADXL345_ACT_X_EN |ADXL345_ACT_Y_EN |ADXL345_ACT_Z_EN);
	new_act_inact_ctl = new_act_inact_ctl | (act_ac_dc | act_axes);
	adxl345_set_register_value(ADXL345_ACT_INACT_CTL,new_act_inact_ctl);
	
	adxl345_set_register_value(ADXL345_THRESH_ACT,act_thresh);
	old_int_map = adxl345_get_register_value(ADXL345_INT_MAP);
	
	new_int_map = old_int_map & ~(ADXL345_ACTIVITY);
	new_int_map = new_int_map | act_int;
	adxl345_set_register_value( ADXL345_INT_MAP,new_int_map);
	
	old_int_enable = adxl345_get_register_value(ADXL345_INT_ENABLE);
	
	new_int_enable = old_int_enable & ~(ADXL345_ACTIVITY);
	new_int_enable = new_int_enable | (ADXL345_ACTIVITY * act_on_off);
	adxl345_set_register_value(ADXL345_INT_ENABLE,new_int_enable);
}

/***************************************************************************//**
 * @brief Enables/disables the inactivity detection.
 *
 * @param dev          - The device structure.
 * @param inact_on_off - Enables/disables the inactivity detection.
 *                       Example: 0x0 - disables the inactivity detection.
 *                                0x1 - enables the inactivity detection.
 * @param inact_axes   - Axes which participate in detecting inactivity.
 *                       Example: 0x0 - disables axes participation.
 *                                ADXL345_INACT_X_EN - enables x-axis.
 *                                ADXL345_INACT_Y_EN - enables y-axis.
 *                                ADXL345_INACT_Z_EN - enables z-axis.
 * @param inact_ac_dc  - Selects dc-coupled or ac-coupled operation.
 *                       Example: 0x0 - dc-coupled operation.
 *                                ADXL345_INACT_ACDC - ac-coupled operation.
 * @param inact_thresh - Threshold value for detecting inactivity. The scale
                         factor is 62.5 mg/LSB.
 * @param inact_time   - Inactivity time. The scale factor is 1 sec/LSB.
 * @param inact_int    - Interrupts pin.
 *		         Example: 0x0 - inactivity interrupts on INT1 pin.
 *				  ADXL345_INACTIVITY - inactivity interrupts on
 *						       INT2 pin.
 *
 * @return None.
*******************************************************************************/
void adxl345_set_inactivity_detection(
				      rt_uint8_t inact_on_off,
				      rt_uint8_t inact_axes,
				      rt_uint8_t inact_ac_dc,
				      rt_uint8_t inact_thresh,
				      rt_uint8_t inact_time,
				      rt_uint8_t inact_int)
{
	rt_uint8_t old_act_inact_ctl = 0;
	rt_uint8_t new_act_inact_ctl = 0;
	rt_uint8_t old_int_map = 0;
	rt_uint8_t new_int_map = 0;
	rt_uint8_t old_int_enable = 0;
	rt_uint8_t new_int_enable = 0;

	old_act_inact_ctl = adxl345_get_register_value( ADXL345_INT_ENABLE);
	new_act_inact_ctl = old_act_inact_ctl & ~(ADXL345_INACT_ACDC |ADXL345_INACT_X_EN |ADXL345_INACT_Y_EN |ADXL345_INACT_Z_EN);
	new_act_inact_ctl = new_act_inact_ctl | (inact_ac_dc | inact_axes);
	adxl345_set_register_value( ADXL345_ACT_INACT_CTL, new_act_inact_ctl);
	adxl345_set_register_value(ADXL345_THRESH_INACT,inact_thresh);
	adxl345_set_register_value(ADXL345_TIME_INACT,inact_time);
	old_int_map = adxl345_get_register_value(ADXL345_INT_MAP);
	new_int_map = old_int_map & ~(ADXL345_INACTIVITY);
	new_int_map = new_int_map | inact_int;
	adxl345_set_register_value(ADXL345_INT_MAP,new_int_map);
	
	old_int_enable = adxl345_get_register_value( ADXL345_INT_ENABLE);
	new_int_enable = old_int_enable & ~(ADXL345_INACTIVITY);
	new_int_enable = new_int_enable | (ADXL345_INACTIVITY * inact_on_off);
	adxl345_set_register_value(ADXL345_INT_ENABLE,new_int_enable);
}

/***************************************************************************//**
 * @brief Enables/disables the free-fall detection.
 *
 * @param dev       - The device structure.
 * @param ff_on_off - Enables/disables the free-fall detection.
 *                    Example: 0x0 - disables the free-fall detection.
 *                             0x1 - enables the free-fall detection.
 * @param ff_thresh - Threshold value for free-fall detection. The scale factor
 *                    is 62.5 mg/LSB.
 * @param ff_time   - Time value for free-fall detection. The scale factor is
 *                    5 ms/LSB.
 * @param ff_int    - Interrupts pin.
 *		      Example: 0x0 - free-fall interrupts on INT1 pin.
 *			       ADXL345_FREE_FALL - free-fall interrupts on
 *                                                 INT2 pin.
 *
 * @return None.
*******************************************************************************/
void adxl345_set_free_fall_detection(rt_uint8_t ff_on_off,rt_uint8_t ff_thresh,rt_uint8_t ff_time,rt_uint8_t ff_int)
{
	rt_uint8_t old_int_map    = 0;
	rt_uint8_t new_int_map    = 0;
	rt_uint8_t new_int_enable = 0;
	rt_uint8_t old_int_enable = 0;

	adxl345_set_register_value(ADXL345_THRESH_FF,ff_thresh);
	adxl345_set_register_value(ADXL345_TIME_FF, ff_time);
	
	old_int_map = adxl345_get_register_value(ADXL345_INT_MAP);
	
	new_int_map = old_int_map & ~(ADXL345_FREE_FALL);
	new_int_map = new_int_map | ff_int;
	
	adxl345_set_register_value(ADXL345_INT_MAP, new_int_map);
	
	old_int_enable = adxl345_get_register_value(ADXL345_INT_ENABLE);
	
	new_int_enable = old_int_enable & ~ADXL345_FREE_FALL;
	new_int_enable = new_int_enable | (ADXL345_FREE_FALL * ff_on_off);
	
	adxl345_set_register_value( ADXL345_INT_ENABLE,new_int_enable);
}

/***************************************************************************//**
 * @brief Sets an offset value for each axis (Offset Calibration).
 *
 * @param dev      - The device structure.
 * @param x_offset - X-axis's offset.
 * @param y_offset - Y-axis's offset.
 * @param z_offset - Z-axis's offset.
 *
 * @return None.
*******************************************************************************/
void adxl345_set_offset(
			rt_uint8_t x_offset,
			rt_uint8_t y_offset,
			rt_uint8_t z_offset)
{
	adxl345_set_register_value(ADXL345_OFSX,x_offset);
	adxl345_set_register_value( ADXL345_OFSY, y_offset);
	adxl345_set_register_value( ADXL345_OFSZ,z_offset);
}

/***************************************************************************//**
 * @brief Selects the measurement range.
 *
 * @param dev      - The device structure.
 * @param g_range  - Range option.
 *                   Example: ADXL345_RANGE_PM_2G  - +-2 g
 *                            ADXL345_RANGE_PM_4G  - +-4 g
 *                            ADXL345_RANGE_PM_8G  - +-8 g
 *                            ADXL345_RANGE_PM_16G - +-16 g
 * @param full_res - Full resolution option.
 *                   Example: 0x0 - Disables full resolution.
 *                            ADXL345_FULL_RES - Enables full resolution.
 *
 * @return None.
*******************************************************************************/
void adxl345_set_range_resolution(
				  rt_uint8_t g_range,
				  rt_uint8_t full_res)
{
	rt_uint8_t old_data_format = 0;
	rt_uint8_t new_data_format = 0;

	old_data_format = adxl345_get_register_value(ADXL345_DATA_FORMAT);
	new_data_format = old_data_format &~(ADXL345_RANGE(0x3) | ADXL345_FULL_RES);
	new_data_format =  new_data_format | ADXL345_RANGE(g_range) | full_res;
	adxl345_set_register_value( ADXL345_DATA_FORMAT,new_data_format);

	dev.selected_range      = (1 << (g_range + 1));
	dev.full_resolution_set = full_res ? 1 : 0;
}
