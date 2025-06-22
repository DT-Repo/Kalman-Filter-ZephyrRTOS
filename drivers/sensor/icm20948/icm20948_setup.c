/*
 * Copyright (c) 2020 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "icm20948.h"
#include "icm20948_reg.h"
#include "icm20948_i2c.h"
#include "icm20948_setup.h"

const ICM_20948_Serif_t icm20948_Serif = {
	my_write_i2c, // write
	my_read_i2c,  // read
	NULL,
};

LOG_MODULE_DECLARE(ICM20948, CONFIG_SENSOR_LOG_LEVEL);

int icm20948_set_fs(const struct device *dev, uint16_t a_sf, uint16_t g_sf)
{
	//const struct icm20948_config *cfg = dev->config;
	struct icm20948_data *drv_data = dev->data;
	ICM_20948_Device_t *driver = &(drv_data->driver);
	int result;
	ICM_20948_fss_t myfss;
	myfss.a = a_sf;
	myfss.g = g_sf;

	result = ICM_20948_set_full_scale(driver, (ICM_20948_InternalSensorID_bm)(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myfss);

	if (result) {
		return result;
	}

	return 0; 
}

int icm20948_set_odr(const struct device *dev, uint16_t a_rate, uint16_t g_rate)
{
	//const struct icm20948_config *cfg = dev->config;
	struct icm20948_data *drv_data = dev->data;
	ICM_20948_Device_t *driver = &(drv_data->driver);
	ICM_20948_smplrt_t mySMPLRTcfg;
	int result = 0;

	mySMPLRTcfg.a = a_rate;
	mySMPLRTcfg.g = g_rate;
	result = ICM_20948_set_sample_rate(driver, (ICM_20948_InternalSensorID_bm)(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySMPLRTcfg);

	return result;
}


int icm20948_sensor_init(const struct device *dev)
{
	//int result = 0;
	struct icm20948_data *drv_data = dev->data;
	ICM_20948_Device_t *driver = &(drv_data->driver);
	const struct icm20948_config *cfg = dev->config;

	drv_data->accel_fss = gpm2;
	drv_data->gyro_fss = dps250;
	drv_data->gyro_hz = 1;
	drv_data->accel_hz = 1;

	ICM_20948_init_struct(driver);
	/* Initialize serial interface and device */
	driver->_serif = &icm20948_Serif;
	
	while (ICM_20948_check_id(driver) != ICM_20948_Stat_Ok)
	{
        //TODO add a timeout or max retries
		LOG_INF("whoami does not match. Halting...");
		k_sleep(K_SECONDS(1));
	}

	return 0;
}


int icm20948_turn_on_sensor(const struct device *dev)
{
	struct icm20948_data *drv_data = dev->data;
	//const struct icm20948_config *cfg = dev->config;
	ICM_20948_Device_t *driver = &(drv_data->driver);
	int result = 0;

	ICM_20948_sw_reset(driver);   

  	k_sleep(K_MSEC(250));

	// Now wake the sensor up
	ICM_20948_sleep(driver, false);
	ICM_20948_low_power(driver, false);
	// Start the magnetometer
	startupMagnetometer(dev, false);

    // Set Gyro and Accelerometer to a particular sample mode
	ICM_20948_set_sample_mode(driver, (ICM_20948_InternalSensorID_bm)(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous); // optiona: ICM_20948_Sample_Mode_Continuous. ICM_20948_Sample_Mode_Cycled
	icm20948_set_fs(dev, drv_data->accel_fss, drv_data->gyro_fss);

	// Set up DLPF configuration
	ICM_20948_dlpcfg_t myDLPcfg;
	myDLPcfg.a = acc_d473bw_n499bw;
	myDLPcfg.g = gyr_d361bw4_n376bw5;
	ICM_20948_set_dlpf_cfg(driver, (ICM_20948_InternalSensorID_bm)(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);

    icm20948_set_odr(dev, drv_data->accel_hz, drv_data->gyro_hz);
	// Choose whether or not to use DLPF
	ICM_20948_enable_dlpf(driver, ICM_20948_Internal_Acc, false);
	ICM_20948_enable_dlpf(driver, ICM_20948_Internal_Gyr, false);

    result = ICM_20948_Stat_Ok;

    return result;
}

