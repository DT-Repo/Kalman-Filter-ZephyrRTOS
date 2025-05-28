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
	//const struct icm20948_config *config = dev->config;
	//int err = 0;

	drv_data->accel_fss = gpm2;
	drv_data->gyro_fss = dps250;
	drv_data->gyro_hz = 1;
	drv_data->accel_hz = 1;

	ICM_20948_init_struct(driver);
	/* Initialize serial interface and device */
	driver->_serif = &icm20948_Serif;
	
/* 	if (err < 0) {
		LOG_ERR("Init failed: %d", err);
		return err;
	} */

/* 	uint8_t whoami = 0x00;
	err = ICM_20948_get_who_am_i(driver, &whoami);
	if (whoami != ICM_20948_WHOAMI)
	{
		return ICM_20948_Stat_WrongID;
	} */
	
	while (ICM_20948_check_id(driver) != ICM_20948_Stat_Ok)
	{
        //TODO add a timeout or max retries
		LOG_INF("whoami does not match. Halting...");
		k_sleep(K_SECONDS(1));
	}
/* 	if (err < 0) {
		LOG_ERR("ID read failed: %d", err);
		return err;
	} */

/* 	if (data->chip_id != data->imu_whoami) {
		LOG_ERR("invalid WHO_AM_I value, was 0x%x but expected 0x%x for %s", data->chip_id,
			data->imu_whoami, data->imu_name);
		return -ENOTSUP;
	}

	LOG_DBG("\"%s\" %s OK", dev->name, data->imu_name); */
	return 0;
}

/* int icm20948_turn_on_fifo(const struct device *dev)
{
	const struct icm20948_data *drv_data = dev->data;
	const struct icm20948_config *cfg = dev->config;

	uint8_t int0_en = BIT_INT_UI_DRDY_INT1_EN;
	uint8_t fifo_en = BIT_FIFO_ACCEL_EN | BIT_FIFO_GYRO_EN | BIT_FIFO_WM_TH;
	uint8_t burst_read[3];
	int result;
	uint8_t v = 0;

	v = BIT_FIFO_MODE_BYPASS;
	result = inv_spi_single_write(&cfg->spi, REG_FIFO_CONFIG, &v);
	if (result) {
		return result;
	}

	v = 0;
	result = inv_spi_single_write(&cfg->spi, REG_FIFO_CONFIG1, &v);
	if (result) {
		return result;
	}

	result = inv_spi_read(&cfg->spi, REG_FIFO_COUNTH, burst_read, 2);
	if (result) {
		return result;
	}

	result = inv_spi_read(&cfg->spi, REG_FIFO_DATA, burst_read, 3);
	if (result) {
		return result;
	}

	v = BIT_FIFO_MODE_STREAM;
	result = inv_spi_single_write(&cfg->spi, REG_FIFO_CONFIG, &v);
	if (result) {
		return result;
	}

	result = inv_spi_single_write(&cfg->spi, REG_FIFO_CONFIG1, &fifo_en);
	if (result) {
		return result;
	}

	result = inv_spi_single_write(&cfg->spi, REG_INT_SOURCE0, &int0_en);
	if (result) {
		return result;
	}

	if (drv_data->tap_en) {
		v = BIT_TAP_ENABLE;
		result = inv_spi_single_write(&cfg->spi, REG_APEX_CONFIG0, &v);
		if (result) {
			return result;
		}

		v = BIT_DMP_INIT_EN;
		result = inv_spi_single_write(&cfg->spi, REG_SIGNAL_PATH_RESET, &v);
		if (result) {
			return result;
		}

		v = BIT_BANK_SEL_4;
		result = inv_spi_single_write(&cfg->spi, REG_BANK_SEL, &v);
		if (result) {
			return result;
		}

		v = BIT_INT_STATUS_TAP_DET;
		result = inv_spi_single_write(&cfg->spi, REG_INT_SOURCE6, &v);
		if (result) {
			return result;
		}

		v = BIT_BANK_SEL_0;
		result = inv_spi_single_write(&cfg->spi, REG_BANK_SEL, &v);
		if (result) {
			return result;
		}
	}

	LOG_DBG("turn on fifo done");
	return 0;
}

int icm20948_turn_off_fifo(const struct device *dev)
{
	const struct icm20948_data *drv_data = dev->data;
	const struct icm20948_config *cfg = dev->config;
	uint8_t int0_en = 0;
	uint8_t burst_read[3];
	int result;
	uint8_t v = 0;

	v = BIT_FIFO_MODE_BYPASS;
	result = inv_spi_single_write(&cfg->spi, REG_FIFO_CONFIG, &v);
	if (result) {
		return result;
	}

	v = 0;
	result = inv_spi_single_write(&cfg->spi, REG_FIFO_CONFIG1, &v);
	if (result) {
		return result;
	}

	result = inv_spi_read(&cfg->spi, REG_FIFO_COUNTH, burst_read, 2);
	if (result) {
		return result;
	}

	result = inv_spi_read(&cfg->spi, REG_FIFO_DATA, burst_read, 3);
	if (result) {
		return result;
	}

	result = inv_spi_single_write(&cfg->spi, REG_INT_SOURCE0, &int0_en);
	if (result) {
		return result;
	}

	if (drv_data->tap_en) {
		v = 0;
		result = inv_spi_single_write(&cfg->spi, REG_APEX_CONFIG0, &v);
		if (result) {
			return result;
		}

		result = inv_spi_single_write(&cfg->spi, REG_SIGNAL_PATH_RESET, &v);
		if (result) {
			return result;
		}

		v = BIT_BANK_SEL_4;
		result = inv_spi_single_write(&cfg->spi, REG_BANK_SEL, &v);
		if (result) {
			return result;
		}

		v = 0;
		result = inv_spi_single_write(&cfg->spi, REG_INT_SOURCE6, &v);
		if (result) {
			return result;
		}

		v = BIT_BANK_SEL_0;
		result = inv_spi_single_write(&cfg->spi, REG_BANK_SEL, &v);
		if (result) {
			return result;
		}
	}

	return 0;
} */

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

/* int icm20948_turn_off_sensor(const struct device *dev)
{
	const struct icm20948_config *cfg = dev->config;
	uint8_t v = 0;
	int result = 0;

	result = inv_spi_read(&cfg->spi, REG_PWR_MGMT0, &v, 1);

	v ^= BIT_ACCEL_MODE_LNM;
	v ^= BIT_GYRO_MODE_LNM;

	result = inv_spi_single_write(&cfg->spi, REG_PWR_MGMT0, &v);
	if (result) {
		return result;
	}

	// Accelerometer sensor need at least 10ms startup time
	// Gyroscope sensor need at least 30ms startup time
	k_msleep(100);

	icm20948_turn_off_fifo(dev);

	return 0;
} */
