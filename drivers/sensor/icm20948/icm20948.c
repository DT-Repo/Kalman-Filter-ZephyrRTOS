/*
 * Copyright (c) 2020 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT invensense_icm20948

//#include <zephyr/init.h>
//#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "icm20948.h"
//#include "icm20948_reg.h"

extern struct device *i2c_dev;
LOG_MODULE_REGISTER(ICM20948, CONFIG_SENSOR_LOG_LEVEL);

/* static const uint16_t icm20948_gyro_sensitivity_x10[] = {
	1310, 655, 328, 164
}; */

/* see "Accelerometer Measurements" section from register map description */
static void icm20948_convert_accel(struct sensor_value *val,
				   int16_t raw_val,
				   uint8_t fss)
{
	float conv_val;

	switch (fss)
	{
	case 0:
	  conv_val = ((float)raw_val) / 16.384;
	  break;
	case 1:
	  conv_val = ((float)raw_val) / 8.192;
	  break;
	case 2:
	  conv_val = ((float)raw_val) / 4.096;
	  break;
	case 3:
	  conv_val = ((float)raw_val) / 2.048;
	  break;
	default:
	  return 0;
	  break;
	}

	val->val1 = (int32_t)conv_val; 
    val->val2 = (int32_t)((conv_val - val->val1) * 1000000); 

	if (val->val2 < 0) {
		val->val1--;
		val->val2 += 1000000;
	} else if (val->val2 >= 1000000) {
		val->val1++;
		val->val2 -= 1000000;
	}
}


/* see "Gyroscope Measurements" section from register map description */
static void icm20948_convert_gyro(struct sensor_value *val,
				  int16_t raw_val,
				  uint8_t fss)
{
	float conv_val;

	switch (fss)
	{
	case 0:
	  conv_val = ((float)raw_val) / 131;
	  break;
	case 1:
	  conv_val = ((float)raw_val) / 65.5;
	  break;
	case 2:
	  conv_val = ((float)raw_val) / 32.8;
	  break;
	case 3:
	  conv_val = ((float)raw_val) / 16.4;
	  break;
	default:
	  return 0;
	  break;
	}
	val->val1 = (int32_t)conv_val; 
    val->val2 = (int32_t)((conv_val - val->val1) * 1000000); 

	if (val->val2 < 0) {
		val->val1--;
		val->val2 += 1000000;
	} else if (val->val2 >= 1000000) {
		val->val1++;
		val->val2 -= 1000000;
	}
}

/* see "Temperature Measurement" section from register map description */
static inline void icm20948_convert_temp(struct sensor_value *val,
					 int16_t raw_val)
{
	float conv_val;
	conv_val = (float)raw_val / 333.87;
/* 	val->val1 = (((int64_t)raw_val * 100) / 333) + 87;
	val->val2 = ((((int64_t)raw_val * 100) % 207) * 1000000) / 207; */

	val->val1 = (int32_t)conv_val; 
    val->val2 = (int32_t)((conv_val - val->val1) * 1000000); 

	if (val->val2 < 0) {
		val->val1--;
		val->val2 += 1000000;
	} else if (val->val2 >= 1000000) {
		val->val1++;
		val->val2 -= 1000000;
	}
}

/* see "Temperature Measurement" section from register map description */
static inline void icm20948_convert_magn(struct sensor_value *val,
	int16_t raw_val)
{
	float conv_val;
	conv_val = (float)raw_val * 0.15;
	
	val->val1 = (int32_t)conv_val; 
    val->val2 = (int32_t)((conv_val - val->val1) * 1000000); 
	
	if (val->val2 < 0) {
		val->val1--;
		val->val2 += 1000000;
	} else if (val->val2 >= 1000000) {
		val->val1++;
		val->val2 -= 1000000;
	}
}

static int icm20948_channel_get(const struct device *dev,
				enum sensor_channel chan,
				struct sensor_value *val)
{
	const struct icm20948_data *drv_data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		icm20948_convert_accel(val, drv_data->accel_x,
				       drv_data->accel_fss);
		icm20948_convert_accel(val + 1, drv_data->accel_y,
				       drv_data->accel_fss);
		icm20948_convert_accel(val + 2, drv_data->accel_z,
				       drv_data->accel_fss);
		break;
	case SENSOR_CHAN_ACCEL_X:
		icm20948_convert_accel(val, drv_data->accel_x,
				       drv_data->accel_fss);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		icm20948_convert_accel(val, drv_data->accel_y,
				       drv_data->accel_fss);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		icm20948_convert_accel(val, drv_data->accel_z,
				       drv_data->accel_fss);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		icm20948_convert_gyro(val, drv_data->gyro_x,
				      drv_data->gyro_fss);
		icm20948_convert_gyro(val + 1, drv_data->gyro_y,
				      drv_data->gyro_fss);
		icm20948_convert_gyro(val + 2, drv_data->gyro_z,
				      drv_data->gyro_fss);
		break;
	case SENSOR_CHAN_GYRO_X:
		icm20948_convert_gyro(val, drv_data->gyro_x,
				      drv_data->gyro_fss);
		break;
	case SENSOR_CHAN_GYRO_Y:
		icm20948_convert_gyro(val, drv_data->gyro_y,
				      drv_data->gyro_fss);
		break;
	case SENSOR_CHAN_GYRO_Z:
		icm20948_convert_gyro(val, drv_data->gyro_z,
				      drv_data->gyro_fss);
		break;
		case SENSOR_CHAN_MAGN_XYZ:
		icm20948_convert_magn(val, drv_data->magn_x);
		icm20948_convert_magn(val + 1, drv_data->magn_y);
		icm20948_convert_magn(val + 2, drv_data->magn_z);
		break;
	case SENSOR_CHAN_MAGN_X:
		icm20948_convert_magn(val, drv_data->magn_x);
		break;
	case SENSOR_CHAN_MAGN_Y:
		icm20948_convert_magn(val, drv_data->magn_y);
		break;
	case SENSOR_CHAN_MAGN_Z:
		icm20948_convert_magn(val, drv_data->magn_z);
		break;
	case SENSOR_CHAN_DIE_TEMP:
		icm20948_convert_temp(val, drv_data->temp);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

/* int icm20948_tap_fetch(const struct device *dev)
{
	int result = 0;
	struct icm20948_data *drv_data = dev->data;
	const struct icm20948_config *cfg = dev->config;

	if (drv_data->tap_en &&
	    (drv_data->tap_handler || drv_data->double_tap_handler)) {
		result = inv_spi_read(&cfg->spi, REG_INT_STATUS3, drv_data->fifo_data, 1);
		if (drv_data->fifo_data[0] & BIT_INT_STATUS_TAP_DET) {
			result = inv_spi_read(&cfg->spi, REG_APEX_DATA4,
					      drv_data->fifo_data, 1);
			if (drv_data->fifo_data[0] & APEX_TAP) {
				if (drv_data->tap_trigger->type ==
				    SENSOR_TRIG_TAP) {
					if (drv_data->tap_handler) {
						LOG_DBG("Single Tap detected");
						drv_data->tap_handler(dev
						      , drv_data->tap_trigger);
					}
				} else {
					LOG_ERR("Trigger type is mismatched");
				}
			} else if (drv_data->fifo_data[0] & APEX_DOUBLE_TAP) {
				if (drv_data->double_tap_trigger->type ==
				    SENSOR_TRIG_DOUBLE_TAP) {
					if (drv_data->double_tap_handler) {
						LOG_DBG("Double Tap detected");
						drv_data->double_tap_handler(dev
						     , drv_data->tap_trigger);
					}
				} else {
					LOG_ERR("Trigger type is mismatched");
				}
			} else {
				LOG_DBG("Not supported tap event");
			}
		}
	}

	return 0;
} */

static int icm20948_sample_fetch(const struct device *dev,
				 enum sensor_channel chan)
{
	struct icm20948_data *drv_data = dev->data;
	ICM_20948_Device_t *driver = &(drv_data->driver);
	//const struct icm20948_config *cfg = dev->config;
	ICM_20948_AGMT_t agmt = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0}};

	 ICM_20948_Status_e ret_gmt = ICM_20948_get_agmt(driver, &agmt);
	if (ret_gmt == ICM_20948_Stat_Ok){
		drv_data->accel_x = agmt.acc.axes.x;
		drv_data->accel_y = agmt.acc.axes.y;
		drv_data->accel_z = agmt.acc.axes.z;
	
		drv_data->gyro_x = agmt.gyr.axes.x;
		drv_data->gyro_y = agmt.gyr.axes.y;
		drv_data->gyro_z = agmt.gyr.axes.z;
	
		drv_data->magn_x = agmt.mag.axes.x;
		drv_data->magn_y = agmt.mag.axes.y;
		drv_data->magn_z = agmt.mag.axes.z;
	}

	return 0;
}

static int icm20948_attr_set(const struct device *dev,
			     enum sensor_channel chan,
			     enum sensor_attribute attr,
			     const struct sensor_value *val)
{
	struct icm20948_data *drv_data = dev->data;

	__ASSERT_NO_MSG(val != NULL);

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
		if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
			if (val->val1 > 8000 || val->val1 < 1) {
				LOG_ERR("Incorrect sampling value");
				return -EINVAL;
			} else {
				//drv_data->accel_hz = val->val1;
			}
		} else if (attr == SENSOR_ATTR_FULL_SCALE) {
			if (val->val1 < 0 ||
			    val->val1 > 0) {
				LOG_ERR("Incorrect fullscale value");
				return -EINVAL;
			} else {
				//drv_data->accel_sf = val->val1;
			}
		} else {
			LOG_ERR("Not supported ATTR");
			return -ENOTSUP;
		}

		break;
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
	case SENSOR_CHAN_GYRO_XYZ:
		if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
			if (val->val1 > 8000 || val->val1 < 12) {
				LOG_ERR("Incorrect sampling value");
				return -EINVAL;
			} else {
				//drv_data->gyro_hz = val->val1;
			}
		} else if (attr == SENSOR_ATTR_FULL_SCALE) {
			if (val->val1 < 0 ||
			    val->val1 > 0) {
				LOG_ERR("Incorrect fullscale value");
				return -EINVAL;
			} else {
				//drv_data->gyro_sf = val->val1;
			}
		} else {
			LOG_ERR("Not supported ATTR");
			return -EINVAL;
		}
		break;
	default:
		LOG_ERR("Not support");
		return -EINVAL;
	}

	return 0;
}

static int icm20948_attr_get(const struct device *dev,
			     enum sensor_channel chan,
			     enum sensor_attribute attr,
			     struct sensor_value *val)
{
	const struct icm20948_data *drv_data = dev->data;

	__ASSERT_NO_MSG(val != NULL);

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
		if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
			//val->val1 = drv_data->accel_hz;
		} else if (attr == SENSOR_ATTR_FULL_SCALE) {
			//val->val1 = drv_data->accel_sf;
		} else {
			LOG_ERR("Not supported ATTR");
			return -EINVAL;
		}

		break;
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
	case SENSOR_CHAN_GYRO_XYZ:
		if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
			//val->val1 = drv_data->gyro_hz;
		} else if (attr == SENSOR_ATTR_FULL_SCALE) {
			//val->val1 = drv_data->gyro_sf;
		} else {
			LOG_ERR("Not supported ATTR");
			return -EINVAL;
		}

		break;

	default:
		LOG_ERR("Not support");
		return -EINVAL;
	}

	return 0;
}

static int icm20948_data_init(struct icm20948_data *data,
			      const struct icm20948_config *cfg)
{
	data->imu_whoami = ICM_20948_WHOAMI;
	memset(&data->driver, 0, sizeof(data->driver));
	data->accel_x = 0;
	data->accel_y = 0;
	data->accel_z = 0;
	data->accel_hz = 0;
	data->accel_fss = 0;

	data->temp = 0;
	data->gyro_x = 0;
	data->gyro_y = 0;
	data->gyro_z = 0;
	data->gyro_hz = 0;
	data->gyro_fss =  0;
	
	data->magn_x = 0;
	data->magn_y = 0;
	data->magn_z = 0;

/* 	data->tap_en = false;
	data->sensor_started = false; */

	return 0;
}


static int icm20948_init(const struct device *dev)
{
	struct icm20948_data *drv_data = dev->data;
	const struct icm20948_config *cfg = dev->config;
	ICM_20948_Device_t *driver = &(drv_data->driver);
	int res;
	
	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("Bus device is not ready");
		return -ENODEV;
	}


    i2c_configure(i2c_dev, I2C_SPEED_SET(I2C_SPEED_STANDARD));

    if (!i2c_dev) {
        // LOG_ERR("I2C device not valid.");
        return ICM_20948_Stat_Err;
    }

	res |= icm20948_data_init(drv_data, cfg);
	
	if (icm20948_sensor_init(dev)) {
		LOG_ERR("could not initialize sensor");
		return -EIO;
	}


	res |= icm20948_turn_on_sensor(dev);

#ifdef CONFIG_ICM20948_TRIGGER
	if (icm20948_init_interrupt(dev) < 0) {
		LOG_ERR("Failed to initialize interrupts.");
		return -EIO;
	}
#endif

	LOG_DBG("Initialize interrupt done");

	return res;
}

static DEVICE_API(sensor, icm20948_driver_api) = {
#ifdef CONFIG_ICM20948_TRIGGER
	.trigger_set = icm20948_trigger_set,
#endif
	.sample_fetch = icm20948_sample_fetch,
	.channel_get = icm20948_channel_get,
	.attr_set = icm20948_attr_set,
	.attr_get = icm20948_attr_get,
};

/* //.gpio_int = GPIO_DT_SPEC_INST_GET(index, int_gpios),    
#define ICM20948_DEFINE_CONFIG(index)					\
	static const struct icm20948_config icm20948_cfg_##index = {	\
		.i2c = I2C_DT_SPEC_INST_GET(index),			\	
		.accel_hz = DT_INST_PROP(index, accel_hz),		\
		.gyro_hz = DT_INST_PROP(index, gyro_hz),		\
		.accel_fs = DT_INST_ENUM_IDX(index, accel_fs),		\
		.gyro_fs = DT_INST_ENUM_IDX(index, gyro_fs),		\
	};


#define ICM20948_INIT(index)						\
	ICM20948_DEFINE_CONFIG(index);					\
	static struct icm20948_data icm20948_driver_##index;		\
	SENSOR_DEVICE_DT_INST_DEFINE(index, icm20948_init,		\
			    NULL,					\
			    &icm20948_driver_##index,			\
			    &icm20948_cfg_##index, POST_KERNEL,		\
			    CONFIG_SENSOR_INIT_PRIORITY,		\
			    &icm20948_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ICM20948_INIT) */





/* 	.accel_hz = DT_INST_PROP(index, accel_hz),		\
	.gyro_hz = DT_INST_PROP(index, gyro_hz),		\
	.accel_fs = DT_INST_ENUM_IDX(index, accel_fs),		\
	.gyro_fs = DT_INST_ENUM_IDX(index, gyro_fs),		\ */




	
#define INIT_ICM20948_INST(inst)						\
	static struct icm20948_data icm20948_data_##inst;			\
	static const struct icm20948_config icm20948_cfg_##inst = {	\
	.i2c = I2C_DT_SPEC_INST_GET(inst),				\
	};								\
									\
	SENSOR_DEVICE_DT_INST_DEFINE(inst, icm20948_init, NULL,		\
			      &icm20948_data_##inst, &icm20948_cfg_##inst,\
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,	\
			      &icm20948_driver_api);

DT_INST_FOREACH_STATUS_OKAY(INIT_ICM20948_INST)
