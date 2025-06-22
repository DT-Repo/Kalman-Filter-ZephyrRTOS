/*
 * Copyright (c) 2020 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef KALMAN_FILTER_APPLICATION_DRIVERS_SENSOR_ICM20948_ICM20948_H_
#define KALMAN_FILTER_APPLICATION_DRIVERS_SENSOR_ICM20948_ICM20948_H_

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/util.h>

//#include "icm20948_reg.h"

#include "icm20948_setup.h"

/* typedef void (*tap_fetch_t)(const struct device *dev);
int icm20948_tap_fetch(const struct device *dev); */

union icm20948_bus {
#if CONFIG_SPI
	struct spi_dt_spec spi;
#endif
#if CONFIG_I2C
	struct i2c_dt_spec i2c;
#endif
};

/* static inline int icm20948_reg_read(const struct device *dev, uint8_t reg, uint8_t *buf,
				    uint32_t size)
{
	const struct icm20948_config *cfg = dev->config;

	return cfg->bus_io->read(&cfg->bus, reg, buf, size);
}

static inline int inv_io_hal_read_reg(const struct device *dev, uint8_t reg, uint8_t *rbuffer, uint32_t rlen)
{
	return icm20948_reg_read(dev, reg, rbuffer, rlen);
} */

typedef int (*icm20948_bus_check_fn)(const union icm20948_bus *bus);
typedef int (*icm20948_reg_read_fn)(const union icm20948_bus *bus, uint8_t reg, uint8_t *buf,
				    uint32_t size);
typedef int (*icm20948_reg_write_fn)(const union icm20948_bus *bus, uint8_t reg, uint8_t *buf,
				     uint32_t size);

struct icm20948_bus_io {
	icm20948_bus_check_fn check;
	icm20948_reg_read_fn read;
	icm20948_reg_write_fn write;
};

#if CONFIG_SPI
extern const struct icm20948_bus_io icm20948_bus_io_spi;
#endif

#if CONFIG_I2C
extern const struct icm20948_bus_io icm20948_bus_io_i2c;
#endif

struct icm20948_data {
	//uint8_t fifo_data[HARDWARE_FIFO_SIZE];
	ICM_20948_Device_t driver;
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	uint16_t accel_fss;
	uint16_t accel_hz;

	int16_t temp;

	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	uint16_t gyro_fss;
	uint16_t gyro_hz;

	int16_t magn_x;
	int16_t magn_y;
	int16_t magn_z;

/* 	bool accel_en;
	bool gyro_en;
	bool tap_en;

	bool sensor_started; */

	const struct device *dev;
/* 	struct gpio_callback gpio_cb;

	const struct sensor_trigger *data_ready_trigger;
	sensor_trigger_handler_t data_ready_handler;

	const struct sensor_trigger *tap_trigger;
	sensor_trigger_handler_t tap_handler;

	const struct sensor_trigger *double_tap_trigger;
	sensor_trigger_handler_t double_tap_handler; */

#ifdef CONFIG_ICM20948_TRIGGER
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_ICM20948_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem gpio_sem;
#endif
};

struct icm20948_config {
	union icm20948_bus bus;
	const struct icm20948_bus_io *bus_io;
	//struct gpio_dt_spec gpio_int;
};

/* int icm20948_trigger_set(const struct device *dev,
			 const struct sensor_trigger *trig,
			 sensor_trigger_handler_t handler);

int icm20948_init_interrupt(const struct device *dev); */

#endif /* __SENSOR_ICM20948__ */
