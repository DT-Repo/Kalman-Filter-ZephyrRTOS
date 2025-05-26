/*
 * Copyright (c) 2020 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ICM20948_ICM20948_SETUP_H_
#define ZEPHYR_DRIVERS_SENSOR_ICM20948_ICM20948_SETUP_H_

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include "hal/ICM_20948_C.h"
#include "hal/AK09916_REGISTERS.h"
#include <zephyr/sys/printk.h>

int icm20948_sensor_init(const struct device *dev);
/* int icm20948_turn_on_fifo(const struct device *dev);
int icm20948_turn_off_fifo(const struct device *dev); */
//int icm20948_turn_off_sensor(const struct device *dev);
int icm20948_turn_on_sensor(const struct device *dev);
int icm20948_set_odr(const struct device *dev, uint16_t a_rate, uint16_t g_rate);
int icm20948_set_fs(const struct device *dev, uint16_t a_sf, uint16_t g_sf);

#endif /* __SENSOR_ICM20948_ICM20948_SETUP__ */
