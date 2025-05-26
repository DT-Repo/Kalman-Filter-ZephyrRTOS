/*
 * Copyright (c) 2020 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ICM20948_ICM20948_I2C_H_
#define ZEPHYR_DRIVERS_SENSOR_ICM20948_ICM20948_I2C_H_

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include "hal/ICM_20948_C.h"
#include "hal/AK09916_REGISTERS.h"
#include <zephyr/sys/printk.h>

#define MAX_MAGNETOMETER_STARTS     10
#define ICM20948_I2C_ADDR			0x68

union icm20948_bus {
#if CONFIG_SPI
	struct spi_dt_spec spi;
#endif
//#if CONFIG_I2C
	struct i2c_dt_spec i2c;
//#endif
};

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

//#if CONFIG_SPI
extern const struct icm20948_bus_io icm20948_bus_io_spi;
//#endif

#if CONFIG_I2C
extern const struct icm20948_bus_io icm20948_bus_io_i2c;
#endif

//#if CONFIG_I2C
static int icm20948_bus_check_i2c(const union icm20948_bus *bus);
static int icm20948_reg_read_i2c(const union icm20948_bus *bus, uint8_t reg, uint8_t *buf,
				 uint32_t size);
static int icm20948_reg_write_i2c(const union icm20948_bus *bus, uint8_t reg, uint8_t *buf,
				  uint32_t size);
ICM_20948_Status_e my_write_i2c(uint8_t reg, uint8_t *data, uint32_t len, void *user);
ICM_20948_Status_e my_read_i2c(uint8_t reg, uint8_t *buff, uint32_t len, void *user);
//#endif

//#if MAGNETOMETER
ICM_20948_Status_e startupMagnetometer(const struct device *dev, bool minimal);
ICM_20948_Status_e i2cMasterPassthrough(const struct device *dev, bool passthrough);
ICM_20948_Status_e i2cMasterEnable(const struct device *dev, bool enable);
ICM_20948_Status_e i2cMasterReset(const struct device *dev);
ICM_20948_Status_e magWhoIAm(const struct device *dev);
uint8_t readMag(const struct device *dev, AK09916_Reg_Addr_e reg);
uint8_t i2cMasterSingleR(const struct device *dev, uint8_t addr, uint8_t reg);
ICM_20948_Status_e i2cControllerConfigurePeripheral(const struct device *dev, uint8_t peripheral, uint8_t addr, uint8_t reg, 
                    uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap, uint8_t dataOut);
ICM_20948_Status_e writeMag(const struct device *dev, AK09916_Reg_Addr_e reg, uint8_t *pdata);
ICM_20948_Status_e i2cMasterSingleW(const struct device *dev, uint8_t addr, uint8_t reg, uint8_t data);
ICM_20948_Status_e resetMag(const struct device *dev);
//#endif

#endif /* __SENSOR_ICM20948_ICM20948_I2C__ */
