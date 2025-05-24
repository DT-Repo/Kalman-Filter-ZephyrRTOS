/*
 * Copyright (c) 2020 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>
#include "icm20948_i2c.h"
#include "icm20948.h"

LOG_MODULE_DECLARE(ICM20948, CONFIG_SENSOR_LOG_LEVEL);
#define LOG_I2C_ENABLE  0

#define LOG_DBG(fmt, ...) printk(fmt, ##__VA_ARGS__)
#define LOG_INF(fmt, ...) printk(fmt, ##__VA_ARGS__)
#define LOG_WRN(fmt, ...) printk(fmt, ##__VA_ARGS__)
#define LOG_ERR(fmt, ...) printk(fmt, ##__VA_ARGS__)

#if LOG_I2C_ENABLE
#define LOG_I2C(fmt, ...) printk(fmt, ##__VA_ARGS__)
#else
#define LOG_I2C(fmt, ...)
#endif

ICM_20948_Status_e status;
uint8_t imu_i2c_addr = ICM20948_I2C_ADDR;
struct device *i2c_dev = DEVICE_DT_GET_ONE(invensense_icm20948);

//#if CONFIG_I2C
static int icm20948_bus_check_i2c(const union icm20948_bus *bus)
{
	return device_is_ready(bus->i2c.bus) ? 0 : -ENODEV;
}

static int icm20948_reg_read_i2c(const union icm20948_bus *bus, uint8_t reg, uint8_t *buf,
				 uint32_t size)
{
	return i2c_burst_read_dt(&bus->i2c, reg, buf, size);
}

static int icm20948_reg_write_i2c(const union icm20948_bus *bus, uint8_t reg, uint8_t *buf,
				  uint32_t size)
{
	return i2c_burst_write_dt(&bus->i2c, reg, buf, size);
}

const struct icm20948_bus_io icm20948_bus_io_i2c = {
	.check = icm20948_bus_check_i2c,
	.read = icm20948_reg_read_i2c,
	.write = icm20948_reg_write_i2c,
};

ICM_20948_Status_e my_write_i2c(uint8_t reg, uint8_t *data, uint32_t len, void *user)
{
	ICM_20948_Status_e ret = ICM_20948_Stat_Err;

	struct i2c_msg msg[2];

	msg[0].buf = &reg;
	msg[0].len = 1;
	msg[0].flags = I2C_MSG_WRITE;

	msg[1].buf = data;
	msg[1].len = len;
	msg[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	LOG_I2C("> W %d bytes to 0x%2x:\t", len, *(msg[0].buf));

	for (int i = 0; i < len; i++)
	{
		LOG_I2C("0x%2x ", (msg[1].buf)[i]);
	}
	LOG_I2C("\n");

	if (0 == i2c_transfer(i2c_dev, msg, 2, imu_i2c_addr))
	{
		ret = ICM_20948_Stat_Ok;
	}
	else
	{
		LOG_I2C("Write failed\n");
	}

	return ret;
}

ICM_20948_Status_e my_read_i2c(uint8_t reg, uint8_t *buff, uint32_t len, void *user)
{
	ICM_20948_Status_e ret = ICM_20948_Stat_Err;

	if (0 == i2c_write(i2c_dev, &reg, 1, imu_i2c_addr))
	{
		if (0 == i2c_read(i2c_dev, buff, len, imu_i2c_addr))
		{
			LOG_I2C("< R %d bytes from 0x%2x:\t", len, reg);
			for (int i = 0; i < len; i++)
			{
				LOG_I2C("0x%2x ", buff[i]);
			}
			LOG_I2C("\n");
			ret = ICM_20948_Stat_Ok;
		}
		else
		{
			LOG_I2C("Read failed\n");
		}
	}
	else
	{
		LOG_I2C("Read failed\n");
	}

	return ret;
}
//#endif /* CONFIG_I2C */

//#if MAGNETOMETER
ICM_20948_Status_e startupMagnetometer(const struct device *dev, bool minimal)
{
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  i2cMasterPassthrough(dev, false); //Do not connect the SDA/SCL pins to AUX_DA/AUX_CL
  i2cMasterEnable(dev, true);

  resetMag(dev);

  //After a ICM reset the Mag sensor may stop responding over the I2C master
  //Reset the Master I2C until it responds
  uint8_t tries = 0;
  while (tries < MAX_MAGNETOMETER_STARTS)
  {
    tries++;

    //See if we can read the WhoIAm register correctly
    retval = magWhoIAm(dev);
    if (retval == ICM_20948_Stat_Ok)
      break; //WIA matched!

    i2cMasterReset(dev); //Otherwise, reset the master I2C and try again

    k_msleep(10);
  }

  if (tries == MAX_MAGNETOMETER_STARTS)
  {
    // debugPrint(F("ICM_20948::startupMagnetometer: reached MAX_MAGNETOMETER_STARTS ("));
    // debugPrintf((int)MAX_MAGNETOMETER_STARTS);
    // debugPrintln(F("). Returning ICM_20948_Stat_WrongID"));
    status = ICM_20948_Stat_WrongID;
    return status;
  }
  else
  {
    // debugPrint(F("ICM_20948::startupMagnetometer: successful magWhoIAm after "));
    // debugPrintf((int)tries);
    // if (tries == 1)
    //   debugPrintln(F(" try"));
    // else
    //   debugPrintln(F(" tries"));
  }

  //Return now if minimal is true. The mag will be configured manually for the DMP
  if (minimal) // Return now if minimal is true
  {
    // debugPrintln(F("ICM_20948::startupMagnetometer: minimal startup complete!"));
    return status;
  }

  //Set up magnetometer
  AK09916_CNTL2_Reg_t reg;
  reg.MODE = AK09916_mode_cont_100hz;
  reg.reserved_0 = 0; // Make sure the unused bits are clear. Probably redundant, but prevents confusion when looking at the I2C traffic
  retval = writeMag(dev, AK09916_REG_CNTL2, (uint8_t *)&reg);
  if (retval != ICM_20948_Stat_Ok)
  {
    // debugPrint(F("ICM_20948::startupMagnetometer: writeMag returned: "));
    // debugPrintStatus(retval);
    // debugPrintln(F(""));
    status = retval;
    return status;
  }

  retval = i2cControllerConfigurePeripheral(dev, 0, MAG_AK09916_I2C_ADDR, AK09916_REG_ST1, 9, true, true, false, false, false, 0);
  if (retval != ICM_20948_Stat_Ok)
  {
    // debugPrint(F("ICM_20948::startupMagnetometer: i2cMasterConfigurePeripheral returned: "));
    // debugPrintStatus(retval);
    // debugPrintln(F(""));
    status = retval;
    return status;
  }

  return status;
}

ICM_20948_Status_e i2cMasterPassthrough(const struct device *dev, bool passthrough)
{
  struct icm20948_data *drv_data = dev->data;
	ICM_20948_Device_t *driver = &(drv_data->driver);
  status = ICM_20948_i2c_master_passthrough(driver, passthrough);
  return status;
}

ICM_20948_Status_e i2cMasterEnable(const struct device *dev, bool enable)
{
  struct icm20948_data *drv_data = dev->data;
	ICM_20948_Device_t *driver = &(drv_data->driver);
  status = ICM_20948_i2c_master_enable(driver, enable);
  return status;
}

ICM_20948_Status_e i2cMasterReset(const struct device *dev)
{
  struct icm20948_data *drv_data = dev->data;
	ICM_20948_Device_t *driver = &(drv_data->driver);
  status = ICM_20948_i2c_master_reset(driver);
  return status;
}

ICM_20948_Status_e magWhoIAm(const struct device *dev)
{
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  uint8_t whoiam1, whoiam2;
  whoiam1 = readMag(dev, AK09916_REG_WIA1);
  // readMag calls i2cMasterSingleR which calls ICM_20948_i2c_master_single_r
  // i2cMasterSingleR updates status so it is OK to set retval to status here
  retval = status;
  if (retval != ICM_20948_Stat_Ok)
  {
    // debugPrint(F("ICM_20948::magWhoIAm: whoiam1: "));
    // debugPrintf((int)whoiam1);
    // debugPrint(F(" (should be 72) readMag set status to: "));
    // debugPrintStatus(status);
    // debugPrintln(F(""));
    return retval;
  }
  whoiam2 = readMag(dev, AK09916_REG_WIA2);
  // readMag calls i2cMasterSingleR which calls ICM_20948_i2c_master_single_r
  // i2cMasterSingleR updates status so it is OK to set retval to status here
  retval = status;
  if (retval != ICM_20948_Stat_Ok)
  {
    // debugPrint(F("ICM_20948::magWhoIAm: whoiam1: "));
    // debugPrintf((int)whoiam1);
    // debugPrint(F(" (should be 72) whoiam2: "));
    // debugPrintf((int)whoiam2);
    // debugPrint(F(" (should be 9) readMag set status to: "));
    // debugPrintStatus(status);
    // debugPrintln(F(""));
    return retval;
  }

  if ((whoiam1 == (MAG_AK09916_WHO_AM_I >> 8)) && (whoiam2 == (MAG_AK09916_WHO_AM_I & 0xFF)))
  {
    retval = ICM_20948_Stat_Ok;
    status = retval;
    return status;
  }

//   debugPrint(F("ICM_20948::magWhoIAm: whoiam1: "));
//   debugPrintf((int)whoiam1);
//   debugPrint(F(" (should be 72) whoiam2: "));
//   debugPrintf((int)whoiam2);
//   debugPrintln(F(" (should be 9). Returning ICM_20948_Stat_WrongID"));

  retval = ICM_20948_Stat_WrongID;
  status = retval;
  return status;
}

uint8_t readMag(const struct device *dev, AK09916_Reg_Addr_e reg)
{

  uint8_t data = i2cMasterSingleR(dev, MAG_AK09916_I2C_ADDR, reg); // i2cMasterSingleR updates status too
  return data;
}

uint8_t i2cMasterSingleR(const struct device *dev, uint8_t addr, uint8_t reg)
{
  uint8_t data = 0;
  struct icm20948_data *drv_data = dev->data;
	ICM_20948_Device_t *driver = &(drv_data->driver);
  status = ICM_20948_i2c_master_single_r(driver, addr, reg, &data);
  if (status != ICM_20948_Stat_Ok)
  {
    // debugPrint(F("ICM_20948::i2cMasterSingleR: ICM_20948_i2c_master_single_r returned: "));
    // debugPrintStatus(status);
    // debugPrintln(F(""));
  }
  return data;
}

ICM_20948_Status_e i2cControllerConfigurePeripheral(const struct device *dev, uint8_t peripheral, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap, uint8_t dataOut)
{
    struct icm20948_data *drv_data = dev->data;
	ICM_20948_Device_t *driver = &(drv_data->driver);
  status = ICM_20948_i2c_controller_configure_peripheral(driver, peripheral, addr, reg, len, Rw, enable, data_only, grp, swap, dataOut);
  return status;
}

ICM_20948_Status_e writeMag(const struct device *dev, AK09916_Reg_Addr_e reg, uint8_t *pdata)
{
  status = i2cMasterSingleW(dev, MAG_AK09916_I2C_ADDR, reg, *pdata);
  return status;
}

ICM_20948_Status_e i2cMasterSingleW(const struct device *dev, uint8_t addr, uint8_t reg, uint8_t data)
{
  struct icm20948_data *drv_data = dev->data;
	ICM_20948_Device_t *driver = &(drv_data->driver);
  status = ICM_20948_i2c_master_single_w(driver, addr, reg, &data);
  return status;
}

ICM_20948_Status_e resetMag(const struct device *dev)
{
  uint8_t SRST = 1;
  // SRST: Soft reset
  // “0”: Normal
  // “1”: Reset
  // When “1” is set, all registers are initialized. After reset, SRST bit turns to “0” automatically.
  status = i2cMasterSingleW(dev, MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL3, SRST);
  return status;
}
//#endif
