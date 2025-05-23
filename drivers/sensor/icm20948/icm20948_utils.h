#ifndef KALMAN_FILTER_APPLICATION_DRIVERS_SENSOR_ICM20948_ICM20948_UTILS_H_ 
#define KALMAN_FILTER_APPLICATION_DRIVERS_SENSOR_ICM20949_ICM20948_UTILS_H_

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include "util/ICM_20948_C.h"
#include "util/AK09916_REGISTERS.h"
#include <zephyr/sys/printk.h>

#define I2C_PORT_DEFAULT            DEVICE_DT_GET_ONE(invensense_icm20948)
#define ICM_20948_ADDR_DEFAULT      0x69
#define MAX_MAGNETOMETER_STARTS     10

#define STACKSIZE	1024
#define PRIORITY 7

#define WAIT_FOR_USB_DTR            true   // Wait for DTR signal before starting the application

#define I2C_PORT                    DEVICE_DT_GET_ONE(invensense_icm20948) // I2C port

#define ICM20948_I2C_ADDR			0x69
#define REG_BANK_SEL_USER_BANK_0	0x00
#define REG_BANK_SEL_USER_BANK_1	0x10
#define REG_BANK_SEL_USER_BANK_2	0x20
#define REG_BANK_SEL_USER_BANK_3	0x30

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

ICM_20948_Status_e ICM_20948_init(ICM_20948_Device_t *_ICM, struct device *_i2c_dev, uint8_t _imu_i2c_addr);
/* icm20948_i2c.h*/
ICM_20948_Status_e my_write_i2c(uint8_t reg, uint8_t *data, uint32_t len, void *user);
ICM_20948_Status_e my_read_i2c(uint8_t reg, uint8_t *buff, uint32_t len, void *user);

ICM_20948_Status_e startupMagnetometer(bool minimal);
ICM_20948_Status_e i2cMasterPassthrough(bool passthrough);
ICM_20948_Status_e i2cMasterEnable(bool enable);
ICM_20948_Status_e i2cMasterReset();
ICM_20948_Status_e magWhoIAm();
uint8_t readMag(AK09916_Reg_Addr_e reg);
uint8_t i2cMasterSingleR(uint8_t addr, uint8_t reg);
ICM_20948_Status_e writeMag(AK09916_Reg_Addr_e reg, uint8_t *pdata);
ICM_20948_Status_e i2cMasterSingleW(uint8_t addr, uint8_t reg, uint8_t data);
ICM_20948_Status_e i2cControllerConfigurePeripheral(uint8_t peripheral, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap, uint8_t dataOut);
void ICM_20948_set_FSS(ICM_20948_fss_t _fss);
ICM_20948_fss_t ICM_20948_get_FSS();

ICM_20948_Status_e resetMag();

#endif /* __SENSOR_ICM42605_ICM20948_UTILS__ */