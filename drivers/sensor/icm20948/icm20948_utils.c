#include "icm20948_utils.h"

struct device *i2c_dev = DEVICE_DT_GET_ONE(invensense_icm20948);
uint8_t imu_i2c_addr = ICM_20948_ADDR_DEFAULT;
ICM_20948_Device_t *ICM;
ICM_20948_fss_t ICM_20948_fss;

ICM_20948_Status_e status;

const ICM_20948_Serif_t _Serif = {
    my_write_i2c, // write
    my_read_i2c,  // read
    NULL,
};

ICM_20948_Status_e ICM_20948_init(ICM_20948_Device_t *_ICM, struct device *_i2c_dev, uint8_t _imu_i2c_addr)
{
    ICM_20948_Status_e ret = ICM_20948_Stat_Err;

    ICM = _ICM;

    if (NULL != _i2c_dev)
    {
        i2c_dev = _i2c_dev;
    }

    i2c_configure(i2c_dev, I2C_SPEED_SET(I2C_SPEED_STANDARD));

    if (!i2c_dev) {
        // LOG_ERR("I2C device not valid.");
        return ICM_20948_Stat_Err;
    }

    if (0 != _imu_i2c_addr)
    {
        imu_i2c_addr = _imu_i2c_addr;
    }

	// Initialize ICM
	ICM_20948_init_struct(ICM);
  // LOG_INF("Init Struct OK!!!!!!!!!!\n");
	// Link the serif
	ICM_20948_link_serif(ICM, &_Serif);
  // LOG_INF("Link Serif OK!!!!!!!!!!\n");
	while (ICM_20948_check_id(ICM) != ICM_20948_Stat_Ok)
	{
    //TODO add a timeout or max retries
		// LOG_INF("whoami does not match. Halting...");
		k_sleep(K_SECONDS(1));
	}
  // LOG_INF("Check ID OK!!!!!!!!!!\n");
	// Here we are doing a SW reset to make sure the device starts in a known state
	ICM_20948_sw_reset(ICM);   

  k_sleep(K_MSEC(250));

	// Now wake the sensor up
	ICM_20948_sleep(ICM, false);
  LOG_INF("Sleep OK!!!!!!!!!!\n");
	ICM_20948_low_power(ICM, false);
  LOG_INF("Low Power OK!!!!!!!!!!\n");
	// Start the magnetometer
	startupMagnetometer(false);

    // Set Gyro and Accelerometer to a particular sample mode
	ICM_20948_set_sample_mode(ICM, (ICM_20948_InternalSensorID_bm)(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous); // optiona: ICM_20948_Sample_Mode_Continuous. ICM_20948_Sample_Mode_Cycled

	ICM_20948_set_full_scale(ICM, (ICM_20948_InternalSensorID_bm)(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_fss);

	// Set up DLPF configuration
	ICM_20948_dlpcfg_t myDLPcfg;
	myDLPcfg.a = acc_d473bw_n499bw;
	myDLPcfg.g = gyr_d361bw4_n376bw5;
	ICM_20948_set_dlpf_cfg(ICM, (ICM_20948_InternalSensorID_bm)(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);

  ICM_20948_smplrt_t mySMPLRTcfg;
  mySMPLRTcfg.a = 1;
  mySMPLRTcfg.g = 1;
  ICM_20948_set_sample_rate(ICM, (ICM_20948_InternalSensorID_bm)(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySMPLRTcfg);
  
	// Choose whether or not to use DLPF
	ICM_20948_enable_dlpf(ICM, ICM_20948_Internal_Acc, false);
	ICM_20948_enable_dlpf(ICM, ICM_20948_Internal_Gyr, false);

    ret = ICM_20948_Stat_Ok;

    return ret;
}

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

void ICM_20948_set_FSS(ICM_20948_fss_t _fss)
{
    ICM_20948_fss = _fss;
}

ICM_20948_fss_t ICM_20948_get_FSS()
{
    return ICM_20948_fss;
}

ICM_20948_Status_e startupMagnetometer(bool minimal)
{
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  i2cMasterPassthrough(false); //Do not connect the SDA/SCL pins to AUX_DA/AUX_CL
  i2cMasterEnable(true);

  resetMag();

  //After a ICM reset the Mag sensor may stop responding over the I2C master
  //Reset the Master I2C until it responds
  uint8_t tries = 0;
  while (tries < MAX_MAGNETOMETER_STARTS)
  {
    tries++;

    //See if we can read the WhoIAm register correctly
    retval = magWhoIAm();
    if (retval == ICM_20948_Stat_Ok)
      break; //WIA matched!

    i2cMasterReset(); //Otherwise, reset the master I2C and try again

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
  retval = writeMag(AK09916_REG_CNTL2, (uint8_t *)&reg);
  if (retval != ICM_20948_Stat_Ok)
  {
    // debugPrint(F("ICM_20948::startupMagnetometer: writeMag returned: "));
    // debugPrintStatus(retval);
    // debugPrintln(F(""));
    status = retval;
    return status;
  }

  retval = i2cControllerConfigurePeripheral(0, MAG_AK09916_I2C_ADDR, AK09916_REG_ST1, 9, true, true, false, false, false, 0);
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

ICM_20948_Status_e i2cMasterPassthrough(bool passthrough)
{
  status = ICM_20948_i2c_master_passthrough(ICM, passthrough);
  return status;
}

ICM_20948_Status_e i2cMasterEnable(bool enable)
{
  status = ICM_20948_i2c_master_enable(ICM, enable);
  return status;
}

ICM_20948_Status_e i2cMasterReset()
{
  status = ICM_20948_i2c_master_reset(ICM);
  return status;
}

ICM_20948_Status_e magWhoIAm()
{
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  uint8_t whoiam1, whoiam2;
  whoiam1 = readMag(AK09916_REG_WIA1);
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
  whoiam2 = readMag(AK09916_REG_WIA2);
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

uint8_t readMag(AK09916_Reg_Addr_e reg)
{
  uint8_t data = i2cMasterSingleR(MAG_AK09916_I2C_ADDR, reg); // i2cMasterSingleR updates status too
  return data;
}

uint8_t i2cMasterSingleR(uint8_t addr, uint8_t reg)
{
  uint8_t data = 0;
  status = ICM_20948_i2c_master_single_r(ICM, addr, reg, &data);
  if (status != ICM_20948_Stat_Ok)
  {
    // debugPrint(F("ICM_20948::i2cMasterSingleR: ICM_20948_i2c_master_single_r returned: "));
    // debugPrintStatus(status);
    // debugPrintln(F(""));
  }
  return data;
}

ICM_20948_Status_e i2cControllerConfigurePeripheral(uint8_t peripheral, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap, uint8_t dataOut)
{
  status = ICM_20948_i2c_controller_configure_peripheral(ICM, peripheral, addr, reg, len, Rw, enable, data_only, grp, swap, dataOut);
  return status;
}

ICM_20948_Status_e writeMag(AK09916_Reg_Addr_e reg, uint8_t *pdata)
{
  status = i2cMasterSingleW(MAG_AK09916_I2C_ADDR, reg, *pdata);
  return status;
}

ICM_20948_Status_e i2cMasterSingleW(uint8_t addr, uint8_t reg, uint8_t data)
{
  status = ICM_20948_i2c_master_single_w(ICM, addr, reg, &data);
  return status;
}

ICM_20948_Status_e resetMag()
{
  uint8_t SRST = 1;
  // SRST: Soft reset
  // “0”: Normal
  // “1”: Reset
  // When “1” is set, all registers are initialized. After reset, SRST bit turns to “0” automatically.
  status = i2cMasterSingleW(MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL3, SRST);
  return status;
}