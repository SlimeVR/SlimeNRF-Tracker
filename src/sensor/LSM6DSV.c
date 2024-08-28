#include <math.h>

#include <zephyr/drivers/i2c.h>

#include "LSM6DSV.h"

static const float accel_sensitivity = 16.0f / 32768.0f; // Always 16G (FS = ±16 g: 0.488 mg/LSB)
static const float gyro_sensitivity = 0.070f; // Always 2000dps (FS = ±2000 dps: 70 mdps/LSB)

static uint8_t last_accel_mode = 0xff;
static uint8_t last_gyro_mode = 0xff;
static uint8_t last_accel_odr = 0xff;
static uint8_t last_gyro_odr = 0xff;

int lsm_init(struct i2c_dt_spec dev_i2c, float clock_rate, float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
	i2c_reg_write_byte_dt(&dev_i2c, LSM6DSV_CTRL6, FS_G_2000DPS); // set gyro FS
	i2c_reg_write_byte_dt(&dev_i2c, LSM6DSV_CTRL8, FS_XL_16G); // set accel FS
	last_accel_odr = 0xff; // reset last odr
	last_gyro_odr = 0xff; // reset last odr
	int err = lsm_update_odr(dev_i2c, accel_time, gyro_time, accel_actual_time, gyro_actual_time);
	i2c_reg_write_byte_dt(&dev_i2c, LSM6DSV_FIFO_CTRL4, 0x01); // enable FIFO mode
	return (err < 0 ? 0 : err);
}

void lsm_shutdown(struct i2c_dt_spec dev_i2c)
{
	last_accel_odr = 0xff; // reset last odr
	last_gyro_odr = 0xff; // reset last odr
	i2c_reg_write_byte_dt(&dev_i2c, LSM6DSV_CTRL3, 0x01); // SW_RESET
}

int lsm_update_odr(struct i2c_dt_spec dev_i2c, float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
	int ODR;
	uint8_t OP_MODE_XL;
	uint8_t OP_MODE_G;
	uint8_t ODR_XL;
	uint8_t ODR_G;

	// Calculate accel
	if (accel_time <= 0 || accel_time == INFINITY) // off, standby interpreted as off
	{
		OP_MODE_XL = OP_MODE_XL_HP;
		ODR_XL = ODR_OFF;
		ODR = 0;
	}
	else
	{
		OP_MODE_XL = OP_MODE_XL_HP;
		ODR = 1 / accel_time;
	}

	if (ODR == 0)
	{
		accel_time = 0; // off
	}
	else if (ODR > 3840) // TODO: this is absolutely awful
	{
		ODR_XL = ODR_7_68kHz;
		accel_time = 1.0 / 7680;
	}
	else if (ODR > 1920)
	{
		ODR_XL = ODR_3_84kHz;
		accel_time = 1.0 / 3840;
	}
	else if (ODR > 960)
	{
		ODR_XL = ODR_1_92kHz;
		accel_time = 1.0 / 1920;
	}
	else if (ODR > 480)
	{
		ODR_XL = ODR_960Hz;
		accel_time = 1.0 / 960;
	}
	else if (ODR > 240)
	{
		ODR_XL = ODR_480Hz;
		accel_time = 1.0 / 480;
	}
	else if (ODR > 120)
	{
		ODR_XL = ODR_240Hz;
		accel_time = 1.0 / 240;
	}
	else if (ODR > 60)
	{
		ODR_XL = ODR_120Hz;
		accel_time = 1.0 / 120;
	}
	else if (ODR > 30)
	{
		ODR_XL = ODR_60Hz;
		accel_time = 1.0 / 60;
	}
	else if (ODR > 15)
	{
		ODR_XL = ODR_30Hz;
		accel_time = 1.0 / 30;
	}
	else if (ODR > 7.5)
	{
		ODR_XL = ODR_15Hz;
		accel_time = 1.0 / 15;
	}
	else if (ODR > 1.875)
	{
		ODR_XL = ODR_7_5Hz;
		accel_time = 1.0 / 7.5;
	}
	else
	{
		ODR_XL = ODR_1_875Hz;
		accel_time = 1.0 / 1.875;
	}

	// Calculate gyro
	if (gyro_time <= 0) // off
	{
		OP_MODE_G = OP_MODE_G_HP;
		ODR_G = ODR_OFF;
		ODR = 0;
	}
	else if (gyro_time == INFINITY) // sleep
	{
		OP_MODE_G = OP_MODE_G_SLEEP;
		ODR_G = last_gyro_odr; // using last ODR
		ODR = 0;
	}
	else
	{
		OP_MODE_G = OP_MODE_G_HP;
		ODR_G = 0; // the compiler complains unless I do this
		ODR = 1 / gyro_time;
	}

	if (ODR == 0)
	{
		gyro_time = 0; // off
	}
	else if (ODR > 3840) // TODO: this is absolutely awful
	{
		ODR_G = ODR_7_68kHz;
		gyro_time = 1.0 / 7680;
	}
	else if (ODR > 1920)
	{
		ODR_G = ODR_3_84kHz;
		gyro_time = 1.0 / 3840;
	}
	else if (ODR > 960)
	{
		ODR_G = ODR_1_92kHz;
		gyro_time = 1.0 / 1920;
	}
	else if (ODR > 480)
	{
		ODR_G = ODR_960Hz;
		gyro_time = 1.0 / 960;
	}
	else if (ODR > 240)
	{
		ODR_G = ODR_480Hz;
		gyro_time = 1.0 / 480;
	}
	else if (ODR > 120)
	{
		ODR_G = ODR_240Hz;
		gyro_time = 1.0 / 240;
	}
	else if (ODR > 60)
	{
		ODR_G = ODR_120Hz;
		gyro_time = 1.0 / 120;
	}
	else if (ODR > 30)
	{
		ODR_G = ODR_60Hz;
		gyro_time = 1.0 / 60;
	}
	else if (ODR > 15)
	{
		ODR_G = ODR_30Hz;
		gyro_time = 1.0 / 30;
	}
	else if (ODR > 7.5)
	{
		ODR_G = ODR_15Hz;
		gyro_time = 1.0 / 15;
	}
	else if (ODR > 1.875)
	{
		ODR_G = ODR_7_5Hz;
		gyro_time = 1.0 / 7.5;
	}
	else
	{
		ODR_G = ODR_1_875Hz;
		gyro_time = 1.0 / 1.875;
	}

	if (last_accel_mode == OP_MODE_XL && last_gyro_mode == OP_MODE_G && last_accel_odr == ODR_XL && last_gyro_odr == ODR_G) // if both were already configured
		return 1;

	last_accel_mode = OP_MODE_XL;
	last_gyro_mode = OP_MODE_G;
	last_accel_odr = ODR_XL;
	last_gyro_odr = ODR_G;

	i2c_reg_write_byte_dt(&dev_i2c, LSM6DSV_CTRL1, OP_MODE_XL << 4 | ODR_XL); // set accel ODR and mode
	i2c_reg_write_byte_dt(&dev_i2c, LSM6DSV_CTRL2, OP_MODE_G << 4 | ODR_G); // set gyro ODR and mode

	i2c_reg_write_byte_dt(&dev_i2c, LSM6DSV_FIFO_CTRL3, 0x00 << 0 | ODR_G << 4); // set accel Not batched, set gyro batch rate

	*accel_actual_time = accel_time;
	*gyro_actual_time = gyro_time;

	return 0;
}

uint16_t lsm_fifo_read(struct i2c_dt_spec dev_i2c, uint8_t *data)
{
	uint8_t rawCount[2];
	i2c_burst_read_dt(&dev_i2c, LSM6DSV_FIFO_STATUS1, &rawCount[0], 2);
	uint16_t count = (uint16_t)((rawCount[1] & 1) << 8 | rawCount[0]); // Turn the 16 bits into a unsigned 16-bit value
	count += 4; // Add a few read buffer packets, since the FIFO may contain more data than when we begin reading (allowing 4ms of transaction time for 1000hz ODR)
	for (int i = 0; i < count; i++)
		i2c_burst_read_dt(&dev_i2c, LSM6DSV_FIFO_DATA_OUT_TAG, &data[i * 7], 7); // Packet size is always 7 bytes
	return count;
}

int lsm_fifo_process(uint16_t index, uint8_t *data, float g[3])
{
	index *= 7; // Packet size 7 bytes
	if (data[index] != 0x01)
		return 1; // Ignore all packets except Gyroscope NC (Gyroscope uncompressed data)
	// combine into 16 bit values
	float raw[3];
	for (int i = 0; i < 3; i++) { // x, y, z
		raw[i] = (int16_t)((((int16_t)data[index + (i * 2) + 2]) << 8) | data[index + (i * 2) + 1]);
		raw[i] *= gyro_sensitivity;
	}
	// TODO: need to skip invalid data
	memcpy(g, raw, sizeof(raw));
	return 0;
}

void lsm_accel_read(struct i2c_dt_spec dev_i2c, float a[3])
{
	uint8_t rawAccel[6];
	i2c_burst_read_dt(&dev_i2c, LSM6DSV_OUTX_L_A, &rawAccel[0], 6);
	float accel_x = (int16_t)((((int16_t)rawAccel[1]) << 8) | rawAccel[0]);
	float accel_y = (int16_t)((((int16_t)rawAccel[3]) << 8) | rawAccel[2]);
	float accel_z = (int16_t)((((int16_t)rawAccel[5]) << 8) | rawAccel[4]);
	a[0] = accel_x * accel_sensitivity;
	a[1] = accel_y * accel_sensitivity;
	a[2] = accel_z * accel_sensitivity;
}

void lsm_gyro_read(struct i2c_dt_spec dev_i2c, float g[3])
{
	uint8_t rawGyro[6];
	i2c_burst_read_dt(&dev_i2c, LSM6DSV_OUTX_L_G, &rawGyro[0], 6);
	float gyro_x = (int16_t)((((int16_t)rawGyro[1]) << 8) | rawGyro[0]);
	float gyro_y = (int16_t)((((int16_t)rawGyro[3]) << 8) | rawGyro[2]);
	float gyro_z = (int16_t)((((int16_t)rawGyro[5]) << 8) | rawGyro[4]);
	g[0] = gyro_x * gyro_sensitivity;
	g[1] = gyro_y * gyro_sensitivity;
	g[2] = gyro_z * gyro_sensitivity;
}

float lsm_temp_read(struct i2c_dt_spec dev_i2c)
{
	uint8_t rawTemp[2];
	i2c_burst_read_dt(&dev_i2c, LSM6DSV_OUT_TEMP_L, &rawTemp[0], 2);
	// TSen Temperature sensitivity 256 LSB/°C
	// The output of the temperature sensor is 0 LSB (typ.) at 25°C
	float temp = (int16_t)((((int16_t)rawTemp[1]) << 8) | rawTemp[0]);
	temp /= 256;
	temp += 25;
	return temp;
}

void lsm_setup_WOM(struct i2c_dt_spec dev_i2c)
{ // TODO: should be off by the time WOM will be setup
//	i2c_reg_write_byte_dt(&dev_i2c, LSM6DSV_CTRL1, ODR_OFF); // set accel off
//	i2c_reg_write_byte_dt(&dev_i2c, LSM6DSV_CTRL2, ODR_OFF); // set gyro off

	i2c_reg_write_byte_dt(&dev_i2c, LSM6DSV_TAP_CFG0, 0x10); // use HPF for wake-up
	i2c_reg_write_byte_dt(&dev_i2c, LSM6DSV_WAKE_UP_THS, 0x28); // set threshold, 40 * 7.8125 mg is ~312 mg
	i2c_reg_write_byte_dt(&dev_i2c, LSM6DSV_MD1_CFG, 0x20); // route wake-up to INT1
	i2c_reg_write_byte_dt(&dev_i2c, LSM6DSV_FUNCTIONS_ENABLE, 0x80); // enable interrupts

	i2c_reg_write_byte_dt(&dev_i2c, LSM6DSV_CTRL8, FS_XL_8G); // set accel FS
	i2c_reg_write_byte_dt(&dev_i2c, LSM6DSV_CTRL1, OP_MODE_XL_LP1 << 4 | ODR_240Hz); // set accel low power mode 1, set accel ODR (enable accel)
}

const sensor_imu_t sensor_imu_lsm6dsv = {
	*lsm_init,
	*lsm_shutdown,

	*lsm_update_odr,

	*lsm_fifo_read,
	*lsm_fifo_process,
	*lsm_accel_read,
	*lsm_gyro_read,
	*lsm_temp_read,

	*lsm_setup_WOM
};
