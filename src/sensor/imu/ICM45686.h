#ifndef ICM45686_h
#define ICM45686_h

#include "../../sensor.h"

// https://invensense.tdk.com/wp-content/uploads/documentation/DS-000577_ICM-45686.pdf
// https://invensense.tdk.com/wp-content/uploads/2024/07/AN-000478_ICM-45605-ICM-45686-User-Guide.pdf

// User Bank 0
#define ICM45686_ACCEL_DATA_X1_UI          0x00
#define ICM45686_GYRO_DATA_X1_UI           0x06

#define ICM45686_TEMP_DATA1_UI             0x0C

#define ICM45686_PWR_MGMT0                 0x10

#define ICM45686_FIFO_COUNT_0              0x12
#define ICM45686_FIFO_DATA                 0x14

#define ICM45686_INT1_CONFIG0              0x16
#define ICM45686_INT1_CONFIG1              0x17

#define ICM45686_INT1_STATUS0              0x19

#define ICM45686_ACCEL_CONFIG0             0x1B
#define ICM45686_GYRO_CONFIG0              0x1C

#define ICM45686_FIFO_CONFIG0              0x1D
#define ICM45686_FIFO_CONFIG3              0x21

#define ICM45686_TMST_WOM_CONFIG           0x23

#define ICM45686_RTC_CONFIG                0x26
#define ICM45686_IOC_PAD_SCENARIO_OVRD     0x31

#define ICM45686_REG_MISC1                 0x35

#define ICM45686_IREG_ADDR_15_8            0x7C
#define ICM45686_IREG_DATA                 0x7E

#define ICM45686_REG_MISC2                 0x7F

// User Bank IPREG_TOP1
#define ICM45686_IPREG_TOP1              0xA200

#define ICM45686_SMC_CONTROL_0             0x58

#define ICM45686_ACCEL_WOM_X_THR           0x7E
#define ICM45686_ACCEL_WOM_Y_THR           0x7F
#define ICM45686_ACCEL_WOM_Z_THR           0x80

// User Bank IPREG_SYS2
#define ICM45686_IPREG_SYS2              0xA500

#define ICM45686_IPREG_SYS2_REG_129        0x81

/*
Burst-write and burst-read operations are not supported when accessing IREGs from the host.
The minimum time gap between two consecutive IREG accesses for various IREG components is 4μs.
1. The host specifies the destination address of an IREG by programming IREG_ADDR_7_0,
IREG_ADDR_15_8.
d. If host wants to access a register in IPREG_SYS2, it should add base address 0xA500 to the
address of that register shown in the IPREG_SYS2 registers section, and then use that resulting
value in registers IREG_ADDR_7_0, IREG_ADDR_15_8
e. If host wants to access a register in IPREG_TOP1, it should add base address 0xA200 to the
address of that register shown in the IPREG_TOP1 registers section, and then use that resulting
value in registers IREG_ADDR_7_0, IREG_ADDR_15_8
2. The host programs the write data to the IREG_DATA register.
3. The above programming steps must be performed in a single burst-write transaction to prevent an un-
intended read-pre-fetch operation
5. After the contents from the IREG_DATA register is written to the selected register, the internal 16-bit
address is auto-incremented.
6. After a minimum wait time-gap, the host can write to the IREG_DATA register again, which is effectively
writing to the register pointed by the post-auto-incremented address.
*/

#define GYRO_MODE_OFF     0x00
#define GYRO_MODE_STANDBY 0x01
#define GYRO_MODE_LP      0x02
#define GYRO_MODE_LN      0x03

#define ACCEL_MODE_OFF 0x01
#define ACCEL_MODE_LP  0x02
#define ACCEL_MODE_LN  0x03

#define ACCEL_UI_FS_SEL_32G 0x00
#define ACCEL_UI_FS_SEL_16G 0x01
#define ACCEL_UI_FS_SEL_8G  0x02
#define ACCEL_UI_FS_SEL_4G  0x03
#define ACCEL_UI_FS_SEL_2G  0x04

// Low Noise mode
#define ACCEL_ODR_6_4kHz   0x03
#define ACCEL_ODR_3_2kHz   0x04
#define ACCEL_ODR_1_6kHz   0x05
#define ACCEL_ODR_800Hz    0x06
// Low Noise or Low Power modes
#define ACCEL_ODR_400Hz    0x07
#define ACCEL_ODR_200Hz    0x08
#define ACCEL_ODR_100Hz    0x09
#define ACCEL_ODR_50Hz     0x0A
#define ACCEL_ODR_25Hz     0x0B
#define ACCEL_ODR_12_5Hz   0x0C
// Low Power mode
#define ACCEL_ODR_6_25Hz   0x0D
#define ACCEL_ODR_3_125Hz  0x0E
#define ACCEL_ODR_1_5625Hz 0x0F

#define GYRO_UI_FS_SEL_4000DPS   0x00
#define GYRO_UI_FS_SEL_2000DPS   0x01
#define GYRO_UI_FS_SEL_1000DPS   0x02
#define GYRO_UI_FS_SEL_500DPS    0x03
#define GYRO_UI_FS_SEL_250DPS    0x04
#define GYRO_UI_FS_SEL_125DPS    0x05
#define GYRO_UI_FS_SEL_62_5DPS   0x06
#define GYRO_UI_FS_SEL_31_25DPS  0x07
#define GYRO_UI_FS_SEL_15_625DPS 0x08

// Low Noise mode
#define GYRO_ODR_6_4kHz   0x03
#define GYRO_ODR_3_2kHz   0x04
#define GYRO_ODR_1_6kHz   0x05
#define GYRO_ODR_800Hz    0x06
// Low Noise or Low Power modes
#define GYRO_ODR_400Hz    0x07
#define GYRO_ODR_200Hz    0x08
#define GYRO_ODR_100Hz    0x09
#define GYRO_ODR_50Hz     0x0A
#define GYRO_ODR_25Hz     0x0B
#define GYRO_ODR_12_5Hz   0x0C
// Low Power mode
#define GYRO_ODR_6_25Hz   0x0D
#define GYRO_ODR_3_125Hz  0x0E
#define GYRO_ODR_1_5625Hz 0x0F

int icm45_init(const struct i2c_dt_spec *dev_i2c, float clock_rate, float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time);
void icm45_shutdown(const struct i2c_dt_spec *dev_i2c);

int icm45_update_odr(const struct i2c_dt_spec *dev_i2c, float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time);

uint16_t icm45_fifo_read(const struct i2c_dt_spec *dev_i2c, uint8_t *data);
int icm45_fifo_process(uint16_t index, uint8_t *data, float g[3]);
void icm45_accel_read(const struct i2c_dt_spec *dev_i2c, float a[3]);
void icm45_gyro_read(const struct i2c_dt_spec *dev_i2c, float g[3]);
float icm45_temp_read(const struct i2c_dt_spec *dev_i2c);

void icm45_setup_WOM(const struct i2c_dt_spec *dev_i2c);

extern const sensor_imu_t sensor_imu_icm45686;

#endif
