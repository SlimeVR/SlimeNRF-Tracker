#ifndef LSM6DSV_h
#define LSM6DSV_h

#include "../sensor.h"

// https://www.st.com/resource/en/datasheet/lsm6dsv.pdf
#define LSM6DSV_FUNC_CFG_ACCESS            0x01
#define LSM6DSV_PIN_CTRL                   0x02
#define LSM6DSV_IF_CFG                     0x03
#define LSM6DSV_ODR_TRIG_CFG               0x06
#define LSM6DSV_FIFO_CTRL1                 0x07
#define LSM6DSV_FIFO_CTRL2                 0x08
#define LSM6DSV_FIFO_CTRL3                 0x09
#define LSM6DSV_FIFO_CTRL4                 0x0A
#define LSM6DSV_COUNTER_BDR_REG1           0x0B
#define LSM6DSV_COUNTER_BDR_REG2           0x0C
#define LSM6DSV_INT1_CTRL                  0x0D
#define LSM6DSV_INT2_CTRL                  0x0E
#define LSM6DSV_WHO_AM_I                   0x0F
#define LSM6DSV_CTRL1                      0x10
#define LSM6DSV_CTRL2                      0x11
#define LSM6DSV_CTRL3                      0x12
#define LSM6DSV_CTRL4                      0x13
#define LSM6DSV_CTRL5                      0x14
#define LSM6DSV_CTRL6                      0x15
#define LSM6DSV_CTRL7                      0x16
#define LSM6DSV_CTRL8                      0x17
#define LSM6DSV_CTRL9                      0x18
#define LSM6DSV_CTRL10                     0x19
#define LSM6DSV_CTRL_STATUS                0x1A
#define LSM6DSV_FIFO_STATUS1               0x1B
#define LSM6DSV_FIFO_STATUS2               0x1C
#define LSM6DSV_ALL_INT_SRC                0x1D
#define LSM6DSV_STATUS_REG                 0x1E
#define LSM6DSV_OUT_TEMP_L                 0x20
#define LSM6DSV_OUT_TEMP_H                 0x21
#define LSM6DSV_OUTX_L_G                   0x22
#define LSM6DSV_OUTX_H_G                   0x23
#define LSM6DSV_OUTY_L_G                   0x24
#define LSM6DSV_OUTY_H_G                   0x25
#define LSM6DSV_OUTZ_L_G                   0x26
#define LSM6DSV_OUTZ_H_G                   0x27
#define LSM6DSV_OUTX_L_A                   0x28
#define LSM6DSV_OUTX_H_A                   0x29
#define LSM6DSV_OUTY_L_A                   0x2A
#define LSM6DSV_OUTY_H_A                   0x2B
#define LSM6DSV_OUTZ_L_A                   0x2C
#define LSM6DSV_OUTZ_H_A                   0x2D
#define LSM6DSV_UI_OUTX_L_G_OIS_EIS        0x2E
#define LSM6DSV_UI_OUTX_H_G_OIS_EIS        0x2F
#define LSM6DSV_UI_OUTY_L_G_OIS_EIS        0x30
#define LSM6DSV_UI_OUTY_H_G_OIS_EIS        0x31
#define LSM6DSV_UI_OUTZ_L_G_OIS_EIS        0x32
#define LSM6DSV_UI_OUTZ_H_G_OIS_EIS        0x33
#define LSM6DSV_UI_OUTX_L_A_OIS_DualC      0x34
#define LSM6DSV_UI_OUTX_H_A_OIS_DualC      0x35
#define LSM6DSV_UI_OUTY_L_A_OIS_DualC      0x36
#define LSM6DSV_UI_OUTY_H_A_OIS_DualC      0x37
#define LSM6DSV_UI_OUTZ_L_A_OIS_DualC      0x38
#define LSM6DSV_UI_OUTZ_H_A_OIS_DualC      0x39
#define LSM6DSV_TIMESTAMP0                 0x40
#define LSM6DSV_TIMESTAMP1                 0x41
#define LSM6DSV_TIMESTAMP2                 0x42
#define LSM6DSV_TIMESTAMP3                 0x43
#define LSM6DSV_UI_STATUS_REG_OIS          0x44
#define LSM6DSV_WAKE_UP_SRC                0x45
#define LSM6DSV_TAP_SRC                    0x46
#define LSM6DSV_D6D_SRC                    0x47
#define LSM6DSV_STATUS_MASTER_MAINPAGE     0x48
#define LSM6DSV_EMB_FUNC_STATUS_MAINPAGE   0x49
#define LSM6DSV_FSM_STATUS_MAINPAGE        0x4A
#define LSM6DSV_INTERNAL_FREQ_FINE         0x4F
#define LSM6DSV_FUNCTIONS_ENABLE           0x50
#define LSM6DSV_DEN                        0x51
#define LSM6DSV_INACTIVITY_DUR             0x54
#define LSM6DSV_INACTIVITY_THS             0x55
#define LSM6DSV_TAP_CFG0                   0x56
#define LSM6DSV_TAP_CFG1                   0x57
#define LSM6DSV_TAP_CFG2                   0x58
#define LSM6DSV_TAP_THS_6D                 0x59
#define LSM6DSV_TAP_DUR                    0x5A
#define LSM6DSV_WAKE_UP_THS                0x5B
#define LSM6DSV_WAKE_UP_DUR                0x5C
#define LSM6DSV_FREE_FALL                  0x5D
#define LSM6DSV_MD1_CFG                    0x5E
#define LSM6DSV_MD2_CFG                    0x5F
#define LSM6DSV_HAODR_CFG                  0x62
#define LSM6DSV_EMB_FUNC_CFG               0x63
#define LSM6DSV_UI_HANDSHAKE_CTRL          0x64
#define LSM6DSV_UI_SPI2_SHARED_0           0x65
#define LSM6DSV_UI_SPI2_SHARED_1           0x66
#define LSM6DSV_UI_SPI2_SHARED_2           0x67
#define LSM6DSV_UI_SPI2_SHARED_3           0x68
#define LSM6DSV_UI_SPI2_SHARED_4           0x69
#define LSM6DSV_UI_SPI2_SHARED_5           0x6A
#define LSM6DSV_CTRL_EIS                   0x6B
#define LSM6DSV_UI_INT_OIS                 0x6F
#define LSM6DSV_UI_CTRL1_OIS               0x70
#define LSM6DSV_UI_CTRL2_OIS               0x71
#define LSM6DSV_UI_CTRL3_OIS               0x72
#define LSM6DSV_X_OFS_USR                  0x73
#define LSM6DSV_Y_OFS_USR                  0x74
#define LSM6DSV_Z_OFS_USR                  0x75
#define LSM6DSV_FIFO_DATA_OUT_TAG          0x78
#define LSM6DSV_FIFO_DATA_OUT_X_L          0x79
#define LSM6DSV_FIFO_DATA_OUT_X_H          0x7A
#define LSM6DSV_FIFO_DATA_OUT_Y_L          0x7B
#define LSM6DSV_FIFO_DATA_OUT_Y_H          0x7C
#define LSM6DSV_FIFO_DATA_OUT_Z_L          0x7D
#define LSM6DSV_FIFO_DATA_OUT_Z_H          0x7E

// Same for XL and G
#define ODR_OFF     0x00
#define ODR_1_875Hz 0x01
#define ODR_7_5Hz   0x02
#define ODR_15Hz    0x03
#define ODR_30Hz    0x04
#define ODR_60Hz    0x05
#define ODR_120Hz   0x06
#define ODR_240Hz   0x07
#define ODR_480Hz   0x08
#define ODR_960Hz   0x09
#define ODR_1_92kHz 0x0A
#define ODR_3_84kHz 0x0B
#define ODR_7_68kHz 0x0C

#define OP_MODE_XL_HP     0x00 // High Performance
#define OP_MODE_XL_HA     0x01 // High Accuracy
#define OP_MODE_XL_ODR_T  0x03 // ODR-Triggered
#define OP_MODE_XL_LP1    0x04 // Low Power mode 1 (2 mean)
#define OP_MODE_XL_LP2    0x05 // Low Power mode 2 (4 mean)
#define OP_MODE_XL_LP3    0x06 // Low Power mode 3 (8 mean)
#define OP_MODE_XL_NORMAL 0x07

#define OP_MODE_G_HP    0x00 // High Performance
#define OP_MODE_G_HA    0x01 // High Accuracy
#define OP_MODE_G_SLEEP 0x04
#define OP_MODE_G_LP    0x05 // Low Power

#define FS_G_125DPS  0x00
#define FS_G_250DPS  0x01
#define FS_G_500DPS  0x02
#define FS_G_1000DPS 0x03
#define FS_G_2000DPS 0x04
#define FS_G_4000DPS 0x0C

#define FS_XL_2G  0x00
#define FS_XL_4G  0x01
#define FS_XL_8G  0x02
#define FS_XL_16G 0x03

int lsm_init(struct i2c_dt_spec dev_i2c, float clock_rate, float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time);
void lsm_shutdown(struct i2c_dt_spec dev_i2c);

int lsm_update_odr(struct i2c_dt_spec dev_i2c, float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time);

uint16_t lsm_fifo_read(struct i2c_dt_spec dev_i2c, uint8_t *data);
int lsm_fifo_process(uint16_t index, uint8_t *data, float g[3]);
void lsm_accel_read(struct i2c_dt_spec dev_i2c, float a[3]);
void lsm_gyro_read(struct i2c_dt_spec dev_i2c, float g[3]);
float lsm_temp_read(struct i2c_dt_spec dev_i2c);

void lsm_setup_WOM(struct i2c_dt_spec dev_i2c);

extern const sensor_imu_t sensor_imu_lsm6dsv;

#endif
