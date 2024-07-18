#ifndef SLIMENRF_SENSOR
#define SLIMENRF_SENSOR

void sensor_retained_read(void);
void sensor_retained_write_quat(void);
void sensor_retained_write_gOff(void);

void sensor_shutdown(void);
void sensor_setup_WOM(void);

void sensor_calibrate_imu(void);
void sensor_calibrate_mag(void);

void set_LN(void);
void set_LP(void);
void reconfigure_imu(const struct i2c_dt_spec imu);
void reconfigure_mag(const struct i2c_dt_spec mag);
bool wait_for_motion(const struct i2c_dt_spec imu, bool motion, int samples);

// TODO: make threads more abstract, pass in imus n stuff instead
void main_imu_init(void);
void main_imu_thread(void);
void wait_for_threads(void);
void main_imu_suspend(void);
void main_imu_wakeup(void);

#endif