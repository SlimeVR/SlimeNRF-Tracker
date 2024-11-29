#include "globals.h"
#include "sensor.h"
#include "connection.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>
#include <hal/nrf_gpio.h>

#include "system.h"

static struct nvs_fs fs;

#define NVS_PARTITION		storage_partition
#define NVS_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(NVS_PARTITION)

LOG_MODULE_REGISTER(system, LOG_LEVEL_INF);

#if DT_NODE_HAS_PROP(DT_ALIAS(sw0), gpios) // Alternate button if available to use as "reset key"
#define BUTTON_EXISTS true
static void button_thread(void);
K_THREAD_DEFINE(button_thread_id, 256, button_thread, NULL, NULL, NULL, 6, 0, 0);
#else
#pragma message "Button GPIO does not exist"
#endif

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)
#define CLKOUT_NODE DT_NODELABEL(pwmclock)

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dock_gpios)
#define DOCK_EXISTS true
static const struct gpio_dt_spec dock = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, dock_gpios);
#else
#pragma message "Dock sense GPIO does not exist"
#endif
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, chg_gpios)
#define CHG_EXISTS true
static const struct gpio_dt_spec chg = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, chg_gpios);
#else
#pragma message "Charge sense GPIO does not exist"
#endif
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, stby_gpios)
#define STBY_EXISTS true
static const struct gpio_dt_spec stby = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, stby_gpios);
#else
#pragma message "Standby sense GPIO does not exist"
#endif

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, clk_gpios)
#define CLK_EN_EXISTS true
static const struct gpio_dt_spec clk_en = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, clk_gpios);
static const struct pwm_dt_spec clk_out = {0};
#elif DT_NODE_HAS_PROP(CLKOUT_NODE, pwms)
#define CLK_OUT_EXISTS true
static const struct pwm_dt_spec clk_out = PWM_DT_SPEC_GET(CLKOUT_NODE);
#else
#pragma message "Clock enable GPIO or clock PWM out does not exist"
static const struct pwm_dt_spec clk_out = {0};
#endif

void configure_sense_pins(void)
{
	// Configure dock interrupt
#if DOCK_EXISTS
	if (dock_read())
	{
		nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_NOPULL); // Still works
		nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_SENSE_HIGH);
	}
	else
	{
		nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_PULLUP); // Still works
		nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_SENSE_LOW);
	}
	LOG_INF("Configured dock interrupt");
#endif
	// Configure chgstat interrupt
#if CHG_EXISTS
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, chg_gpios), NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, chg_gpios), chg_read() ? NRF_GPIO_PIN_SENSE_HIGH : NRF_GPIO_PIN_SENSE_LOW);
	LOG_INF("Configured chg interrupt");
#endif
#if STBY_EXISTS
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, stby_gpios), NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, stby_gpios), stby_read() ? NRF_GPIO_PIN_SENSE_HIGH : NRF_GPIO_PIN_SENSE_LOW);
	LOG_INF("Configured stby interrupt");
#endif
	// Configure sw0 interrupt
#if BUTTON_EXISTS // Alternate button if available to use as "reset key"
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(DT_ALIAS(sw0), gpios), NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(DT_ALIAS(sw0), gpios), NRF_GPIO_PIN_SENSE_LOW);
	LOG_INF("Configured sw0 interrupt");
#endif
}

static bool nvs_init = false;

static inline void sys_nvs_init(void)
{
	if (nvs_init)
		return;
	struct flash_pages_info info;
	fs.flash_device = NVS_PARTITION_DEVICE;
	fs.offset = NVS_PARTITION_OFFSET; // Start NVS FS here
	flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
	fs.sector_size = info.size; // Sector size equal to page size
	fs.sector_count = 4U; // 4 sectors
	nvs_mount(&fs);
	nvs_init = true;
}

static int sys_retained_init(void)
{
	bool ram_retention = retained_validate(); // Check ram retention
	// All contents of NVS was stored in RAM to not need initializing NVS often
	if (!ram_retention)
	{ 
		LOG_WRN("Invalidated RAM");
		sys_nvs_init();
		// read from nvs to retained
		nvs_read(&fs, PAIRED_ID, &retained.paired_addr, sizeof(retained.paired_addr));
		nvs_read(&fs, MAIN_ACCEL_BIAS_ID, &retained.accelBias, sizeof(retained.accelBias));
		nvs_read(&fs, MAIN_GYRO_BIAS_ID, &retained.gyroBias, sizeof(retained.gyroBias));
		nvs_read(&fs, MAIN_MAG_BIAS_ID, &retained.magBAinv, sizeof(retained.magBAinv));
		nvs_read(&fs, MAIN_ACC_6_BIAS_ID, &retained.accBAinv, sizeof(retained.accBAinv));
		retained_update();
		ram_retention = true;
	}
	else
	{
		LOG_INF("Validated RAM");
	}
	return 0;
}

SYS_INIT(sys_retained_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

// read from retained
uint8_t reboot_counter_read(void)
{
	return retained.reboot_counter;
}

// write to retained
void reboot_counter_write(uint8_t reboot_counter)
{
	retained.reboot_counter = reboot_counter;
	retained_update();
}

// write to retained and nvs
void sys_write(uint16_t id, void *retained_ptr, const void *data, size_t len)
{
	sys_nvs_init();
	memcpy(retained_ptr, data, len);
	nvs_write(&fs, id, data, len);
	retained_update();
}

// return 0 if clock applied, -1 if failed (because there is no clk_en or clk_out)
int set_sensor_clock(bool enable, float rate, float *actual_rate)
{
#if CLK_EN_EXISTS
	gpio_pin_set_dt(&clk_en, enable); // if enabling some external oscillator is available
//	*actual_rate = enable ? (float)NSEC_PER_SEC / clk_out.period : 0; // assume pwm period is the same as an equivalent external oscillator
	*actual_rate = enable ? 32768 : 0; // default
	return 0;
#endif
	*actual_rate = 0; // rate is 0 if there will be no clock source available
	if (!device_is_ready(clk_out.dev))
		return -1;
	int err = pwm_set_dt(&clk_out, PWM_HZ(rate), enable ? PWM_HZ(rate * 2) : 0); // if clk_out is used
	if (!err)
		*actual_rate = enable ? rate : 0; // the system probably could provide the correct rate
	return err;
}

#if BUTTON_EXISTS // Alternate button if available to use as "reset key"
static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static int64_t press_time;

static void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	if (button_read())
	{
		press_time = k_uptime_get();
	}
	else
	{
		if (press_time != 0 && k_uptime_get() - press_time > 50) // Debounce
			sys_reboot(SYS_REBOOT_COLD); // treat like pin reset but without pin reset reason
		press_time = 0;
	}
}

static struct gpio_callback button_cb_data;

static int sys_button_init(void)
{
	gpio_pin_configure_dt(&button0, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&button0, GPIO_INT_EDGE_BOTH);
	gpio_init_callback(&button_cb_data, button_pressed, BIT(button0.pin));
	gpio_add_callback(button0.port, &button_cb_data);
	return 0;
}

SYS_INIT(sys_button_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
#endif

bool button_read(void)
{
#if BUTTON_EXISTS // Alternate button if available to use as "reset key"
	return gpio_pin_get_dt(&button0);
#else
	return false;
#endif
}

#if BUTTON_EXISTS // Alternate button if available to use as "reset key"
static void button_thread(void)
{
	while (1)
	{
		k_msleep(10);
		if (press_time != 0 && k_uptime_get() - press_time > 50 && button_read()) // Button is being pressed
			sys_reboot(SYS_REBOOT_COLD);
	}
}
#endif

static int sys_gpio_init(void)
{
#if DOCK_EXISTS // configure if exists
	gpio_pin_configure_dt(&dock, GPIO_INPUT);
#endif
#if CHG_EXISTS
	gpio_pin_configure_dt(&chg, GPIO_INPUT);
#endif
#if STBY_EXISTS
	gpio_pin_configure_dt(&stby, GPIO_INPUT);
#endif
#if CLK_EN_EXISTS
	gpio_pin_configure_dt(&clk_en, GPIO_OUTPUT);
#endif
#if DCDC_EN_EXISTS
	gpio_pin_configure_dt(&dcdc_en, GPIO_OUTPUT);
#endif
#if LDO_EN_EXISTS
	gpio_pin_configure_dt(&ldo_en, GPIO_OUTPUT);
#endif
	return 0;
}

SYS_INIT(sys_gpio_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

bool dock_read(void)
{
#if DOCK_EXISTS
	return gpio_pin_get_dt(&dock);
#else
	return false;
#endif
}

bool chg_read(void)
{
#if CHG_EXISTS
	return gpio_pin_get_dt(&chg);
#else
	return false;
#endif
}

bool stby_read(void)
{
#if STBY_EXISTS
	return gpio_pin_get_dt(&stby);
#else
	return false;
#endif
}
