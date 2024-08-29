#include <zephyr/logging/log.h>
#include <zephyr/types.h>
#include <zephyr/drivers/i2c.h>

#define SCAN_ADDR_START 8
#define SCAN_ADDR_STOP 119

LOG_MODULE_REGISTER(sensor_scan, LOG_LEVEL_INF);

// TODO: any address out of range (00, 7f, etc. will search all addresses and stored address first if available, specify an address here to search first otherwise
// On reset (or user power on) or first power on, the scan function will be forced to run and do the above
// Otherwise it should use an address and imus already stored in volatile memory
// Cannot gurantee storage in flash, it's probably okay to keep it volatile

int sensor_scan(struct i2c_dt_spec *i2c_dev, int dev_addr_count, const uint8_t dev_addr[], const uint8_t dev_reg[], const uint8_t dev_id[], const int dev_ids[])
{
	const struct device *dev = i2c_dev->bus;

	uint16_t addr = 0;

	int addr_index = 0;
	int reg_index = 0;
	int id_index = 0;
	int found_id = 0;

	for (int i = 0; i < dev_addr_count; i++)
	{
		int addr_count = dev_addr[addr_index];
		int reg_count = dev_reg[reg_index];
		int id_count = dev_id[id_index];
		addr_index++;
		reg_index++;
		id_index++;
		for (int j = 0; j < addr_count; j++)
		{
			if (i2c_dev->addr >= SCAN_ADDR_START && i2c_dev->addr <= SCAN_ADDR_STOP && i2c_dev->addr != addr)
				continue; // if an address was provided try to scan it first
			addr = dev_addr[addr_index + j];
			LOG_DBG("Scanning address: %02X", addr);
			int id_cnt = id_count;
			int id_ind = id_index;
			int fnd_id = found_id;
			for (int k = 0; k < reg_count; k++)
			{
				uint8_t reg = dev_reg[reg_index + k];
				uint8_t id;
				LOG_DBG("Scanning register: %02X", reg);
				int err = i2c_reg_read_byte(dev, addr, reg, &id);
				LOG_DBG("Read value: %02X", id);
				if (err)
					break;
				for (int l = 0; l < id_cnt; l++)
				{
					if (id == dev_id[id_ind + l])
					{
						i2c_dev->addr = addr;
						return dev_ids[fnd_id + l];
					}
				}
				id_ind += id_cnt;
				fnd_id += id_cnt;
				id_cnt = dev_id[id_ind];
				id_ind++;
			}
		}
		addr_index += addr_count;
		reg_index += reg_count;
		id_index += id_count;
		found_id += id_count;
		for (int j = 1; j < reg_count; j++)
		{
			id_count = dev_id[id_index];
			id_index++;
			id_index += id_count;
			found_id += id_count;
		}
	}

	if (i2c_dev->addr >= SCAN_ADDR_START && i2c_dev->addr <= SCAN_ADDR_STOP) // preferred address failed, try again with full scan
	{
		i2c_dev->addr = 0;
		return sensor_scan(i2c_dev, dev_addr_count, dev_addr, dev_reg, dev_id, dev_ids);
	}

	return -1;
}
