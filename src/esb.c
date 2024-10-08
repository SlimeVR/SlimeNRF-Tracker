#include "globals.h"
#include "system.h"
#include "connection.h"

#include <zephyr/drivers/clock_control/nrf_clock_control.h>

#include "esb.h"

uint8_t last_reset = 0;
const nrfx_timer_t m_timer = NRFX_TIMER_INSTANCE(1);
bool esb_state = false;
bool timer_state = false;
bool send_data = false;
uint16_t led_clock = 0;
uint32_t led_clock_offset = 0;

uint32_t tx_errors = 0;

static struct esb_payload rx_payload;
static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0,
														  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
static struct esb_payload tx_payload_pair = ESB_CREATE_PAYLOAD(0,
														  0, 0, 0, 0, 0, 0, 0, 0);

static uint8_t paired_addr[8] = {0};

LOG_MODULE_REGISTER(esb_event, LOG_LEVEL_INF);

void event_handler(struct esb_evt const *event)
{
	switch (event->evt_id)
	{
	case ESB_EVENT_TX_SUCCESS:
		tx_errors = 0;
		set_status(SYS_STATUS_CONNECTION_ERROR, false);
		break;
	case ESB_EVENT_TX_FAILED:
		if (++tx_errors > 100) // consecutive failure to transmit
			set_status(SYS_STATUS_CONNECTION_ERROR, true);
//		LOG_INF("TX FAILED");
		LOG_DBG("TX FAILED");
		break;
	case ESB_EVENT_RX_RECEIVED:
		if (esb_read_rx_payload(&rx_payload))
		{
			if (!paired_addr[0]) // zero, not paired
			{
				if (rx_payload.length == 8)
					memcpy(paired_addr, rx_payload.data, sizeof(paired_addr));
			}
			else
			{
				if (rx_payload.length == 4)
				{
					// TODO: Device should never receive packets if it is already paired, why is this packet received?
					// This may be part of acknowledge
					if (!nrfx_timer_init_check(&m_timer))
					{
						LOG_WRN("Timer not initialized");
						break;
					}
					if (timer_state == false)
					{
						nrfx_timer_resume(&m_timer);
						timer_state = true;
					}
					nrfx_timer_clear(&m_timer);
					last_reset = 0;
					led_clock = (rx_payload.data[0] << 8) + rx_payload.data[1]; // sync led flashes :)
					led_clock_offset = 0;
					LOG_DBG("RX, timer reset");
				}
			}
		}
		break;
	}
}

int clocks_start(void)
{
	int err;
	int res;
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr)
	{
		LOG_ERR("Unable to get the Clock manager");
		return -ENXIO;
	}

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0)
	{
		LOG_ERR("Clock request failed: %d", err);
		return err;
	}

	do
	{
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res)
		{
			LOG_ERR("Clock could not be started: %d", res);
			return res;
		}
	} while (err);

	LOG_DBG("HF clock started");
	return 0;
}

// this was randomly generated
// TODO: I have no idea?
static const uint8_t discovery_base_addr_0[4] = {0x62, 0x39, 0x8A, 0xF2};
static const uint8_t discovery_base_addr_1[4] = {0x28, 0xFF, 0x50, 0xB8};
static const uint8_t discovery_addr_prefix[8] = {0xFE, 0xFF, 0x29, 0x27, 0x09, 0x02, 0xB2, 0xD6};

static uint8_t base_addr_0[4], base_addr_1[4], addr_prefix[8] = {0};

static bool esb_initialized = false;

int esb_initialize(bool tx)
{
	int err;

	struct esb_config config = ESB_DEFAULT_CONFIG;

	if (tx)
	{
		// config.protocol = ESB_PROTOCOL_ESB_DPL;
		// config.mode = ESB_MODE_PTX;
		config.event_handler = event_handler;
		// config.bitrate = ESB_BITRATE_2MBPS;
		// config.crc = ESB_CRC_16BIT;
		config.tx_output_power = 4;
		// config.retransmit_delay = 600;
		//config.retransmit_count = 0;
		//config.tx_mode = ESB_TXMODE_MANUAL;
		// config.payload_length = 32;
		config.selective_auto_ack = true;
	}
	else
	{
		// config.protocol = ESB_PROTOCOL_ESB_DPL;
		config.mode = ESB_MODE_PRX;
		config.event_handler = event_handler;
		// config.bitrate = ESB_BITRATE_2MBPS;
		// config.crc = ESB_CRC_16BIT;
		config.tx_output_power = 4;
		// config.retransmit_delay = 600;
		// config.retransmit_count = 3;
		// config.tx_mode = ESB_TXMODE_AUTO;
		// config.payload_length = 32;
		config.selective_auto_ack = true;
	}

	// Fast startup mode
	NRF_RADIO->MODECNF0 |= RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos;
	// nrf_radio_modecnf0_set(NRF_RADIO, true, 0);

	err = esb_init(&config);

	if (!err)
		esb_set_base_address_0(base_addr_0);

	if (!err)
		esb_set_base_address_1(base_addr_1);

	if (!err)
		esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));

	if (err)
	{
		LOG_ERR("ESB initialization failed: %d", err);
		set_status(SYS_STATUS_CONNECTION_ERROR, true);
		return err;
	}

	esb_initialized = true;
	return 0;
}

inline void esb_set_addr_discovery(void)
{
	memcpy(base_addr_0, discovery_base_addr_0, sizeof(base_addr_0));
	memcpy(base_addr_1, discovery_base_addr_1, sizeof(base_addr_1));
	memcpy(addr_prefix, discovery_addr_prefix, sizeof(addr_prefix));
}

inline void esb_set_addr_paired(void)
{
	// Recreate receiver address
	uint8_t addr_buffer[16] = {0};
	for (int i = 0; i < 4; i++)
	{
		addr_buffer[i] = paired_addr[i + 2];
		addr_buffer[i + 4] = paired_addr[i + 2] + paired_addr[6];
	}
	for (int i = 0; i < 8; i++)
		addr_buffer[i + 8] = paired_addr[7] + i;
	for (int i = 0; i < 16; i++)
	{
		if (addr_buffer[i] == 0x00 || addr_buffer[i] == 0x55 || addr_buffer[i] == 0xAA) // Avoid invalid addresses (see nrf datasheet)
			addr_buffer[i] += 8;
	}
	memcpy(base_addr_0, addr_buffer, sizeof(base_addr_0));
	memcpy(base_addr_1, addr_buffer + 4, sizeof(base_addr_1));
	memcpy(addr_prefix, addr_buffer + 8, sizeof(addr_prefix));
}

static bool esb_paired = false;

void esb_pair(void)
{
	// Read paired address from retained
	// TODO: should pairing data stay within esb?
	memcpy(paired_addr, retained.paired_addr, sizeof(paired_addr));

	if (!paired_addr[0]) // zero, no receiver paired
	{
		LOG_INF("Pairing");
		esb_set_addr_discovery();
		esb_initialize();
//		timer_init(); // TODO: shouldn't be here!!!
		tx_payload_pair.noack = false;
		uint64_t addr = (((uint64_t)(NRF_FICR->DEVICEADDR[1]) << 32) | NRF_FICR->DEVICEADDR[0]) & 0xFFFFFF; // Use device address as unique identifier (although it is not actually guaranteed, see datasheet)
		uint8_t check = addr & 0xFF;
		if (check == 0)
			check = 8;
		LOG_INF("Check code: %02X", check);
		tx_payload_pair.data[0] = check; // Use int from device address to make sure packet is for this device
		for (int i = 0; i < 6; i++)
			tx_payload_pair.data[i + 2] = (addr >> (8 * i)) & 0xFF;
		set_led(SYS_LED_PATTERN_SHORT, SYS_LED_PRIORITY_CONNECTION);
		while (paired_addr[0] != check)
		{
			if (paired_addr[0])
			{
				LOG_INF("Incorrect check code: %02X", paired_addr[0]);
				paired_addr[0] = 0x00; // Packet not for this device
			}
			esb_flush_rx();
			esb_flush_tx();
			esb_write_payload(&tx_payload_pair); // TODO: Does this still fail after a while?
			esb_start_tx();
			k_msleep(1000);
		}
		set_led(SYS_LED_PATTERN_ONESHOT_PAIRED, SYS_LED_PRIORITY_CONNECTION);
		LOG_INF("Paired");
		sys_write(PAIRED_ID, retained.paired_addr, paired_addr, sizeof(paired_addr)); // Write new address and tracker id
		esb_disable();
		k_msleep(1500); // wait for led pattern
	}
	LOG_INF("Read pairing data");
	LOG_INF("Check code: %02X", paired_addr[0]);
	LOG_INF("Tracker ID: %u", paired_addr[1]);
	LOG_INF("Address: %02X %02X %02X %02X %02X %02X", paired_addr[2], paired_addr[3], paired_addr[4], paired_addr[5], paired_addr[6], paired_addr[7]);

	connection_set_id(paired_addr[1]);

	esb_set_addr_paired();
	esb_paired = true;
}

void esb_reset_pair(void)
{
	uint8_t empty_addr[8] = {0};
	sys_write(PAIRED_ID, &retained.paired_addr, empty_addr, sizeof(paired_addr)); // write zeroes
	LOG_INF("Pairing data reset");
}

void esb_write(uint8_t *data)
{
	if (!esb_initialized || !esb_paired)
		return;
	tx_payload.noack = false;
	memcpy(tx_payload.data, data, sizeof(tx_payload.data));
	esb_flush_tx(); // this will clear all transmissions even if they did not complete
	esb_write_payload(&tx_payload); // Add transmission to queue
	send_data = true;
}

bool esb_ready(void)
{
	return esb_initialized && esb_paired;
}
