/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <stdio.h>
#include <string.h>
#include <nrf_modem_at.h>
#include <nrf_modem_gnss.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/console/console.h>
#include <dk_buttons_and_leds.h>
#include <modem/lte_lc.h>
#include <zephyr/sys/reboot.h>
#include <modem/nrf_modem_lib.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>

#include "ble.h"

LOG_MODULE_REGISTER(lte_ble_gw, CONFIG_LTE_BLE_GW_LOG_LEVEL);

struct device* i2c_accel;
uint8_t WhoAmI = 0u;
#define I2C_ACCEL_ADDR (0x19)
static struct k_work_delayable acc_fifo_h_loop;
bool spi_send_in_progress = false;


static struct device* spi_slave_dev;
struct device* gpio_dev;
static struct gpio_callback acc_int_callback;

int16_t tx_buffer[31] = { 0 };
int16_t rx_buffer[31] = { 0 };

const struct spi_buf tx_buf = {
	.buf = tx_buffer,
	.len = sizeof(tx_buffer),
};

const struct spi_buf_set tx = {
	.buffers = &tx_buf,
	.count = 1,
};

const struct spi_buf rx_buf = {
	.buf = rx_buffer,
	.len = sizeof(rx_buffer),
};

const struct spi_buf_set rx = {
	.buffers = &rx_buf,
	.count = 1,
};

static const struct spi_config spi_cfg = {
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_SLAVE | SPI_MODE_CPOL | SPI_MODE_CPHA,
	.frequency = 8000000,
	.slave = 0,
};

static struct k_poll_signal spi_done_sig = K_POLL_SIGNAL_INITIALIZER(spi_done_sig);

/* Interval in milliseconds between each time status LEDs are updated. */
#define LEDS_UPDATE_INTERVAL K_MSEC(500)

/* Interval in microseconds between each time LEDs are updated when indicating
 * that an error has occurred.
 */
#define LEDS_ERROR_UPDATE_INTERVAL 250000

#define BUTTON_1 BIT(0)
#define BUTTON_2 BIT(1)
#define SWITCH_1 BIT(2)
#define SWITCH_2 BIT(3)

#define LED_ON(x) (x)
#define LED_BLINK(x) ((x) << 8)
#define LED_GET_ON(x) ((x)&0xFF)
#define LED_GET_BLINK(x) (((x) >> 8) & 0xFF)

 /* Interval in milliseconds after the device will retry cloud connection
  * if the event NRF_CLOUD_EVT_TRANSPORT_CONNECTED is not received.
  */
#define RETRY_CONNECT_WAIT K_MSEC(90000)

enum
{
	LEDS_INITIALIZING = LED_ON(0),
	LEDS_LTE_CONNECTING = LED_BLINK(DK_LED3_MSK),
	LEDS_LTE_CONNECTED = LED_ON(DK_LED3_MSK),
	LEDS_CLOUD_CONNECTING = LED_BLINK(DK_LED4_MSK),
	LEDS_CLOUD_PAIRING_WAIT = LED_BLINK(DK_LED3_MSK | DK_LED4_MSK),
	LEDS_CLOUD_CONNECTED = LED_ON(DK_LED4_MSK),
	LEDS_ERROR = LED_ON(DK_ALL_LEDS_MSK)
} display_state;

/* Variable to keep track of nRF cloud user association request. */
static atomic_val_t association_requested;

/* GNSS data */
static struct nrf_modem_gnss_pvt_data_frame pvt_data = {0};

/* Structures for work */
static struct k_work_delayable leds_update_work;
struct k_work_delayable aggregated_work;

static struct nrf_modem_gnss_pvt_data_frame last_pvt;
static bool nrf_modem_gnss_fix;

/* Forward declaration of functions. */
static void sensors_init(void);
static void work_init(void);

static void gnss_event_handler(int event_id)
{
	int err;

	struct nrf_modem_gnss_nmea_data_frame *nmea_data;
	/* Process event */
	switch (event_id)
	{
	case NRF_MODEM_GNSS_EVT_PVT:
		/* Read PVT data */
		err = nrf_modem_gnss_read(&pvt_data, sizeof(pvt_data),
								  NRF_MODEM_GNSS_DATA_PVT);
		break;
	case NRF_MODEM_GNSS_EVT_BLOCKED:
		printk("GNSS Service blocked!\n");
		break;
	case NRF_MODEM_GNSS_EVT_UNBLOCKED:
		printk("GNSS Service unblocked!\n");
		break;
	default:
		break;
	}
}

static void gnss_init(void)
{
	int err;

	err = nrf_modem_gnss_event_handler_set(gnss_event_handler);
	if (err)
	{
		printk("Cannot set gnss event handler, error: %d\n", err);
		return;
	}

	uint16_t nmea_mask =
		NRF_MODEM_GNSS_NMEA_RMC_MASK | NRF_MODEM_GNSS_NMEA_GGA_MASK |
		NRF_MODEM_GNSS_NMEA_GLL_MASK | NRF_MODEM_GNSS_NMEA_GSA_MASK |
		NRF_MODEM_GNSS_NMEA_GSV_MASK;

	err = nrf_modem_gnss_nmea_mask_set(nmea_mask);
	if (err)
	{
		printk("Failed to set GNSS NMEA mask\n");
		return;
	}

	/* This use case flag should always be set. */
	uint8_t use_case = NRF_MODEM_GNSS_USE_CASE_MULTIPLE_HOT_START;
	use_case |= NRF_MODEM_GNSS_USE_CASE_SCHED_DOWNLOAD_DISABLE;

	err = nrf_modem_gnss_use_case_set(use_case);
	if (err)
	{
		printk("Failed to set GNSS use case\n");
		return;
	}

	uint8_t power_mode = NRF_MODEM_GNSS_PSM_DISABLED;

	err = nrf_modem_gnss_power_mode_set(power_mode);
	if (err)
	{
		printk("Failed to set GNSS power saving mode\n");
		return;
	}

	err = nrf_modem_gnss_fix_interval_set(1);
	if (err)
	{
		printk("Cannot set gnss fix interval, error: %d\n", err);
		return;
	}

	err = nrf_modem_gnss_fix_retry_set(0);
	if (err)
	{
		printk("Cannot set gnss fix retry, error: %d\n", err);
		return;
	}

	err = nrf_modem_gnss_start();
	if (err)
	{
		printk("Cannot initialize gnss, error: %d\n", err);
		return;
	}
}

static void modem_init(void)
{
	int err;

	if (IS_ENABLED(CONFIG_LTE_AUTO_INIT_AND_CONNECT))
	{
		/* Do nothing, modem is already configured and LTE connected. */
	}
	else
	{
		err = lte_lc_init();
		if (err)
		{
			printk("Modem initialization failed, error: %d\n", err);
			return;
		}

		enum lte_lc_func_mode func_mode = LTE_LC_FUNC_MODE_ACTIVATE_GNSS;
		err = lte_lc_func_mode_set(func_mode);
		if (err)
		{
			printk("Cannot activate GNSS, error: %d\n", err);
			return;
		}
	}
}

static void acc_int_handler(const struct device* port, struct gpio_callback* cb, gpio_port_pins_t pins)
{
	k_work_schedule(&acc_fifo_h_loop, K_NO_WAIT);
}

void spi_slave_callback(const struct device* dev, int result, void* data)
{
	// printk("SPI transmitted: %d\r\n", result);
	gpio_pin_set(gpio_dev, 9, 0);
	spi_send_in_progress = false;
}

void acc_fifo_h_loop_fn()
{
	// LOG_INF("ACC INT Triggered");
	static bool led_state = false;

	// static int int_counter = 0;

	uint8_t interrupt;
	i2c_reg_read_byte(i2c_accel, I2C_ACCEL_ADDR, 0x31, &interrupt);
	// LOG_INF("INT: %02X", interrupt);

	int16_t x_data, y_data, z_data;
	for (int i = 0; i < 10; i++)
	{
		int8_t x_data_l, x_data_h, y_data_l, y_data_h, z_data_l, z_data_h;
		i2c_reg_read_byte(i2c_accel, I2C_ACCEL_ADDR, 0x28, &x_data_l);
		i2c_reg_read_byte(i2c_accel, I2C_ACCEL_ADDR, 0x29, &x_data_h);

		i2c_reg_read_byte(i2c_accel, I2C_ACCEL_ADDR, 0x2A, &y_data_l);
		i2c_reg_read_byte(i2c_accel, I2C_ACCEL_ADDR, 0x2B, &y_data_h);

		i2c_reg_read_byte(i2c_accel, I2C_ACCEL_ADDR, 0x2C, &z_data_l);
		i2c_reg_read_byte(i2c_accel, I2C_ACCEL_ADDR, 0x2D, &z_data_h);

		x_data = (((x_data_h << 8) | x_data_l) >> 6);
		y_data = (((y_data_h << 8) | y_data_l) >> 6);
		z_data = (((z_data_h << 8) | z_data_l) >> 6);

		tx_buffer[(i * 3)] = x_data;
		tx_buffer[(i * 3) + 1] = y_data;
		tx_buffer[(i * 3) + 2] = z_data;
	}

	tx_buffer[30] = pvt_data.speed * 3.6;

	// for (int i = 0; i < 31; i++)
	// {
	// 	printk("%5d ", tx_buffer[i]);
	// }

	// printk("\r\n");

	if (!spi_send_in_progress)
	{
		// printk("Sending...\r\n");
		int err = spi_transceive_cb(spi_slave_dev, &spi_cfg, &tx, &rx, spi_slave_callback, NULL);
		if (err != 0)
		{
			LOG_ERR("SPI error: %d", err);
		}

		spi_send_in_progress = true;
		gpio_pin_set(gpio_dev, 9, 1);
		if (!led_state)
		{
			dk_set_leds_state(0x01, DK_LED1_MSK);
		}
		else
		{
			dk_set_leds_state(0x00, DK_LED1_MSK);
		}
		led_state = !led_state;
	}
}

/**@brief Update LEDs state. */
static void leds_update(struct k_work* work)
{
	static bool led_on;
	static uint8_t current_led_on_mask;
	uint8_t led_on_mask = current_led_on_mask;

	ARG_UNUSED(work);

	/* Reset LED3 and LED4. */
	led_on_mask &= ~(DK_LED3_MSK | DK_LED4_MSK);

	/* Set LED3 and LED4 to match current state. */
	led_on_mask |= LED_GET_ON(display_state);

	led_on = !led_on;
	if (led_on)
	{
		led_on_mask |= LED_GET_BLINK(display_state);
	}
	else
	{
		led_on_mask &= ~LED_GET_BLINK(display_state);
	}

	if (led_on_mask != current_led_on_mask)
	{
		dk_set_leds(led_on_mask);
		current_led_on_mask = led_on_mask;
	}

	k_work_schedule(&leds_update_work, LEDS_UPDATE_INTERVAL);
}

/**@brief Callback for button events from the DK buttons and LEDs library. */
static void button_handler(uint32_t buttons, uint32_t has_changed)
{
	LOG_INF("button_handler: button 1: %u, button 2: %u "
		"switch 1: %u, switch 2: %u",
		(bool)(buttons & BUTTON_1), (bool)(buttons & BUTTON_2), (bool)(buttons & SWITCH_1),
		(bool)(buttons & SWITCH_2));
}

/**@brief Initializes and submits delayed work. */
static void work_init(void)
{
	k_work_init_delayable(&leds_update_work, leds_update);
	k_work_schedule(&leds_update_work, LEDS_UPDATE_INTERVAL);
}

/**@brief Initializes buttons and LEDs, using the DK buttons and LEDs
 * library.
 */
static void buttons_leds_init(void)
{
	int err;

	err = dk_buttons_init(button_handler);
	if (err)
	{
		LOG_ERR("Could not initialize buttons, err code: %d", err);
	}

	err = dk_leds_init();
	if (err)
	{
		LOG_ERR("Could not initialize leds, err code: %d", err);
	}

	err = dk_set_leds_state(0x00, DK_ALL_LEDS_MSK);
	if (err)
	{
		LOG_ERR("Could not set leds state, err code: %d", err);
	}
}

uint8_t init_accelerometer()
{
	i2c_accel = DEVICE_DT_GET(DT_N_NODELABEL_i2c1);
	if (!i2c_accel)
	{
		printk("error\n");
		return -1;
	}
	else
	{
		i2c_configure(i2c_accel, I2C_SPEED_SET(I2C_SPEED_STANDARD));
		return 0;
	}
}

void main(void)
{
	LOG_INF("LTE Sensor Gateway sample started");

	buttons_leds_init();

	modem_init();

	gnss_init();

	gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));

	int err;
	if (!device_is_ready(gpio_dev))
	{
		LOG_ERR("GPIO is not ready");
	}

	err = gpio_pin_configure(gpio_dev, 10, GPIO_INPUT | GPIO_ACTIVE_LOW);
	if (err)
	{
		LOG_ERR("GPIO pin10 IN counfiguration failed");
	}

	err = gpio_pin_configure(gpio_dev, 9, GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_HIGH);
	if (err)
	{
		LOG_ERR("GPIO pin9 OUT counfiguration failed");
	}

	k_work_init_delayable(&acc_fifo_h_loop, acc_fifo_h_loop_fn);

	gpio_init_callback(&acc_int_callback, acc_int_handler, BIT(10));
	gpio_add_callback(gpio_dev, &acc_int_callback);
	gpio_pin_interrupt_configure(gpio_dev, 10, GPIO_INT_EDGE_RISING);

	spi_slave_dev = DEVICE_DT_GET(DT_NODELABEL(spi3));

	if (!device_is_ready(spi_slave_dev))
	{
		LOG_ERR("SPI3 is not ready");
	}

	printk("GNSS sample has started\n");

	ble_init();

	work_init();

	if (0 != init_accelerometer())
	{
		printk("Cannot initialize i2c\r\n");
		return;
	}

	if (i2c_reg_read_byte(i2c_accel, I2C_ACCEL_ADDR, 0x0F, &WhoAmI) != 0)
	{
		printk("Error on i2c_read()\n");
		return;
	}

	if (i2c_reg_write_byte(i2c_accel, I2C_ACCEL_ADDR, 0x20, 0x57) != 0) // All axis, 100 Hz, Normal mode
	{
		printk("Error on i2c write to CTRL1\n");
		return;
	}

	if (i2c_reg_write_byte(i2c_accel, I2C_ACCEL_ADDR, 0x21, 0x18) != 0) // 1Hz HP Cutoff
	{
		printk("Error on i2c write to CTRL2\n");
		return;
	}

	if (i2c_reg_write_byte(i2c_accel, I2C_ACCEL_ADDR, 0x22, 0x04) != 0) // WTM
	{
		printk("Error on i2c write to CTRL3\n");
		return;
	}

	if (i2c_reg_write_byte(i2c_accel, I2C_ACCEL_ADDR, 0x23, 0x08) != 0)
	{
		printk("Error on i2c write to CTRL4\n");
		return;
	}

	if (i2c_reg_write_byte(i2c_accel, I2C_ACCEL_ADDR, 0x24, 0x48) != 0) // FIFO, INT1 latched
	{
		printk("Error on i2c write to CTRL5\n");
		return;
	}

	if (i2c_reg_write_byte(i2c_accel, I2C_ACCEL_ADDR, 0x30, 0x2A) != 0)
	{
		printk("Error on i2c write to INT1_CFG\n");
		return;
	}

	if (i2c_reg_write_byte(i2c_accel, I2C_ACCEL_ADDR, 0x2E, 0x00) != 0) // Clear FIFO
	{
		printk("Error on i2c write to CTRL5\n");
		return;
	}

	if (i2c_reg_write_byte(i2c_accel, I2C_ACCEL_ADDR, 0x2E, 0x8A) != 0) // Stream Mode, 10 samples WTM
	{
		printk("Error on i2c write to CTRL5\n");
		return;
	}

	printk("WhoAmI = 0x%02X\r\n", WhoAmI);
}
