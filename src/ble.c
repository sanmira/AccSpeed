/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>

#include <bluetooth/gatt_dm.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#include <dk_buttons_and_leds.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/logging/log.h>
#include <nrfx_gpiote.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>

#include "ble.h"

LOG_MODULE_DECLARE(lte_ble_gw);

static K_SEM_DEFINE(bt_init_ok, 0, 1);

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

void on_connected(struct bt_conn* conn, uint8_t err);
void on_disconnected(struct bt_conn* conn, uint8_t reason);
void on_notif_changed(enum bt_button_notifications_enabled status);
int send_bt_notification(struct bt_conn *conn, uint8_t value, uint16_t length);
static struct k_work_delayable notification_loop;

static struct bt_conn* current_conn;
enum bt_button_notifications_enabled notifications_enabled = BT_BUTTON_NOTIFICATIONS_DISABLED;

static struct bt_conn_cb bluetooth_callbacks = {
    .connected = on_connected,
    .disconnected = on_disconnected,
};

static struct bt_remote_service_cb remote_serv_callbacks = {
  .notif_changed = on_notif_changed,
};

void on_connected(struct bt_conn* conn, uint8_t err)
{
  if (err)
  {
    LOG_ERR("Connection err: %d", err);
    return;
  }

  LOG_INF("Connected");
  current_conn = bt_conn_ref(conn);

  uint16_t current_mtu = bt_gatt_get_mtu(current_conn);
  LOG_INF("Current MTU: %d", current_mtu);
}

void on_disconnected(struct bt_conn* conn, uint8_t reason)
{
  LOG_INF("Disconnected (reason: %d)", reason);
  if (current_conn)
  {
    bt_conn_unref(current_conn);
    current_conn = NULL;
  }
}

void on_notif_changed(enum bt_button_notifications_enabled status)
{
  if (status == BT_BUTTON_NOTIFICATIONS_ENABLED) {
    LOG_INF("Notifications enabled");
  } else {
    LOG_INF("Notifications disabled");
  }
}

void notification_loop_fn()
{
  static uint8_t counter = 0;

  if (notifications_enabled == BT_BUTTON_NOTIFICATIONS_ENABLED)
  {
    send_bt_notification(current_conn, counter, 1);
    counter++;
  }
  k_work_schedule(&notification_loop, K_SECONDS(1));
}

static uint8_t button_value = 0;

void button_chrc_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);

static ssize_t read_button_characteristic_cb(struct bt_conn* conn, const struct bt_gatt_attr* attr,
  void* buf, uint16_t len, uint16_t offset);

BT_GATT_SERVICE_DEFINE(remote_srv,
  BT_GATT_PRIMARY_SERVICE(BT_UUID_REMOTE_SERVICE),
  BT_GATT_CHARACTERISTIC(BT_UUID_REMOTE_BUTTON_CHRC,
    BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
    BT_GATT_PERM_READ,
    read_button_characteristic_cb, NULL, NULL),
    BT_GATT_CCC(button_chrc_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
  );

void button_chrc_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
  bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);
  // LOG_INF("Notifications %s", notif_enabled ? "enabled" : "disabled");

  notifications_enabled = notif_enabled ? BT_BUTTON_NOTIFICATIONS_ENABLED : BT_BUTTON_NOTIFICATIONS_DISABLED;

  if (remote_serv_callbacks.notif_changed) {
    remote_serv_callbacks.notif_changed(notifications_enabled);
  }
}

static ssize_t read_button_characteristic_cb(struct bt_conn* conn, const struct bt_gatt_attr* attr,
  void* buf, uint16_t len, uint16_t offset)
{
  ssize_t ret_size = bt_gatt_attr_read(conn, attr, buf, len, offset, &button_value, sizeof(button_value));
  button_value++;
  return ret_size;
}

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN) };

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_REMOTE_SERV_VAL),
};

uint32_t counter = 0;

void on_sent(struct bt_conn *conn, void *user_data)
{
  ARG_UNUSED(user_data);
  LOG_INF("%d, Notification sent on connection  %p", counter, (void *) conn);
  counter++;
}

int send_bt_notification(struct bt_conn *conn, uint8_t value, uint16_t length) 
{
  int err = 0;

  struct bt_gatt_notify_params params = {0};
  const struct bt_gatt_attr *attr = &remote_srv.attrs[2];
  params.attr = attr;
  params.data = "01234567890123456789";
  params.len = strlen(params.data);
  params.func = on_sent;

  // dk_set_leds_state(0x01, DK_LED1_MSK);
  // err = bt_gatt_notify_cb(conn, &params);
  // dk_set_leds_state(0x00, DK_LED1_MSK);

  dk_set_leds_state(((counter % 2) == 0) ? 0x1 : 0x0, DK_LED1_MSK);
  bt_gatt_notify(conn, attr, (void*)&counter, 4);
  counter++;

  return err;
}

static void ble_ready(int err)
{
  LOG_INF("Bluetooth ready");
  k_sem_give(&bt_init_ok);
}

int ble_init(void)
{
  int err;

  LOG_INF("Initializing Bluetooth..");

  bt_conn_cb_register(&bluetooth_callbacks);

  err = bt_enable(ble_ready);
  if (err)
  {
    LOG_ERR("Bluetooth init failed (err %d)", err);
    return err;
  }

  k_sem_take(&bt_init_ok, K_FOREVER);

  err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
  if (err)
  {
    LOG_ERR("Couldn't start advertising (err = %d)", err);
    return err;
  }

  k_work_init_delayable(&notification_loop, notification_loop_fn);

  err = dk_leds_init();
	if (err)
	{
		LOG_ERR("Could not initialize leds, err code: %d", err);
	}

  err = dk_set_leds_state(0x00, DK_LED1_MSK);
  
	if (err)
	{
		LOG_ERR("Could not set leds state, err code: %d", err);
	}

  k_work_schedule(&notification_loop, K_NO_WAIT);

  return err;
}
