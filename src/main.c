/* This file is based on the periodic_sync sample: 
 * https://github.com/nrfconnect/sdk-zephyr/blob/main/samples/bluetooth/periodic_sync/src/main.c
 * which is subject to the following License:
 * 
 * Copyright (c) 2020-2024 Nordic Semiconductor ASA
 * 
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdint.h>

#include <zephyr/kernel.h> //kerenl services

#include <zephyr/bluetooth/gap.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/sys/util.h>

#define TIMEOUT_SYNC_CREATE K_SECONDS(10)
#define NAME_LEN            30

///Change this to your Auracast Broadcast name
#define BROADCAST_NAME "Tomer"
//Remeber to use COM8 to see logs.
#define RAW_PERIODIC_DATA_MAX_LEN 80

static uint8_t raw_periodic_data[RAW_PERIODIC_DATA_MAX_LEN] = {0}; //TODO: maybe get rid if not needed.

static bool         got_per_adv_data = false;


static bool         per_adv_found;
static bt_addr_le_t per_addr;
static uint16_t per_adv_sync_timeout;
static uint8_t      per_sid;

static K_SEM_DEFINE(sem_per_adv, 0, 1);
static K_SEM_DEFINE(sem_per_sync, 0, 1);
static K_SEM_DEFINE(sem_per_sync_lost, 0, 1);

//static K_SEM_DEFINE(sem_auracast_found, 0, 1); //found our auracast broadcast

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#ifdef CONFIG_PER_BLINK_LED0
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
#define BLINK_ONOFF K_MSEC(500)

static struct k_work_delayable blink_work;
static bool                  led_is_on;

static void blink_timeout(struct k_work *work)
{
	led_is_on = !led_is_on;
	gpio_pin_set(led.port, led.pin, (int)led_is_on);

	k_work_schedule(&blink_work, BLINK_ONOFF);
}
#endif

//Results for high-quality broadcast are:
// 23:08:02.347: discovered EXTENDED: BdAddr([74, 178, 179, 118, 40, 28]) and the direct addr is BdAddr([0, 0, 0, 0, 0, 0])
// 23:08:02.347: The data is [6, 22, 82, 24, 58, 169, 91, 5, 22, 86, 24, 4, 0, 6, 48, 84, 111, 109, 101, 114]
// 23:08:20.356: discovered EXTENDED: BdAddr([227, 22, 199, 247, 209, 3]) and the direct addr is BdAddr([0, 0, 0, 0, 0, 0])
// 23:08:20.356: The data is [6, 22, 82, 24, 108, 141, 197, 5, 22, 86, 24, 4, 0, 6, 48, 84, 111, 109, 101, 114]
// 17:49:01.722: discovered EXTENDED: BdAddr([30, 66, 139, 63, 217, 52]) and the direct addr is BdAddr([0, 0, 0, 0, 0, 0])
// 17:49:01.722: The data is [6, 22, 82, 24, 7, 111, 34, 5, 22, 86, 24, 4, 0, 6, 48, 84, 111, 109, 101, 114]
//                            ^               ^   ^    ^ I think these 3 bytes that vary might be the broadcast id.
//                           Length, AD Type (22; 0x16 in hex)--Service Data - 16-bit UUID,
// 


/// @brief This is called on each Length Type Value triplet In a particular scan result.
/// When it returns true the parsing continues. When it returns false the parsing stops.
///

/// 48 in hex is 0x30 which is Broadcast_Name in the Bluetooth assigned numbers.
/// This is the only way for us to identify the Auracast broadcast 
/// as the (broadcast source) device address is randomized.
/// @param data 
/// @param user_data 
/// @return 
static bool data_cb(struct bt_data *data, void *user_data)
{
	char *name = user_data;
	uint8_t len;

	switch (data->type) {
	case BT_DATA_BROADCAST_NAME:
		len = MIN(data->data_len, NAME_LEN - 1);
		memcpy(name, data->data, len);
		name[len] = '\0';
		//printk("The name is %s \n", name);
		return false;
	default:
		return true;
	}
}

static const char *phy2str(uint8_t phy)
{
	switch (phy) {
	case 0: return "No packets";
	case BT_GAP_LE_PHY_1M: return "LE 1M";
	case BT_GAP_LE_PHY_2M: return "LE 2M";
	case BT_GAP_LE_PHY_CODED: return "LE Coded";
	default: return "Unknown";
	}
}

static void scan_recv(const struct bt_le_scan_recv_info *info,
		      struct net_buf_simple *buf)
{
	char le_addr[BT_ADDR_LE_STR_LEN];
	char name[NAME_LEN];

	(void)memset(name, 0, sizeof(name));

	//This function modifies name which is why we need to reset it above.
	bt_data_parse(buf, data_cb, name);
	
	bool found_auracast= !strcmp(name, BROADCAST_NAME);
	
	if (found_auracast) {

		if(!per_adv_found) //To avoid seeing these over and over
		{
			printk("Found AURACAST!\n");
			
			bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));
			printk("[DEVICE]: %s, AD evt type %u, Tx Pwr: %i, RSSI %i Broadcast name: %s "
				"C:%u S:%u D:%u SR:%u E:%u Prim: %s, Secn: %s, "
				"Interval: 0x%04x (%u ms), SID: %u\n",
				le_addr, info->adv_type, info->tx_power, info->rssi, name,
				(info->adv_props & BT_GAP_ADV_PROP_CONNECTABLE) != 0,
				(info->adv_props & BT_GAP_ADV_PROP_SCANNABLE) != 0,
				(info->adv_props & BT_GAP_ADV_PROP_DIRECTED) != 0,
				(info->adv_props & BT_GAP_ADV_PROP_SCAN_RESPONSE) != 0,
				(info->adv_props & BT_GAP_ADV_PROP_EXT_ADV) != 0,
				phy2str(info->primary_phy), phy2str(info->secondary_phy),
				info->interval, info->interval * 5 / 4, info->sid);
		}

		if (!per_adv_found && info->interval) {
			uint32_t interval_us;
			uint32_t timeout;

			per_adv_found = true;

			/* Add retries and convert to unit in 10's of ms */
			interval_us = BT_GAP_PER_ADV_INTERVAL_TO_US(info->interval);

			timeout = BT_GAP_US_TO_PER_ADV_SYNC_TIMEOUT(interval_us);

			/* 10 attempts */
			timeout *= 10;

			/* Enforce restraints */
			per_adv_sync_timeout =
				CLAMP(timeout, BT_GAP_PER_ADV_MIN_TIMEOUT, BT_GAP_PER_ADV_MAX_TIMEOUT);

			per_sid = info->sid;
			bt_addr_le_copy(&per_addr, info->addr);

			k_sem_give(&sem_per_adv);
		}
	}
}

static struct bt_le_scan_cb scan_callbacks = {
	.recv = scan_recv,
};

static void sync_cb(struct bt_le_per_adv_sync *sync,
		    struct bt_le_per_adv_sync_synced_info *info)
{
	char le_addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

	printk("PER_ADV_SYNC[%u]: [DEVICE]: %s synced, "
	       "Interval 0x%04x (%u ms), PHY %s\n",
	       bt_le_per_adv_sync_get_index(sync), le_addr,
	       info->interval, info->interval * 5 / 4, phy2str(info->phy));

	k_sem_give(&sem_per_sync);
}

static void term_cb(struct bt_le_per_adv_sync *sync,
		    const struct bt_le_per_adv_sync_term_info *info)
{
	char le_addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

	printk("PER_ADV_SYNC[%u]: [DEVICE]: %s sync terminated\n",
	       bt_le_per_adv_sync_get_index(sync), le_addr);

	k_sem_give(&sem_per_sync_lost);
}

static void recv_cb(struct bt_le_per_adv_sync *sync,
		    const struct bt_le_per_adv_sync_recv_info *info,
		    struct net_buf_simple *buf)
{
	if (!got_per_adv_data) {
		
	
		char le_addr[BT_ADDR_LE_STR_LEN];
		

		bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));
		//bin2hex(buf->data, buf->len, data_str, sizeof(data_str));

		printk("PER_ADV_SYNC[%u]: [DEVICE]: %s, tx_power %i, "
			"RSSI %i, CTE %u, data length %u, data: [",
			bt_le_per_adv_sync_get_index(sync), le_addr, info->tx_power,
			info->rssi, info->cte_type, buf->len);
		
		uint8_t *raw_data;
		uint16_t remaining;
		//Note the 3 * here so that we don't overflow the buffer. 
		//The +3 accounts for the NULL terminating and the end "]\n"
		char out[3 * RAW_PERIODIC_DATA_MAX_LEN +3], *put = out; 
		for (remaining = buf->len, raw_data =raw_periodic_data; remaining>0; remaining--, raw_data++)
		{
			
			uint8_t byte = net_buf_simple_pull_u8(buf);
			*raw_data = byte;
			put += sprintf(put, "%i, ", byte);
			
		}
		strcat(put, "]\n");
		printk("%s", out);


		got_per_adv_data = true;
	}
		
}

static struct bt_le_per_adv_sync_cb sync_callbacks = {
	.synced = sync_cb,
	.term = term_cb,
	.recv = recv_cb
	//.biginfo = TODO!
};

int main(void)
{
	struct bt_le_per_adv_sync_param sync_create_param;
	struct bt_le_per_adv_sync *sync;
	int err;

	printk("Starting Periodic Advertising Synchronization Demo\n");

#ifdef CONFIG_PER_BLINK_LED0
	printk("Checking LED device...");
	if (!gpio_is_ready_dt(&led)) {
		printk("failed.\n");
		return 0;
	}
	printk("done.\n");

	printk("Configuring GPIO pin...");
	err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (err) {
		printk("failed.\n");
		return 0;
	}
	printk("done.\n");

	k_work_init_delayable(&blink_work, blink_timeout);
#endif /* CONFIG_PER_BLINK_LED0 */

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Scan callbacks register...");
	bt_le_scan_cb_register(&scan_callbacks);
	printk("success.\n");

	printk("Periodic Advertising callbacks register...");
	bt_le_per_adv_sync_cb_register(&sync_callbacks);
	printk("Success.\n");

	printk("Start scanning...");
	err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, NULL);
	if (err) {
		printk("failed (err %d)\n", err);
		return 0;
	}
	printk("success.\n");

	do {
#ifdef CONFIG_PER_BLINK_LED0
		struct k_work_sync work_sync;

		printk("Start blinking LED...\n");
		led_is_on = false;
		gpio_pin_set(led.port, led.pin, (int)led_is_on);
		k_work_schedule(&blink_work, BLINK_ONOFF);
#endif /* CONFIG_PER_BLINK_LED0 */

		printk("Waiting for periodic advertising...\n");
		per_adv_found = false;
		err = k_sem_take(&sem_per_adv, K_FOREVER);
		if (err) {
			printk("failed (err %d)\n", err);
			return 0;
		}
		printk("Found periodic advertising.\n");

		printk("Creating Periodic Advertising Sync...");
		bt_addr_le_copy(&sync_create_param.addr, &per_addr);
		sync_create_param.options = 0;
		sync_create_param.sid = per_sid;
		sync_create_param.skip = 0;
		sync_create_param.timeout = per_adv_sync_timeout;
		err = bt_le_per_adv_sync_create(&sync_create_param, &sync);
		if (err) {
			printk("failed (err %d)\n", err);
			return 0;
		}
		printk("success.\n");

		printk("Waiting for periodic sync...\n");
		err = k_sem_take(&sem_per_sync, TIMEOUT_SYNC_CREATE);
		if (err) {
			printk("failed (err %d)\n", err);

			printk("Deleting Periodic Advertising Sync...");
			err = bt_le_per_adv_sync_delete(sync);
			if (err) {
				printk("failed (err %d)\n", err);
				return 0;
			}
			continue;
		}
		printk("Periodic sync established.\n");

#ifdef CONFIG_PER_BLINK_LED0
		printk("Stop blinking LED.\n");
		k_work_cancel_delayable_sync(&blink_work, &work_sync);

		/* Keep LED on */
		led_is_on = true;
		gpio_pin_set(led.port, led.pin, (int)led_is_on);
#endif /* CONFIG_PER_BLINK_LED0 */

		printk("Waiting for periodic sync lost...\n");
		err = k_sem_take(&sem_per_sync_lost, K_FOREVER);
		if (err) {
			printk("failed (err %d)\n", err);
			return 0;
		}
		printk("Periodic sync lost.\n");
	} while (true);
}
