/* This file is based on the periodic_sync sample: 
 * https://github.com/nrfconnect/sdk-zephyr/blob/main/samples/bluetooth/periodic_sync/src/main.c
 * and the iso_receive sample:
 * https://github.com/zephyrproject-rtos/zephyr/blob/main/samples/bluetooth/iso_receive/src/main.c
 * 
 * which are subject to the following license:
 * 
 * Copyright (c) 2020-2024 Nordic Semiconductor ASA
 * 
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdint.h>

#include <zephyr/kernel.h> //kernel services
#include <zephyr/bluetooth/gap.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/iso.h>


#define TIMEOUT_SYNC_CREATE K_SECONDS(10)
#define NAME_LEN            30

///Change this to your Auracast Broadcast name
#define BROADCAST_NAME "Tomer"
#define RAW_PERIODIC_DATA_MAX_LEN 200
#define MAX_OCTETS_PER_FRAME 200
#define BIS_ISO_CHAN_COUNT 2
#define CONFIG_ISO_PRINT_INTERVAL 360
static uint32_t iso_recv_count = 0;
static bool inc = true;

static K_SEM_DEFINE(sem_big_sync, 0, BIS_ISO_CHAN_COUNT);
static K_SEM_DEFINE(sem_big_sync_lost, 0, BIS_ISO_CHAN_COUNT);

static bool         got_per_adv_data;
static bool 		got_big_info;


static bool         per_adv_found;
static bt_addr_le_t per_addr;
static uint16_t per_adv_sync_timeout;
static uint8_t      per_sid;

static K_SEM_DEFINE(sem_per_adv, 0, 1);
static K_SEM_DEFINE(sem_per_sync, 0, 1);
static K_SEM_DEFINE(sem_per_sync_lost, 0, 1);


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

/*
The raw data results (extended scan raw data) for high-quality broadcast I get are:
	The data is [6, 22, 82, 24, 7, 111, 34, 5, 22, 86, 24, 4, 0, 6, 48, 84, 111, 109, 101, 114]
				 ^              ^   ^   ^ these 3 bytes that vary are the broadcast id.
				 Length, AD Type (22; 0x16 in hex)--Service Data - 16-bit UUID,
 
The Broadcast Audio Announcement Service UUID is 0x1852
which is 0b11000 01010010 which is 24 82 -- you get the UUID transmitted backwards!
It is transmitted LSB (least significant bit) first.
See  3.7.2.1. in  BAP (https://www.bluetooth.com/specifications/specs/basic-audio-profile-1-0-1/) 
to see where this is specified.

So the second UUID is 86 24 -- flip order 24 86 -- 0b11000 01010110 -- 0b1100001010110 
Which is 0x1856 which is the UUID for Public Broadcast Announcement service

The public broadcast announcement is detailed in section 4 of PBP.
so 5 is the length, 22 is the type, 86 24 is the uuid.
4 is the public broadcast announcement features and 0 is the metadata length.
We need to look at 4 bitwise. 4 is 0b100. Since all but the first 3 bits are RFU
Reserved for future use. The bits are transmitted in reverse order, that is 
it is transmitted LSB (least significant bit) first.
so that bit 0 is 0, bit 1 is 0, and bit 2 is 1.
Which makes sense as that would mean no encryption,
Standard Quality Public Broadcast Audio has no configuration present,
while High Quality Public Broadcast Audio does have an audio configuration present.

so 6 is the Length, 48 is the broadcast name type (from assigned numbers
48 is 0x30 and 0x30=Broadcast_Name), and the value is the name of the broadcast.
Note having the broadcast name here is required by Bluetooth (see PBP section 5:
https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/11452-PBP-html5/out/en/index-en.html#UUID-de267bf3-7adb-d590-1d82-5cc803f2bf51) 

From reading 4.3 in PBP:
If a PBS transmits the Public Broadcast Announcement with bit 2 of the 
Public Broadcast Announcement features field set to a value of 0b1 
(to show High Quality Public Broadcast Audio), 
the advertising set used to transmit the Public Broadcast Announcement (i.e this extended advertising data)
points to a BIG, which shall include at least one broadcast Audio Stream configuration setting listed in Table 4.2. 

From BAP: 3.7.2.1.1. Broadcast_ID:
For each BIG, the Broadcast Source shall generate a Broadcast_ID according to the requirements 
for random number generation as defined in Volume 3, Part H, Section 2 in [1]. 
The Broadcast_ID shall not change for the lifetime of the BIG. 


In data_cb() below we parse each LTV triplet in this raw data.
*/



/// @brief This is called on each Length Type Value (LTV) triplet in a particular scan result.
/// When it returns true the parsing continues. When it returns false the parsing stops.
///
/// 48 in hex is 0x30 which is Broadcast_Name in the Bluetooth assigned numbers.
/// This is the only way for us to identify the Auracast broadcast 
/// as the (broadcast source) device address is randomized.
static bool data_cb(struct bt_data *data, void *user_data)
{
	char *name = user_data;
	uint8_t len;

	switch (data->type) {
	case BT_DATA_BROADCAST_NAME:
		len = MIN(data->data_len, NAME_LEN - 1);
		memcpy(name, data->data, len);
		name[len] = '\0';
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
	
	if (found_auracast && !per_adv_found) {

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
	

		if (info->interval) {
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


/// @brief The callback on receving periodic adv report.
/// @remarks 
///We got output: 
/// PER_ADV_SYNC[0]: [DEVICE]: 29:41:D7:F3:46:F9 (random), 
/// tx_power 127, RSSI -38, CTE 0, 
/// data length 65, 
/// data: [64, 22, 81, 24, 64, 156, 0, 1, 2, 6, 0, 0, 0, 0, 10, 2, 1, 8, 2, 2, 1, 3, 4, 
/// 	120, 0, 23, 3, 2, 4, 0, 8, 3, 85, 110, 107, 110, 111, 119, 110, 2, 5, 2, 6, 11, 84, 
/// 	111, 109, 101, 114, 1, 6, 5, 3, 1, 0, 0, 0, 2, 6, 5, 3, 2, 0, 0, 0, ]
///We can parse this according
///to https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/16212-BAP-html5/out/en/index-en.html#UUID-8810d9ca-716a-9902-b58b-8d35b5e9a149
/// Here is the parsing result (remember Bluetooth provides data in LTV triplets)
/// 22= 0x16 =16 bit service uuid
/// so we read the next two as 24 81 (Notice the order swap)
/// which corresponds to the following 16bits: 0001100001010001 = 0x1851 
/// which is Basic Audio Announcement Service
/// 64, 156, 0 = presentation delay in µs. 3 octets in binary (Remember LSB first) 0b1001110001000000 = 40000 in decimal.
/// So our presentation delay is 40ms.
///
/// 1 = number of subgroups
/// 2 = number of BIS
/// 6 = Coding_Format (part of codec info) which in this case is LC3. The following 0, 0, 0, 0, are irrelevant here.
/// 10 = Length of the Codec_Specific_Configuration for the [ith] subgroup
/// 2, 1, 8, 2, 2, 1, 3, 4, 120, 0 = Codec-specific configuration parameters for the [ith] subgroup
/// Based on 6.12.5 Codec_Specific_Configuration LTV structures in BT assigned numbers, here this is:
/// 8=  48000 Hz Sampling Frequency, 1 =  Use 10 ms codec frames, 
/// 120 = Number of octets used per codec frame.
/// Note this results in a bit rate of 96 kbps (see Table 3.17 here:
/// https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/16212-BAP-html5/out/en/index-en.html#UUID-90a900d2-de3f-acff-c7ff-d9961e742b4a).
///
/// 23 = Metadata length for the subgroup (series of LTV values)
/// 3, 2, 4, 0 = Streaming audio context is media
/// 8, 3, 85, 110, 107, 110, 111, 119, 110, = Program info (Title and/or summary of Audio Stream content: UTF-8 format)
/// Which here is "Unknown"
/// 2, 5, 2, = CCID (The Content Control ID) value 2 
///(The ID of the content control service instance containing this characteristic.)
///
/// 6, 11, 84, 111, 109, 101, 114, = The UTF-8 string of the Broadcast_Name AD Type. 
///This is simply our broadcast name (here "Tomer").
///
/// 1 = BIS index
/// For BIS[1]:
/// 	6 = Length of the Codec_Specific_Configuration 
///		5, 3, 1, 0, 0, 0, = Audio_Channel_Allocation (4-octet bitfield of Audio Location values) so here it is 0b1.
/// 	Which is Front Left
///
/// 2= BIS index
/// For BIS[2]:
/// 	6 = Length of the Codec_Specific_Configuration
///     5, 3, 2, 0, 0, 0, = Audio_Channel_Allocation (4-octet bitfield of Audio Location values) so here it is 0b10.
/// 	Which is Front Right.
static void recv_cb(struct bt_le_per_adv_sync *sync,
		    const struct bt_le_per_adv_sync_recv_info *info,
		    struct net_buf_simple *buf)
{
	if (!got_per_adv_data) {
		
	
		char le_addr[BT_ADDR_LE_STR_LEN];
		

		bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

		printk("PER_ADV_SYNC[%u]: [DEVICE]: %s, tx_power %i, "
			"RSSI %i, CTE %u, data length %u, data: [",
			bt_le_per_adv_sync_get_index(sync), le_addr, info->tx_power,
			info->rssi, info->cte_type, buf->len);
		
		
		uint16_t remaining;
		//Note the 3 * here so that we don't overflow the buffer. 
		//The +3 accounts for the NULL terminating and the end "]\n"
		char out[3 * RAW_PERIODIC_DATA_MAX_LEN +3], *put = out; 
		for (remaining = buf->len; remaining>0; remaining--)
		{
			
			uint8_t byte = net_buf_simple_pull_u8(buf);
			put += sprintf(put, "%i, ", byte);
			
		}
		strcat(put, "]\n");
		printk("%s", out);


		got_per_adv_data = true;
	}
		
}

static void biginfo_cb(struct bt_le_per_adv_sync *sync,
		       const struct bt_iso_biginfo *biginfo)
{

	if (!got_big_info)
	{
		char le_addr[BT_ADDR_LE_STR_LEN];

		bt_addr_le_to_str(biginfo->addr, le_addr, sizeof(le_addr));

		printk("BIG INFO[%u]: [DEVICE]: %s, sid 0x%02x, "
			"num_bis %u, nse %u, interval 0x%04x (%u ms), "
			"bn %u, pto %u, irc %u, max_pdu %u, "
			"sdu_interval %u us, max_sdu %u, phy %s, "
			"%s framing, %sencrypted\n",
			bt_le_per_adv_sync_get_index(sync), le_addr, biginfo->sid,
			biginfo->num_bis, biginfo->sub_evt_count,
			biginfo->iso_interval,
			(biginfo->iso_interval * 5 / 4),
			biginfo->burst_number, biginfo->offset,
			biginfo->rep_count, biginfo->max_pdu, biginfo->sdu_interval,
			biginfo->max_sdu, phy2str(biginfo->phy),
			biginfo->framing ? "with" : "without",
			biginfo->encryption ? "" : "not ");
			/*From 4.4.6.5 
			(https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-54/out/en/low-energy-controller/link-layer-specification.html)
			The Framed parameter of a BIG shall indicate whether all the constituent BISes are framed or unframed. 
			Framed BIGs shall only use framed BIS Data PDUs to carry data; 
			unframed BIGs shall only use unframed BIS Data PDUs to carry data.
			*/

		got_big_info= true;
	}
}

static struct bt_le_per_adv_sync_cb sync_callbacks = {
	.synced = sync_cb,
	.term = term_cb,
	.recv = recv_cb,
	.biginfo = biginfo_cb
};

///Reproduced here for convenience.
/// @brief Channel recv callback
/// @param chan The channel receiving data.
/// @param buf Buffer containing incoming data.
/// @param info Pointer to the metadata for the buffer. 
///The lifetime of the pointer is linked to the lifetime of the net_buf. 
///Metadata such as sequence number and timestamp can be provided by the bluetooth controller.
static void iso_recv(struct bt_iso_chan *chan, const struct bt_iso_recv_info *info,
		struct net_buf *buf)
{
	
	//Note the buffer has non-zero length *only* when the phone is *actually* transmitting sound
	//(say when a YouTube video plays; If the video is paused, the buffer has length zero again).

	if ((iso_recv_count % CONFIG_ISO_PRINT_INTERVAL) == 0) {
		if(buf->len)
		{
			printk("Actively receiving audio!\n");
		}

		printk("Incoming data channel %p flags 0x%x seq_num %u ts %u len %u data:[",
			chan, info->flags, info->seq_num, info->ts, buf->len);
		
		
		uint8_t *raw_data;
		uint16_t remaining;
		//Note the 3 * here so that we don't overflow the buffer. 
		//The +3 accounts for the NULL terminating and the end "]\n"
		char out[MAX_OCTETS_PER_FRAME *3  + 3] = {0}, *put = out; 
		for (raw_data = buf->data, remaining = buf->len; remaining>0; remaining--, raw_data++)
		{
			uint8_t byte = *raw_data;
			put += sprintf(put, "%i, ", byte);
			
		}
		strcat(put, "]\n");
		printk("%s", out);
		
		/*The raw output (if we didn't filter it) is like:
			flags 9 means BT_ISO_FLAGS_TS and BT_ISO_FLAGS_VALID (both the packet and timestamp are valid)
			Incoming data channel 0x20008134 flags 0x9 seq_num 37959 ts 457271418 len X data: Y
			Incoming data channel 0x20008148 flags 0x9 seq_num 37959 ts 457271418 len X data: Y
			Incoming data channel 0x20008134 flags 0x9 seq_num 37960 ts 457281418 len X data: Y
		*/
	}

	//slow down printing
	inc = !inc;
	if(inc){
		iso_recv_count++;
	}
}

static void iso_connected(struct bt_iso_chan *chan)
{
	const struct bt_iso_chan_path hci_path = {
		.pid = BT_ISO_DATA_PATH_HCI,
		.format = BT_HCI_CODING_FORMAT_TRANSPARENT //changing it to BT_HCI_CODING_FORMAT_LC3 causes errors
	};
	int err;

	printk("ISO Channel %p connected\n", chan);

	err = bt_iso_setup_data_path(chan, BT_HCI_DATAPATH_DIR_CTLR_TO_HOST, &hci_path);
	if (err != 0) {
		printk("Failed to setup ISO RX data path: %d\n", err);
		return;
	}

	k_sem_give(&sem_big_sync);
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	printk("ISO Channel %p disconnected with reason 0x%02x\n",
	       chan, reason);

	if (reason != BT_HCI_ERR_OP_CANCELLED_BY_HOST) {
		k_sem_give(&sem_big_sync_lost);
	}
}

static struct bt_iso_chan_ops iso_ops = {
	.recv		= iso_recv,
	.connected	= iso_connected,
	.disconnected	= iso_disconnected,
};

static struct bt_iso_chan_io_qos iso_rx_qos[BIS_ISO_CHAN_COUNT];

static struct bt_iso_chan_qos bis_iso_qos[] = {
	{ .rx = &iso_rx_qos[0], },
	{ .rx = &iso_rx_qos[1], },
};

static struct bt_iso_chan bis_iso_chan[] = {
	{ .ops = &iso_ops,
	  .qos = &bis_iso_qos[0], },
	{ .ops = &iso_ops,
	  .qos = &bis_iso_qos[1], },
};

static struct bt_iso_chan *bis[] = {
	&bis_iso_chan[0],
	&bis_iso_chan[1],
};

static struct bt_iso_big_sync_param big_sync_param = {
	.bis_channels = bis,
	.num_bis = BIS_ISO_CHAN_COUNT,
	.bis_bitfield = (BIT_MASK(BIS_ISO_CHAN_COUNT)),
	.mse = BT_ISO_SYNC_MSE_ANY, /* any number of subevents */
	.sync_timeout = 100, /* in 10 ms units */
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
		got_per_adv_data = false;
		got_big_info = false;
		err = k_sem_take(&sem_per_adv, K_FOREVER);
		if (err) {
			printk("failed (err %d)\n", err);
			return 0;
		}
		printk("Found periodic advertising.\n");

		//We can stop scanning after we found the periodic advertising (scanning is power hungry)
		err = bt_le_scan_stop();
		if (err != 0) {
			printk("bt_le_scan_stop failed with error %d, resetting\n", err);
			continue;
		}


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

		printk("Create BIG Sync...\n");
		struct bt_iso_big *big;
		err = bt_iso_big_sync(sync, &big_sync_param, &big);
		if (err) {
			printk("failed (err %d)\n", err);
			return 0;
		}
		printk("success.\n");

		for (uint8_t chan = 0; chan < BIS_ISO_CHAN_COUNT; chan++) {
			printk("Waiting for BIG sync chan %u...\n", chan);
			err = k_sem_take(&sem_big_sync, TIMEOUT_SYNC_CREATE);
			if (err) {
				break;
			}
			printk("BIG sync chan %u successful.\n", chan);
		}
		if (err) {
			printk("failed (err %d)\n", err);

			printk("BIG Sync Terminate...");
			err = bt_iso_big_terminate(big);
			if (err) {
				printk("failed (err %d)\n", err);
				return 0;
			}
			printk("done.\n");
			continue;
		}
		printk("BIG sync established.\n");

#ifdef CONFIG_PER_BLINK_LED0
		printk("Stop blinking LED.\n");
		k_work_cancel_delayable_sync(&blink_work, &work_sync);

		/* Keep LED on */
		led_is_on = true;
		gpio_pin_set(led.port, led.pin, (int)led_is_on);
#endif /* CONFIG_PER_BLINK_LED0 */

		for (uint8_t chan = 0; chan < BIS_ISO_CHAN_COUNT; chan++) {
			printk("Waiting for BIG sync lost chan %u...\n", chan);
			err = k_sem_take(&sem_big_sync_lost, K_FOREVER);
			if (err) {
				printk("failed (err %d)\n", err);
				return 0;
			}
			printk("BIG sync lost chan %u.\n", chan);
		}
		printk("BIG sync lost.\n");

		printk("Waiting for periodic sync lost...\n");
		err = k_sem_take(&sem_per_sync_lost, K_FOREVER);
		if (err) {
			printk("failed (err %d)\n", err);
			return 0;
		}
		printk("Periodic sync lost.\n");
	} while (true);
}
