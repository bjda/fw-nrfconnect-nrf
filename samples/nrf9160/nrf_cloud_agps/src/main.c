/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <stdio.h>
#include <lte_lc.h>
#include <net/cloud.h>
#include <net/socket.h>
#include <dk_buttons_and_leds.h>
#include <drivers/gps.h>
#include <nrf_cloud_agps.h>
#include <sys/base64.h>
#include <power/reboot.h>

static struct cloud_backend *cloud_backend;
static struct k_delayed_work cloud_update_work;
static u64_t start_search_timestamp;
static u64_t fix_timestamp;
static struct device *gps_dev;
static struct k_delayed_work gps_start_work;
static struct k_delayed_work reboot_work;

static void gps_start_work_fn(struct k_work *work);

static void cloud_update_work_fn(struct k_work *work)
{
	int err;

	printk("Publishing message: %s\n", CONFIG_CLOUD_MESSAGE);

	struct cloud_msg msg = {
		.qos = CLOUD_QOS_AT_MOST_ONCE,
		.endpoint.type = CLOUD_EP_TOPIC_MSG,
		.buf = CONFIG_CLOUD_MESSAGE,
		.len = sizeof(CONFIG_CLOUD_MESSAGE)
	};

	err = cloud_send(cloud_backend, &msg);
	if (err) {
		printk("cloud_send failed, error: %d\n", err);
	}
}

static void gps_start_work_fn(struct k_work *work)
{
	int err;
	static bool first = true;
	struct gps_config gps_cfg = {
		.nav_mode = GPS_NAV_MODE_PERIODIC,
		.power_mode = GPS_POWER_MODE_DISABLED,
		.timeout = 120,
		.interval = 240,
	};

	ARG_UNUSED(work);

	err = gps_start(gps_dev, &gps_cfg);
	if (err) {
		printk("Failed to start GPS, error: %d\n", err);
		return;
	}

	if (first) {
		first = false;
		gps_cfg.delete_agps_data = false;
	}

	start_search_timestamp = k_uptime_get();

	printk("GPS started\n");
}

static void process_agps_data(char *buf, size_t len)
{
	int err;
	size_t olen;
	char decoded_buf[2500];

	printk("Starting A-GPS test\n");

	err = base64_decode(decoded_buf, sizeof(decoded_buf), &olen, buf, len);
	if (err) {
		printk("Base64 encoded data could not be decoded, err: %d\n", err);
		return;
	}

	printk("Base64: %d bytes written to buffer\n", olen);

	err = nrf_cloud_agps_process(decoded_buf, olen, NULL);
	if (err) {
		printk("A-GPS failed, error: %d\n", err);
		printk("GPS starting without new assistance data...\n");
	} else {
		printk("A-GPS success\n");
		printk("GPS starting...\n");
	}

	k_delayed_work_submit(&gps_start_work, K_SECONDS(3));
}

static void on_ready_evt(void)
{
	int err;

	err = nrf_cloud_agps_request_all();
	if (err) {
		printk("Failed to request A-GPS data, error: %d\n", err);
		return;
	}

	printk("A-GPS data request sent, waiting for response...\n");
}

static void cloud_event_handler(const struct cloud_backend *const backend,
			 const struct cloud_event *const evt,
			 void *user_data)
{
	ARG_UNUSED(backend);
	ARG_UNUSED(user_data);

	switch (evt->type) {
	case CLOUD_EVT_CONNECTED:
		printk("CLOUD_EVT_CONNECTED\n");
		break;
	case CLOUD_EVT_READY:
		printk("CLOUD_EVT_READY\n");
		on_ready_evt();
		break;
	case CLOUD_EVT_DISCONNECTED:
		printk("CLOUD_EVT_DISCONNECTED\n");
		break;
	case CLOUD_EVT_ERROR:
		printk("CLOUD_EVT_ERROR\n");
		break;
	case CLOUD_EVT_DATA_SENT:
		printk("CLOUD_EVT_DATA_SENT\n");
		break;
	case CLOUD_EVT_DATA_RECEIVED:
		printk("CLOUD_EVT_DATA_RECEIVED\n");

		if (evt->data.msg.buf[0] == '{') {
			int ret;
			ret = strncmp(evt->data.msg.buf,
				      "{\"reboot\":true}",
				      strlen("{\"reboot\":true}"));
			if (ret == 0) {
				k_delayed_work_submit(&reboot_work, K_NO_WAIT);
			}
			break;
		}

		printk("Assumed base64 encoded AGPS data incoming\n");

		process_agps_data(evt->data.msg.buf, evt->data.msg.len);
		break;
	case CLOUD_EVT_PAIR_REQUEST:
		printk("CLOUD_EVT_PAIR_REQUEST\n");
		break;
	case CLOUD_EVT_PAIR_DONE:
		printk("CLOUD_EVT_PAIR_DONE\n");
		break;
	case CLOUD_EVT_FOTA_DONE:
		printk("CLOUD_EVT_FOTA_DONE\n");
		break;
	default:
		printk("Unknown cloud event type: %d\n", evt->type);
		break;
	}
}

static void print_pvt_data(struct gps_pvt *pvt_data)
{
	printf("Longitude:  %f\n", pvt_data->longitude);
	printf("Latitude:   %f\n", pvt_data->latitude);
	printf("Altitude:   %f\n", pvt_data->altitude);
	printf("Speed:      %f\n", pvt_data->speed);
	printf("Heading:    %f\n", pvt_data->heading);
	printk("Date:       %02u-%02u-%02u\n", pvt_data->datetime.day,
					       pvt_data->datetime.month,
					       pvt_data->datetime.year);
	printk("Time (UTC): %02u:%02u:%02u\n", pvt_data->datetime.hour,
					       pvt_data->datetime.minute,
					       pvt_data->datetime.seconds);
}

static void print_satellite_stats(struct gps_pvt *pvt_data)
{
	u8_t tracked = 0;
	u32_t tracked_sats = 0;
	static u32_t prev_tracked_sats;

	for (int i = 0; i < GPS_PVT_MAX_SV_COUNT; ++i) {
		if ((pvt_data->sv[i].sv > 0) &&
		    (pvt_data->sv[i].sv < 33)) {
			tracked++;
			tracked_sats |= BIT(pvt_data->sv[i].sv - 1);
		}
	}

	if ((tracked_sats == 0) || (tracked_sats == prev_tracked_sats)) {
		if (tracked_sats != prev_tracked_sats) {
			prev_tracked_sats = tracked_sats;
			printk("Tracking no satellites\n");
		}

		return;
	}

	prev_tracked_sats = tracked_sats;

	printk("Tracking:  ");

	for (size_t i = 0; i < 32; i++) {
		if (tracked_sats & BIT(i)) {
			printk("%d  ", i + 1);
		}
	}

	printk("\n");
	printk("Searching for %lld seconds\n",
			(k_uptime_get() - start_search_timestamp) / 1000);
}

static void send_nmea(char *nmea)
{
	int err;
	struct cloud_msg msg = {
		.qos = CLOUD_QOS_AT_MOST_ONCE,
		.endpoint.type = CLOUD_EP_TOPIC_MSG,
	};
	char buf[150];
	size_t len = snprintf(buf, sizeof(buf),
		"{"
			"\"appId\":\"GPS\","
			"\"data\":\"%s\","
			"\"messageType\":\"DATA\""
		"}", nmea);

	if (len < 0) {
		printk("Failed to create GPS cloud message\n");
		return;
	}

	msg.buf = buf;
	msg.len = len;

	err = cloud_send(cloud_backend, &msg);
	if (err) {
		printk("Failed to send message to cloud, error: %d\n", err);
		return;
	}

	printk("Message sent to cloud\n");
}

static void gps_handler(struct device *dev, struct gps_event *evt)
{
	ARG_UNUSED(dev);

	switch (evt->type) {
	case GPS_EVT_SEARCH_STARTED:
		printk("GPS_EVT_SEARCH_STARTED\n");
		break;
	case GPS_EVT_SEARCH_STOPPED:
		printk("GPS_EVT_SEARCH_STOPPED\n");
		break;
	case GPS_EVT_SEARCH_TIMEOUT:
		printk("GPS_EVT_SEARCH_TIMEOUT\n");
		break;
	case GPS_EVT_HAS_TIME_WINDOW:
		printk("GPS_EVT_HAS_TIME_WINDOW\n");
		break;
	case GPS_EVT_AGPS_DATA_NEEDED:
		printk("GPS_EVT_AGPS_DATA_NEEDED\n");
		break;
	case GPS_EVT_PVT:
		print_satellite_stats(&evt->pvt);
		break;
	case GPS_EVT_PVT_FIX:
		fix_timestamp = k_uptime_get();

		printk("---------       FIX       ---------\n");
		printk("Time to fix: %d s\n\n",
			(u32_t)(fix_timestamp - start_search_timestamp) / 1000);
		print_pvt_data(&evt->pvt);
		printk("-----------------------------------\n");
		break;
	case GPS_EVT_NMEA_FIX:
		send_nmea(evt->nmea.buf);
		break;
	default:
		break;
	}
}

static void reboot_work_fn(struct k_work *work)
{
	printk("Rebooting...\n");
	k_sleep(K_SECONDS(2));
	sys_reboot(0);
}

static void work_init(void)
{
	k_delayed_work_init(&cloud_update_work, cloud_update_work_fn);
	k_delayed_work_init(&gps_start_work, gps_start_work_fn);
	k_delayed_work_init(&reboot_work, reboot_work_fn);
}

static void modem_configure(void)
{
	if (IS_ENABLED(CONFIG_LTE_AUTO_INIT_AND_CONNECT)) {
		/* Do nothing, modem is already turned on
		 * and connected.
		 */
	} else {
		int err;

		printk("Connecting to LTE network. ");
		printk("This may take several minutes.\n");

		err = lte_lc_init_and_connect();
		if (err) {
			printk("LTE link could not be established.\n");
		}

		printk("Connected to LTE network\n");

#if defined(CONFIG_POWER_SAVING_MODE_ENABLE)
		err = lte_lc_psm_req(true);
		if (err) {
			printk("lte_lc_psm_req, error: %d\n", err);
		}

		printk("PSM mode requested\n");
#endif
	}
}

static void button_handler(u32_t button_states, u32_t has_changed)
{
	if (has_changed & button_states & DK_BTN1_MSK) {
		k_delayed_work_submit(&cloud_update_work, K_NO_WAIT);
		k_delayed_work_submit(&reboot_work, K_SECONDS(3));
	} else if (has_changed & ~button_states & DK_BTN1_MSK) {
		k_delayed_work_cancel(&reboot_work);
	}
}

void main(void)
{
	int err;

	printk("Cloud client has started\n");

	cloud_backend = cloud_get_binding("NRF_CLOUD");
	__ASSERT(cloud_backend, "Could not get binding to cloud backend");

	err = cloud_init(cloud_backend, cloud_event_handler);
	if (err) {
		printk("Cloud backend could not be initialized, error: %d\n",
			err);
		return;
	}

	work_init();
	modem_configure();

	gps_dev = device_get_binding("NRF9160_GPS");
	if (gps_dev == NULL) {
		printk("Could not get binding to nRF9160 GPS\n");
		return;
	}

	err = gps_init(gps_dev, gps_handler);
	if (err) {
		printk("Could not initialize GPS, error: %d\n", err);
		return;
	}

	err = dk_buttons_init(button_handler);
	if (err) {
		printk("dk_buttons_init, error: %d\n", err);
	}

	err = cloud_connect(cloud_backend);
	if (err) {
		printk("cloud_connect, error: %d\n", err);
	}

	struct pollfd fds[] = {
		{
			.fd = cloud_backend->config->socket,
			.events = POLLIN
		}
	};

	while (true) {
		err = poll(fds, ARRAY_SIZE(fds),
			   cloud_keepalive_time_left(cloud_backend));
		if (err < 0) {
			printk("poll() returned an error: %d\n", err);
			continue;
		}

		if (err == 0) {
			cloud_ping(cloud_backend);
			continue;
		}

		if ((fds[0].revents & POLLIN) == POLLIN) {
			cloud_input(cloud_backend);
		}

		if ((fds[0].revents & POLLNVAL) == POLLNVAL) {
			printk("Socket error: POLLNVAL\n");
			printk("The cloud socket was unexpectedly closed.\n");
			return;
		}

		if ((fds[0].revents & POLLHUP) == POLLHUP) {
			printk("Socket error: POLLHUP\n");
			printk("Connection was closed by the cloud.\n");
			return;
		}

		if ((fds[0].revents & POLLERR) == POLLERR) {
			printk("Socket error: POLLERR\n");
			printk("Cloud connection was unexpectedly closed.\n");
			return;
		}
	}
}
