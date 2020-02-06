/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <stdint.h>
#include <stdio.h>

#include <soc.h>
#include <nrfx.h>

#include <nrf_cloud_agps.h>
#include <sys/base64.h>

#include <nrf_socket.h>
#include <net/socket.h>

#include <shell/shell.h>
#include <shell/shell_uart.h>

#include <at_cmd.h>
#include <at_notif.h>

#include <lte_lc.h>
#include <net/cloud.h>
#include <modem_info.h>

#include <drivers/gps.h>

static struct device *gps_dev;
static u64_t start_search_timestamp;
static u64_t fix_timestamp;
static struct cloud_backend *cloud_backend;
static K_THREAD_STACK_DEFINE(cloud_thread_stack, 8192);
static struct k_thread cloud_thread;
static K_SEM_DEFINE(cloud_connect_sem, 1, 1);
static struct gps_config gps_cfg = {
	.nav_mode = GPS_NAV_MODE_SINGLE_FIX,
	.power_mode = GPS_POWER_MODE_DISABLED,
	.timeout = 20,
};

void bsd_recoverable_error_handler(uint32_t error)
{
	printf("Err: %lu\n", (unsigned long)error);
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
	} else {
		printk("A-GPS success\n");
	}
}

static void send_agps_request(
	const struct shell *shell, size_t argc, char **argv)
{
	int err;
	enum gps_agps_type types[10];
	size_t count = 0;
	char *element;

	if (argc == 1) {
		err = nrf_cloud_agps_request_all();
		if (err) {
			goto fail;
		}

		goto success;
	}

	if (strlen(argv[1]) == 1) {
		types[count++] = (enum gps_agps_type)strtoul(argv[1], NULL, 10);
	} else {
		/* Create array of enums from string */
		element = strtok(argv[1], ",");

		while(element != NULL) {
			types[count++] =
				(enum gps_agps_type)strtoul(element, NULL, 10);
			element = strtok(NULL, ",");
		}
	}

	err = nrf_cloud_agps_request(types, count);
	if (err) {
		goto fail;
	}

success:
	shell_print(shell, "A-GPS request sent");

	return;
fail:
	shell_print(shell, "Failed to request A-GPS data: %d", err);
}

static void cloud_event_handler(const struct cloud_backend *const backend,
			 const struct cloud_event *const evt,
			 void *user_data)
{
	ARG_UNUSED(user_data);

	switch (evt->type) {
	case CLOUD_EVT_CONNECTED:
		printk("CLOUD_EVT_CONNECTED\n");
		break;
	case CLOUD_EVT_READY:
		printk("CLOUD_EVT_READY\n");
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
			printk("JSON received, ignoring message\n");
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

static int cloud_thread_fn(void *arg1, void *arg2, void *arg3)
{
	int ret;
	enum lte_lc_nw_reg_status reg_status;

	cloud_backend = cloud_get_binding("NRF_CLOUD");
	__ASSERT(cloud_backend != NULL, "nRF Cloud backend not found");

connect:
	k_sem_take(&cloud_connect_sem, K_FOREVER);

	ret = cloud_init(cloud_backend, cloud_event_handler);
	if (ret) {
		printk("Cloud backend could not be initialized, error: %d\n",
			ret);
		printk("Terminating...\n");
		return ret;
	}


	lte_lc_nw_reg_status_get(&reg_status);

	while ((reg_status != LTE_LC_NW_REG_REGISTERED_HOME) &&
	       (reg_status != LTE_LC_NW_REG_REGISTERED_ROAMING))
	{
		k_sleep(K_SECONDS(2));
		lte_lc_nw_reg_status_get(&reg_status);
	}

	ret = cloud_connect(cloud_backend);
	if (ret) {
		printk("cloud_connect failed: %d\n", ret);
		printk("Trerminating thread\n");

		return -1;
	}

	struct pollfd fds[] = {
		{
			.fd = cloud_backend->config->socket,
			.events = POLLIN
		}
	};

	while (true) {
		/* The timeout is set to (keepalive / 3), so that the worst case
		 * time between two messages from device to broker is
		 * ((4 / 3) * keepalive + connection overhead), which is within
		 * MQTT specification of (1.5 * keepalive) before the broker
		 * must close the connection.
		 */
		ret = poll(fds, ARRAY_SIZE(fds),
			K_SECONDS(CONFIG_MQTT_KEEPALIVE / 3));

		if (ret < 0) {
			printk("poll() returned an error: %d\n", ret);
			continue;
		}

		if (ret == 0) {
			cloud_ping(cloud_backend);
			continue;
		}

		if ((fds[0].revents & POLLIN) == POLLIN) {
			cloud_input(cloud_backend);
		}

		if ((fds[0].revents & POLLNVAL) == POLLNVAL) {
			printk("Socket error: POLLNVAL\n");
			printk("The cloud socket was unexpectedly closed.\n");
			break;
		}

		if ((fds[0].revents & POLLHUP) == POLLHUP) {
			printk("Socket error: POLLHUP\n");
			printk("Connection was closed by the cloud.\n");
			break;
		}

		if ((fds[0].revents & POLLERR) == POLLERR) {
			printk("Socket error: POLLERR\n");
			printk("Cloud connection was unexpectedly closed.\n");
			break;
		}
	}

	goto connect;
}

/**@brief Configures modem to provide LTE link. Blocks until link is
 * successfully established.
 */
static int modem_configure(void)
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
			return err;
		}

		printk("Connected to LTE network\n");
	}

	return 0;
}

static void notif_handler(void *context, const char *response)
{
	printk("\nAT notification:\n");
	printk("%s\n", response);
}

static void handle_at_cmd(const struct shell *shell, size_t argc, char **argv)
{
	int err;
	char response[200];
	enum at_cmd_state state;

	if (argc <= 1) {
		shell_print(shell, "AT command was not provided");
		return;
	}

	err = at_cmd_write(argv[1], response, sizeof(response), &state);
	if (err < 0) {
		shell_print(shell,
			"Error while processing AT command: %d", err);
		state = AT_CMD_ERROR;
	}

	switch (state) {
	case AT_CMD_OK:
		shell_print(shell, "%s", response);
		shell_print(shell, "OK");
		break;
	case AT_CMD_ERROR:
		shell_print(shell, "ERROR");
		break;
	default:
		break;
	}
}

extern void agps_print_enable(bool enable);

static void agps_print_on(void)
{
	agps_print_enable(true);
}

static void agps_print_off(void)
{
	agps_print_enable(false);
}

static int start_gps(void)
{
	int err = gps_start(gps_dev, &gps_cfg);
	if (err) {
		printk("GPS could not be started, error: %d\n", err);
		return err;
	} else {
		printk("GPS started\n");
	}

	start_search_timestamp = k_uptime_get();

	return err;
}

static void stop_gps(void)
{
	int err = gps_stop(gps_dev);
	if (err) {
		printk("GPS could not be stopped, error: %d\n", err);
	} else {
		printk("GPS stopped\n");
	}
}

static void delete_agps(void)
{
	int err;
	bool prev_delete_value = gps_cfg.delete_agps_data;

	gps_cfg.delete_agps_data = true;

	err = gps_start(gps_dev, &gps_cfg);

	gps_cfg.delete_agps_data = prev_delete_value;

	if (err) {
		printk("Failed to delete A-GPS data, error: %d\n", err);
		return;
	}

	err = gps_stop(gps_dev);
	if (err) {
		printk("Failed to stop GPS, error: %d\n", err);
		return;
	}

	printk("GPS stopped and A-GPS data deleted\n");
}

static void lte_enable(void)
{
	int err = at_cmd_write("AT+CFUN=21", NULL, 0, NULL);
	if (err) {
		printk("Could not enable LTE, error: %d\n", err);
	}

	k_sem_give(&cloud_connect_sem);
}

static void lte_disable(void)
{
	int err;

	cloud_disconnect(cloud_backend);

	err = at_cmd_write("AT+CFUN=20", NULL, 0, NULL);
	if (err) {
		printk("Could not disable LTE, error: %d\n", err);
	}
}

static void lte_psm_enable(void)
{
	int err = lte_lc_psm_req(true);
	if (err) {
		printk("Failed to request PSM, error: %d\n", err);
		return;
	}

	printk("PSM requested\n");
}

static void lte_psm_disable(void)
{
	int err = lte_lc_psm_req(false);
	if (err) {
		printk("Failed to disable PSM, error: %d\n", err);
		return;
	}

	printk("PSM disabled\n");
}

static void lte_edrx_enable(void)
{
	int err = lte_lc_edrx_req(true);
	if (err) {
		printk("Failed to request eDRX, error: %d\n", err);
		return;
	}

	printk("eDRX requested\n");
}

static void lte_edrx_disable(void)
{
	int err = lte_lc_edrx_req(false);
	if (err) {
		printk("Failed to disable eDRX, error: %d\n", err);
		return;
	}

	printk("eDRX disabled\n");
}

static void agps_fresh(const struct shell *shell, size_t argc, char **argv)
{
	delete_agps();
	send_agps_request(shell, argc, argv);
}

SHELL_STATIC_SUBCMD_SET_CREATE(agps_cmd_print,
	SHELL_CMD(on, NULL, "Turn ON A-GPS data dump", agps_print_on),
	SHELL_CMD(off, NULL, "Turn OFF A-GPS data dump", agps_print_off),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_STATIC_SUBCMD_SET_CREATE(agps_cmds,
	SHELL_CMD_ARG(
		request, NULL, "Request A-GPS data", send_agps_request, 0, 1),
	SHELL_CMD_ARG(
		delete, NULL, "Delete A-GPS data from the GPS",
		delete_agps, 0, 0),
	SHELL_CMD_ARG(replace, NULL,
		      "Delete and replace A-GPS data", agps_fresh, 0, 1),
	SHELL_CMD_ARG(print, &agps_cmd_print, "Enable A-GPS data dumping",
		      NULL, 0, 0),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);
SHELL_CMD_REGISTER(agps, &agps_cmds, "A-GPS commands", NULL);

SHELL_CMD_REGISTER(at, NULL, "AT commands", handle_at_cmd);

SHELL_STATIC_SUBCMD_SET_CREATE(gps_cmds,
	SHELL_CMD(start, NULL, "Start GPS search", start_gps),
	SHELL_CMD(stop, NULL, "Stop GPS search", stop_gps),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(gps, &gps_cmds, "GPS handling", NULL);

SHELL_STATIC_SUBCMD_SET_CREATE(lte_psm_cmds,
	SHELL_CMD(enable, NULL, "Enable PSM", lte_psm_enable),
	SHELL_CMD(disable, NULL, "Disable PSM", lte_psm_disable),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(lte_edrx_cmds,
	SHELL_CMD(enable, NULL, "Enable eDRX", lte_edrx_enable),
	SHELL_CMD(disable, NULL, "Disable eDRX", lte_edrx_disable),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(lte_cmds,
	SHELL_CMD(enable, NULL, "Enable LTE", lte_enable),
	SHELL_CMD(disable, NULL, "Disable LTE", lte_disable),
	SHELL_CMD(psm, &lte_psm_cmds, "PSM handling", NULL),
	SHELL_CMD(edrx, &lte_edrx_cmds, "eDRX handling", NULL),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(lte, &lte_cmds, "LTE handling", NULL);

void modem_trace_enable(void)
{
    /* GPIO configurations for trace and debug */
    #define CS_PIN_CFG_TRACE_CLK    21 //GPIO_OUT_PIN21_Pos
    #define CS_PIN_CFG_TRACE_DATA0  22 //GPIO_OUT_PIN22_Pos
    #define CS_PIN_CFG_TRACE_DATA1  23 //GPIO_OUT_PIN23_Pos
    #define CS_PIN_CFG_TRACE_DATA2  24 //GPIO_OUT_PIN24_Pos
    #define CS_PIN_CFG_TRACE_DATA3  25 //GPIO_OUT_PIN25_Pos

    NRF_GPIO_Type *port = NRF_P0_NS;

    // Configure outputs.
    // CS_PIN_CFG_TRACE_CLK
    port->PIN_CNF[CS_PIN_CFG_TRACE_CLK] = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                                (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);

    // CS_PIN_CFG_TRACE_DATA0
    port->PIN_CNF[CS_PIN_CFG_TRACE_DATA0] = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                                  (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);

    // CS_PIN_CFG_TRACE_DATA1
    port->PIN_CNF[CS_PIN_CFG_TRACE_DATA1] = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                                  (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);

    // CS_PIN_CFG_TRACE_DATA2
    port->PIN_CNF[CS_PIN_CFG_TRACE_DATA2] = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                                  (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);

    // CS_PIN_CFG_TRACE_DATA3
    port->PIN_CNF[CS_PIN_CFG_TRACE_DATA3] = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                                  (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);

    port->DIR = 0xFFFFFFFF;

    printk("Traces enabled\n");
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
	case GPS_EVT_NO_TIME_WINDOW:
		printk("GPS_EVT_NO_TIME_WINDOW\n");
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

		stop_gps();
		break;
	default:
		break;
	}
}

void main(void)
{
	int err;

	modem_trace_enable();

	printk("Starting A-GPS application\n");

	gps_dev = device_get_binding("NRF9160_GPS");
	if (gps_dev == NULL) {
		printk("Could not get binding to nRF9160 GPS\n");
		return;
	}

	at_notif_register_handler(NULL, notif_handler);

	err = modem_configure();
	if (err) {
		printk("Error when configuring modem: %d\n", err);
		return;
	}

	err = gps_init(gps_dev, gps_handler);
	if (err) {
		printk("Could not initialize GPS, error: %d\n", err);
		return;
	}

	k_thread_create(&cloud_thread, cloud_thread_stack,
		K_THREAD_STACK_SIZEOF(cloud_thread_stack),
		(k_thread_entry_t)cloud_thread_fn, NULL, NULL, NULL,
		0, 0, 0);

	while (true) {
		k_sleep(K_HOURS(10));
	}
}
