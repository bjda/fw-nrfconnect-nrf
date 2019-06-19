/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <stdio.h>
#include <string.h>
#include <gps.h>
#include <sensor.h>
#include <console.h>
#include <nrf_cloud.h>
#include <misc/reboot.h>
#if defined(CONFIG_BSD_LIBRARY)
#include <net/bsdlib.h>
#include <lte_lc.h>
#include <modem_info.h>
#endif

#include "orientation_detector.h"
#include "ui.h"

#define CALIBRATION_PRESS_DURATION 	K_SECONDS(5)

#if defined(CONFIG_FLIP_POLL)
#define FLIP_POLL_INTERVAL		K_MSEC(CONFIG_FLIP_POLL_INTERVAL)
#else
#define FLIP_POLL_INTERVAL		0
#endif

#ifdef CONFIG_ACCEL_USE_SIM
#define FLIP_INPUT			CONFIG_FLIP_INPUT
#define CALIBRATION_INPUT		-1
#else
#define FLIP_INPUT			-1
#ifdef CONFIG_ACCEL_CALIBRATE
#define CALIBRATION_INPUT              CONFIG_CALIBRATION_INPUT
#else
#define CALIBRATION_INPUT              -1
#endif /* CONFIG_ACCEL_CALIBRATE */
#endif /* CONFIG_ACCEL_USE_SIM */

#define CLOUD_LED_ON_STR		"{\"led\":\"on\"}"
#define CLOUD_LED_OFF_STR		"{\"led\":\"off\"}"
#define CLOUD_LED_NUMBER		UI_LED_1

#define CLOUD_DOOR_CLOSE_STR "{\"door\":\"close\"}"
#define CLOUD_DOOR_OPEN_STR "{\"door\":\"open\"}"
#define CLOUD_DOOR_LOCK_STR "{\"lock\":\"lock\"}"
#define CLOUD_DOOR_UNLOCK_STR "{\"lock\":\"unlock\"}"

/* Constants for servo operation. All time units are us. */
#define PWM_PERIOD 20000
#define DOOR_PWM_NMOS_ID UI_NMOS_1
#define LOCK_PWM_NMOS_ID UI_NMOS_2
#define DOOR_PWR_NMOS_ID UI_NMOS_3
#define LOCK_PWR_NMOS_ID UI_NMOS_4
#define DOOR_PWM_WIDTH_CLOSE 1930
#define DOOR_PWM_WIDTH_OPEN 1020
#define LOCK_PWM_WIDTH_LOCK 1930
#define LOCK_PWM_WIDTH_UNLOCK 1020


#if defined(CONFIG_BSD_LIBRARY) && !defined(CONFIG_LTE_LINK_CONTROL)
#error "Missing CONFIG_LTE_LINK_CONTROL"
#endif

#if defined(CONFIG_BSD_LIBRARY) && \
	defined(CONFIG_LTE_AUTO_INIT_AND_CONNECT) && \
	defined(CONFIG_NRF_CLOUD_PROVISION_CERTIFICATES)
#error "PROVISION_CERTIFICATES \
	requires CONFIG_LTE_AUTO_INIT_AND_CONNECT to be disabled!"
#endif

struct env_sensor {
	enum nrf_cloud_sensor type;
	enum sensor_channel channel;
	u8_t *dev_name;
	struct device *dev;
};

struct rsrp_data {
	u16_t value;
	u16_t offset;
};

#if CONFIG_MODEM_INFO
static struct rsrp_data rsrp = {
	.value = 0,
	.offset = MODEM_INFO_RSRP_OFFSET_VAL,
};
#endif /* CONFIG_MODEM_INFO */
/* Array containing all nRF Cloud sensor types that are available to the
 * application.
 */
static const enum nrf_cloud_sensor available_sensors[] = {
	NRF_CLOUD_SENSOR_GPS,
	NRF_CLOUD_SENSOR_FLIP,
	NRF_CLOUD_SENSOR_BUTTON,
	NRF_CLOUD_SENSOR_TEMP,
	NRF_CLOUD_SENSOR_HUMID,
	NRF_CLOUD_SENSOR_AIR_PRESS,
	NRF_CLOUD_LTE_LINK_RSRP,
	NRF_CLOUD_DEVICE_INFO,
};

static struct env_sensor temp_sensor = {
	.type = NRF_CLOUD_SENSOR_TEMP,
	.channel = SENSOR_CHAN_AMBIENT_TEMP,
	.dev_name = CONFIG_TEMP_DEV_NAME
};

static struct env_sensor humid_sensor = {
	.type = NRF_CLOUD_SENSOR_HUMID,
	.channel = SENSOR_CHAN_HUMIDITY,
	.dev_name = CONFIG_TEMP_DEV_NAME
};

static struct env_sensor pressure_sensor = {
	.type = NRF_CLOUD_SENSOR_AIR_PRESS,
	.channel = SENSOR_CHAN_PRESS,
	.dev_name = CONFIG_TEMP_DEV_NAME
};

/* Array containg environment sensors available on the board. */
static struct env_sensor *env_sensors[] = {
	&temp_sensor,
	&humid_sensor,
	&pressure_sensor
};

 /* Variables to keep track of nRF cloud user association. */
static u8_t ua_pattern[10];
static int buttons_to_capture;
static int buttons_captured;
static bool pattern_recording;
static struct k_sem user_assoc_sem;

/* Sensor data */
static struct gps_data nmea_data;
static struct nrf_cloud_sensor_data flip_cloud_data;
static struct nrf_cloud_sensor_data gps_cloud_data;
static struct nrf_cloud_sensor_data button_cloud_data;
static struct nrf_cloud_sensor_data env_cloud_data[ARRAY_SIZE(env_sensors)];
#if CONFIG_MODEM_INFO
static struct nrf_cloud_sensor_data signal_strength_cloud_data;
static struct nrf_cloud_sensor_data device_cloud_data;
#endif /* CONFIG_MODEM_INFO */
static atomic_val_t send_data_enable;

/* Flag used for flip detection */
static bool flip_mode_enabled = true;

/* Structures for work */
static struct k_work connect_work;
static struct k_work send_gps_data_work;
static struct k_work send_env_data_work;
static struct k_work send_button_data_work;
static struct k_work send_flip_data_work;
static struct k_delayed_work flip_poll_work;
static struct k_delayed_work long_press_button_work;
#if CONFIG_MODEM_INFO
static struct k_work device_status_work;
static struct k_work rsrp_work;
#endif /* CONFIG_MODEM_INFO */

enum error_type {
	ERROR_NRF_CLOUD,
	ERROR_BSD_RECOVERABLE,
	ERROR_BSD_IRRECOVERABLE,
	ERROR_LTE_LC
};

/* Forward declaration of functions */
static void cloud_connect(struct k_work *work);
static void flip_send(struct k_work *work);
static void env_data_send(void);
static void sensors_init(void);
static void work_init(void);
static void sensor_data_send(struct nrf_cloud_sensor_data *data);

/**@brief nRF Cloud error handler. */
void error_handler(enum error_type err_type, int err_code)
{
	if (err_type == ERROR_NRF_CLOUD) {
#if defined(CONFIG_LTE_LINK_CONTROL)
		/* Turn off and shutdown modem */
		int err = lte_lc_power_off();
		if (err) {
			printk("lte_lc_power_off failed: %d\n", err);
		}
#endif
#if defined(CONFIG_BSD_LIBRARY)
		bsdlib_shutdown();
#endif
	}

#if !defined(CONFIG_DEBUG)
	sys_reboot(SYS_REBOOT_COLD);
#else
	switch (err_type) {
	case ERROR_NRF_CLOUD:
		/* Blinking all LEDs ON/OFF in pairs (1 and 4, 2 and 3)
		 * if there is an application error.
		 */
		ui_led_set_pattern(UI_LED_ERROR_CLOUD);
		printk("Error of type ERROR_NRF_CLOUD: %d\n", err_code);
	break;
	case ERROR_BSD_RECOVERABLE:
		/* Blinking all LEDs ON/OFF in pairs (1 and 3, 2 and 4)
		 * if there is a recoverable error.
		 */
		ui_led_set_pattern(UI_LED_ERROR_BSD_REC);
		printk("Error of type ERROR_BSD_RECOVERABLE: %d\n", err_code);
	break;
	case ERROR_BSD_IRRECOVERABLE:
		/* Blinking all LEDs ON/OFF if there is an
		 * irrecoverable error.
		 */
		ui_led_set_pattern(UI_LED_ERROR_BSD_IRREC);
		printk("Error of type ERROR_BSD_IRRECOVERABLE: %d\n", err_code);
	break;
	default:
		/* Blinking all LEDs ON/OFF in pairs (1 and 2, 3 and 4)
		 * undefined error.
		 */
		ui_led_set_pattern(UI_LED_ERROR_UNKNOWN);
		printk("Unknown error type: %d, code: %d\n",
			err_type, err_code);
	break;
	}

	while (true) {
		k_cpu_idle();
	}
#endif /* CONFIG_DEBUG */
}

void nrf_cloud_error_handler(int err)
{
	error_handler(ERROR_NRF_CLOUD, err);
}

/**@brief Recoverable BSD library error. */
void bsd_recoverable_error_handler(uint32_t err)
{
	error_handler(ERROR_BSD_RECOVERABLE, (int)err);
}

/**@brief Irrecoverable BSD library error. */
void bsd_irrecoverable_error_handler(uint32_t err)
{
	error_handler(ERROR_BSD_IRRECOVERABLE, (int)err);
}

static void send_gps_data_work_fn(struct k_work *work)
{
	sensor_data_send(&gps_cloud_data);
	env_data_send();
}

static void send_env_data_work_fn(struct k_work *work)
{
	env_data_send();
}

static void send_button_data_work_fn(struct k_work *work)
{
	sensor_data_send(&button_cloud_data);
}

static void send_flip_data_work_fn(struct k_work *work)
{
	sensor_data_send(&flip_cloud_data);
}

/**@brief Callback for GPS trigger events */
static void gps_trigger_handler(struct device *dev, struct gps_trigger *trigger)
{
	ARG_UNUSED(trigger);

	if (ui_button_is_active(UI_SWITCH_2)
	   || !atomic_get(&send_data_enable)) {
		return;
	}

	gps_sample_fetch(dev);
	gps_channel_get(dev, GPS_CHAN_NMEA, &nmea_data);
	gps_cloud_data.data.ptr = nmea_data.str;
	gps_cloud_data.data.len = nmea_data.len;
	gps_cloud_data.tag += 1;

	if (gps_cloud_data.tag == 0) {
		gps_cloud_data.tag = 0x1;
	}

	k_work_submit(&send_gps_data_work);
}

/**@brief Callback for sensor trigger events */
static void sensor_trigger_handler(struct device *dev,
			struct sensor_trigger *trigger)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(trigger);

	flip_send(NULL);
}

/**@brief Send button presses to cloud */
static void button_send(bool pressed)
{
	static char data[] = "1";

	if (!atomic_get(&send_data_enable)) {
		return;
	}

	data[0] = pressed ? '1' : '0';

	button_cloud_data.data.ptr = &data;
	button_cloud_data.data.len = strlen(data);
	button_cloud_data.tag += 1;

	if (button_cloud_data.tag == 0) {
		button_cloud_data.tag = 0x1;
	}

	k_work_submit(&send_button_data_work);
}

/**@brief Poll flip orientation and send to cloud if flip mode is enabled. */
static void flip_send(struct k_work *work)
{
	static enum orientation_state last_orientation_state =
		ORIENTATION_NOT_KNOWN;
	static struct orientation_detector_sensor_data sensor_data;

	if (!flip_mode_enabled || !atomic_get(&send_data_enable)) {
		goto exit;
	}

	if (orientation_detector_poll(&sensor_data) == 0) {
		if (sensor_data.orientation == last_orientation_state) {
			goto exit;
		}

		switch (sensor_data.orientation) {
		case ORIENTATION_NORMAL:
			flip_cloud_data.data.ptr = "NORMAL";
			flip_cloud_data.data.len = sizeof("NORMAL") - 1;
			break;
		case ORIENTATION_UPSIDE_DOWN:
			flip_cloud_data.data.ptr = "UPSIDE_DOWN";
			flip_cloud_data.data.len = sizeof("UPSIDE_DOWN") - 1;
			break;
		default:
			goto exit;
		}

		last_orientation_state = sensor_data.orientation;

		k_work_submit(&send_flip_data_work);
	}

exit:
	if (work) {
		k_delayed_work_submit(&flip_poll_work,
					FLIP_POLL_INTERVAL);
	}
}

#if CONFIG_MODEM_INFO
/**@brief Callback handler for LTE RSRP data. */
static void modem_rsrp_handler(char rsrp_value)
{
	rsrp.value = rsrp_value;

	k_work_submit(&rsrp_work);
}

/**@brief Publish RSRP data to the cloud. */
static void modem_rsrp_data_send(struct k_work *work)
{
	char buf[CONFIG_MODEM_INFO_BUFFER_SIZE] = {0};
	static u32_t timestamp_prev;
	size_t len;

	if (!atomic_get(&send_data_enable)) {
		return;
	}

	if (k_uptime_get_32() - timestamp_prev <
	    K_SECONDS(CONFIG_HOLD_TIME_RSRP)) {
		return;
	}

	len = snprintf(buf, CONFIG_MODEM_INFO_BUFFER_SIZE,
			"%d", rsrp.value - rsrp.offset);

	signal_strength_cloud_data.data.ptr = buf;
	signal_strength_cloud_data.data.len = len;
	signal_strength_cloud_data.tag += 1;

	if (signal_strength_cloud_data.tag == 0) {
		signal_strength_cloud_data.tag = 0x1;
	}

	sensor_data_send(&signal_strength_cloud_data);
	timestamp_prev = k_uptime_get_32();
}

/**@brief Poll device info and send data to the cloud. */
static void device_status_send(struct k_work *work)
{
	int len;
	char data_buffer[MODEM_INFO_JSON_STRING_SIZE] = {0};

	if (!atomic_get(&send_data_enable)) {
		return;
	}

	len = modem_info_json_string_get(data_buffer);
	if (len < 0) {
		return;
	}

	device_cloud_data.data.ptr = data_buffer;
	device_cloud_data.data.len = len;
	device_cloud_data.tag += 1;

	if (device_cloud_data.tag == 0) {
		device_cloud_data.tag = 0x1;
	}

	sensor_data_send(&device_cloud_data);
}
#endif /* CONFIG_MODEM_INFO */

/**@brief Get environment data from sensors and send to cloud. */
static void env_data_send(void)
{
	int num_sensors = ARRAY_SIZE(env_sensors);
	struct sensor_value data[num_sensors];
	char buf[6];
	int err;
	u8_t len;

	if (!atomic_get(&send_data_enable)) {
		return;
	}

	for (int i = 0; i < num_sensors; i++) {
		err = sensor_sample_fetch_chan(env_sensors[i]->dev,
			env_sensors[i]->channel);
		if (err) {
			printk("Failed to fetch data from %s, error: %d\n",
				env_sensors[i]->dev_name, err);
			return;
		}

		err = sensor_channel_get(env_sensors[i]->dev,
			env_sensors[i]->channel, &data[i]);
		if (err) {
			printk("Failed to fetch data from %s, error: %d\n",
				env_sensors[i]->dev_name, err);
			return;
		}

		len = snprintf(buf, sizeof(buf), "%.1f",
			sensor_value_to_double(&data[i]));
		env_cloud_data[i].data.ptr = buf;
		env_cloud_data[i].data.len = len;
		env_cloud_data[i].tag += 1;

		if (env_cloud_data[i].tag == 0) {
			env_cloud_data[i].tag = 0x1;
		}

		sensor_data_send(&env_cloud_data[i]);
	}
}

/**@brief Send sensor data to nRF Cloud. **/
static void sensor_data_send(struct nrf_cloud_sensor_data *data)
{
	int err;

	if (pattern_recording || !atomic_get(&send_data_enable)) {
		return;
	}

	if (data->type == NRF_CLOUD_SENSOR_GPS) {
		err = nrf_cloud_sensor_data_send(data);
	} else {
		err = nrf_cloud_sensor_data_stream(data);
	}

	if (err) {
		printk("sensor_data_send failed: %d\n", err);
		nrf_cloud_error_handler(err);
	}
}

/**@brief Callback for user association event received from nRF Cloud. */
static void on_user_association_req(const struct nrf_cloud_evt *p_evt)
{
	if (!pattern_recording) {
		k_sem_init(&user_assoc_sem, 0, 1);
		ui_led_set_pattern(UI_ACCEL_CALIBRATING);
		pattern_recording = true;
		buttons_captured = 0;
		buttons_to_capture = p_evt->param.ua_req.sequence.len;

		printk("Please enter the user association pattern ");

		if (IS_ENABLED(CONFIG_CLOUD_UA_BUTTONS)) {
			printk("using the buttons and switches\n");
		} else if (IS_ENABLED(CONFIG_CLOUD_UA_CONSOLE)) {
			printk("using the console\n");
			console_init();
		}
	}
}

/**@brief Callback for data received event from nRF Cloud. */
static void on_data_received(const struct nrf_cloud_evt *p_evt)
{
#ifdef CONFIG_UI_LED_USE_PWM
	static enum ui_led_pattern prev_pattern = UI_CLOUD_CONNECTED;
#endif

	if (memcmp(p_evt->param.data.ptr, CLOUD_LED_ON_STR,
	    strlen(CLOUD_LED_ON_STR)) == 0) {
#ifdef CONFIG_UI_LED_USE_PWM
		prev_pattern = ui_led_get_pattern();
		ui_led_set_color(50, 50, 50);
#else
		ui_led_set_state(CLOUD_LED_NUMBER, 1);
#endif /* CONFIG_UI_LED_USE_PWM */
	} else if (memcmp(p_evt->param.data.ptr, CLOUD_LED_OFF_STR,
		   strlen(CLOUD_LED_OFF_STR)) == 0) {
#ifdef CONFIG_UI_LED_USE_PWM
		ui_led_set_pattern(prev_pattern);
#else
		ui_led_set_state(CLOUD_LED_NUMBER, 0);
#endif /* CONFIG_UI_LED_USE_PWM */
	} else if (memcmp(p_evt->param.data.ptr, CLOUD_DOOR_CLOSE_STR,
		strlen(CLOUD_DOOR_CLOSE_STR)) == 0) {
		ui_nmos_pwm_set(DOOR_PWM_NMOS_ID, PWM_PERIOD, 
						PWM_PERIOD - DOOR_PWM_WIDTH_CLOSE);
	} else if (memcmp(p_evt->param.data.ptr, CLOUD_DOOR_OPEN_STR,
		strlen(CLOUD_DOOR_OPEN_STR)) == 0) {
		ui_nmos_pwm_set(DOOR_PWM_NMOS_ID, PWM_PERIOD, 
						PWM_PERIOD - DOOR_PWM_WIDTH_OPEN);
	} else if (memcmp(p_evt->param.data.ptr, CLOUD_DOOR_LOCK_STR,
		strlen(CLOUD_DOOR_LOCK_STR)) == 0) {
		ui_nmos_pwm_set(LOCK_PWM_NMOS_ID, PWM_PERIOD, 
						PWM_PERIOD - LOCK_PWM_WIDTH_LOCK);
	} else if (memcmp(p_evt->param.data.ptr, CLOUD_DOOR_UNLOCK_STR,
		strlen(CLOUD_DOOR_UNLOCK_STR)) == 0) {
		ui_nmos_pwm_set(LOCK_PWM_NMOS_ID, PWM_PERIOD, 
						PWM_PERIOD - LOCK_PWM_WIDTH_UNLOCK);
	} else {
		printk("Data not recognised\n");
	}
}

/**@brief Attach available sensors to nRF Cloud. */
void sensors_attach(void)
{
	int err;
	struct nrf_cloud_sa_param param;

	for (u8_t i = 0; i < ARRAY_SIZE(available_sensors); i++) {
		param.sensor_type = available_sensors[i];

		err = nrf_cloud_sensor_attach(&param);
		if (err) {
			printk("nrf_cloud_sensor_attach failed: %d\n",
				err);
			nrf_cloud_error_handler(err);
		}
	}
}

/**@brief Callback for sensor attached event from nRF Cloud. */
void sensor_attached(void)
{
	static u8_t attached_sensors;
	attached_sensors++;

	if (attached_sensors == ARRAY_SIZE(available_sensors)) {
		atomic_set(&send_data_enable, 1);
		sensors_init();

		if (IS_ENABLED(CONFIG_FLIP_POLL)) {
			k_delayed_work_submit(&flip_poll_work, K_NO_WAIT);
		}
	}
}

/**@brief Callback for nRF Cloud events. */
static void cloud_event_handler(const struct nrf_cloud_evt *p_evt)
{
	switch (p_evt->type) {
	case NRF_CLOUD_EVT_TRANSPORT_CONNECTED:
		printk("NRF_CLOUD_EVT_TRANSPORT_CONNECTED\n");
		break;
	case NRF_CLOUD_EVT_USER_ASSOCIATION_REQUEST:
		printk("NRF_CLOUD_EVT_USER_ASSOCIATION_REQUEST\n");
		on_user_association_req(p_evt);
		break;
	case NRF_CLOUD_EVT_USER_ASSOCIATED:
		printk("NRF_CLOUD_EVT_USER_ASSOCIATED\n");
		break;
	case NRF_CLOUD_EVT_READY:
		printk("NRF_CLOUD_EVT_READY\n");
		ui_led_set_pattern(UI_CLOUD_CONNECTED);
		sensors_attach();
		break;
	case NRF_CLOUD_EVT_SENSOR_ATTACHED:
		printk("NRF_CLOUD_EVT_SENSOR_ATTACHED\n");
		sensor_attached();
		break;
	case NRF_CLOUD_EVT_SENSOR_DATA_ACK:
		printk("NRF_CLOUD_EVT_SENSOR_DATA_ACK\n");
		break;
	case NRF_CLOUD_EVT_TRANSPORT_DISCONNECTED:
		printk("NRF_CLOUD_EVT_TRANSPORT_DISCONNECTED\n");
		atomic_set(&send_data_enable, 0);
		ui_led_set_pattern(UI_LTE_DISCONNECTED);

		/* Reconnect to nRF Cloud. */
		k_work_submit(&connect_work);
		break;
	case NRF_CLOUD_EVT_ERROR:
		printk("NRF_CLOUD_EVT_ERROR, status: %d\n", p_evt->status);
		atomic_set(&send_data_enable, 0);
		nrf_cloud_error_handler(p_evt->status);
		break;
	case NRF_CLOUD_EVT_RX_DATA:
		printk("NRF_CLOUD_EVT_RX_DATA\n");
		on_data_received(p_evt);
		break;
	default:
		printk("Received unknown %d\n", p_evt->type);
		break;
	}
}

/**@brief Initialize nRF CLoud library. */
static void cloud_init(void)
{
	const struct nrf_cloud_init_param param = {
		.event_handler = cloud_event_handler
	};

	int err = nrf_cloud_init(&param);

	__ASSERT(err == 0, "nRF Cloud library could not be initialized.");
}

/**@brief Connect to nRF Cloud, */
static void cloud_connect(struct k_work *work)
{
	int err;

	ARG_UNUSED(work);

	const enum nrf_cloud_ua supported_uas[] = {
		NRF_CLOUD_UA_BUTTON
	};

	const struct nrf_cloud_ua_list ua_list = {
		.size = ARRAY_SIZE(supported_uas),
		.ptr = supported_uas
	};

	const struct nrf_cloud_sensor_list sensor_list = {
		.size = ARRAY_SIZE(available_sensors),
		.ptr = available_sensors
	};

	const struct nrf_cloud_connect_param param = {
		.ua = &ua_list,
		.sensor = &sensor_list,
	};

	ui_led_set_pattern(UI_CLOUD_CONNECTING);
	err = nrf_cloud_connect(&param);

	if (err) {
		printk("nrf_cloud_connect failed: %d\n", err);
		nrf_cloud_error_handler(err);
	}
}

/**@brief Send user association information to nRF Cloud. */
static void cloud_user_associate(void)
{
	int err;
	const struct nrf_cloud_ua_param ua = {
		.type = NRF_CLOUD_UA_BUTTON,
		.sequence = {
			.len = buttons_to_capture,
			.ptr = ua_pattern
		}
	};

	err = nrf_cloud_user_associate(&ua);

	if (err) {
		printk("nrf_cloud_user_associate failed: %d\n", err);
		nrf_cloud_error_handler(err);
	}
}

static void accelerometer_calibrate(struct k_work *work)
{
	int err;
	enum ui_led_pattern temp_led_state = ui_led_get_pattern();

	printk("Starting accelerometer calibration...\n");

	ui_led_set_pattern(UI_ACCEL_CALIBRATING);

	err = orientation_detector_calibrate();
	if (err) {
		printk("Accelerometer calibration failed: %d\n",
			err);
	} else {
		printk("Accelerometer calibration done.\n");
	}

	ui_led_set_pattern(temp_led_state);
}

/**@brief Processing of user inputs to the application. */
static void input_process(void)
{
	if (!pattern_recording) {
		return;
	}

	if (k_sem_take(&user_assoc_sem, K_NO_WAIT) == 0) {
		cloud_user_associate();
		pattern_recording = false;
		return;
	}

	if (!IS_ENABLED(CONFIG_CLOUD_UA_CONSOLE)) {
		return;
	}

	u8_t c[2];

	c[0] = console_getchar();
	c[1] = console_getchar();

	if (c[0] == 'b' && c[1] == '1') {
		printk("Button 1\n");
		ua_pattern[buttons_captured++] = NRF_CLOUD_UA_BUTTON_INPUT_3;
	} else if (c[0] == 'b' && c[1] == '2') {
		printk("Button 2\n");
		ua_pattern[buttons_captured++] = NRF_CLOUD_UA_BUTTON_INPUT_4;
	} else if (c[0] == 's' && c[1] == '1') {
		printk("Switch 1\n");
		ua_pattern[buttons_captured++] = NRF_CLOUD_UA_BUTTON_INPUT_1;
	} else if (c[0] == 's' && c[1] == '2') {
		printk("Switch 2\n");
		ua_pattern[buttons_captured++] = NRF_CLOUD_UA_BUTTON_INPUT_2;
	}

	if (buttons_captured == buttons_to_capture) {
		k_sem_give(&user_assoc_sem);
	}
}

/**@brief Initializes and submits delayed work. */
static void work_init(void)
{
	k_work_init(&connect_work, cloud_connect);
	k_work_init(&send_gps_data_work, send_gps_data_work_fn);
	k_work_init(&send_env_data_work, send_env_data_work_fn);
	k_work_init(&send_button_data_work, send_button_data_work_fn);
	k_work_init(&send_flip_data_work, send_flip_data_work_fn);
	k_delayed_work_init(&flip_poll_work, flip_send);
	k_delayed_work_init(&long_press_button_work, accelerometer_calibrate);
#if CONFIG_MODEM_INFO
	k_work_init(&device_status_work, device_status_send);
	k_work_init(&rsrp_work, modem_rsrp_data_send);
#endif /* CONFIG_MODEM_INFO */
}

/**@brief Configures modem to provide LTE link. Blocks until link is
 * successfully established.
 */
static void modem_configure(void)
{
#if defined(CONFIG_BSD_LIBRARY)
	if (IS_ENABLED(CONFIG_LTE_AUTO_INIT_AND_CONNECT)) {
		/* Do nothing, modem is already turned on
		 * and connected.
		 */
	} else {
		int err;

		printk("Connecting to LTE network. ");
		printk("This may take several minutes.\n");
		ui_led_set_pattern(UI_LTE_CONNECTING);
		err = lte_lc_init_and_connect();
		__ASSERT(err == 0, "LTE link could not be established.");

		printk("Connected to LTE network\n");
		ui_led_set_pattern(UI_LTE_CONNECTED);
	}

	ui_led_set_pattern(UI_LTE_CONNECTED);
#endif
}

/**@brief Initializes the accelerometer device and
 * configures trigger if set.
 */
static void accelerometer_init(void)
{
	if (!IS_ENABLED(CONFIG_FLIP_POLL) &&
	     IS_ENABLED(CONFIG_ACCEL_USE_EXTERNAL)) {

		struct device *accel_dev =
		device_get_binding(CONFIG_ACCEL_DEV_NAME);

		if (accel_dev == NULL) {
			printk("Could not get %s device\n",
				CONFIG_ACCEL_DEV_NAME);
			return;
		}

		struct sensor_trigger sensor_trig = {
			.type = SENSOR_TRIG_THRESHOLD,
		};

		printk("Setting trigger\n");
		int err = 0;

		err = sensor_trigger_set(accel_dev, &sensor_trig,
				sensor_trigger_handler);

		if (err) {
			printk("Unable to set trigger\n");
		}
	}
}

/**@brief Initializes GPS device and configures trigger if set.
 * Gets initial sample from GPS device.
 */
static void gps_init(void)
{
	int err;
	struct device *gps_dev = device_get_binding(CONFIG_GPS_DEV_NAME);
	struct gps_trigger gps_trig = {
		.type = GPS_TRIG_DATA_READY,
	};

	if (gps_dev == NULL) {
		printk("Could not get %s device\n", CONFIG_GPS_DEV_NAME);
		return;
	}
	printk("GPS device found\n");

	if (IS_ENABLED(CONFIG_GPS_TRIGGER)) {
		err = gps_trigger_set(gps_dev, &gps_trig,
				gps_trigger_handler);

		if (err) {
			printk("Could not set trigger, error code: %d\n", err);
			return;
		}
	}

	err = gps_sample_fetch(gps_dev);
	__ASSERT(err == 0, "GPS sample could not be fetched.");

	err = gps_channel_get(gps_dev, GPS_CHAN_NMEA, &nmea_data);
	__ASSERT(err == 0, "GPS sample could not be retrieved.");
}

/**@brief Initializes flip detection using orientation detector module
 * and configured accelerometer device.
 */
static void flip_detection_init(void)
{
	int err;
	struct device *accel_dev =
		device_get_binding(CONFIG_ACCEL_DEV_NAME);

	if (accel_dev == NULL) {
		printk("Could not get %s device\n", CONFIG_ACCEL_DEV_NAME);
		return;
	}

	orientation_detector_init(accel_dev);

	if (!IS_ENABLED(CONFIG_ACCEL_CALIBRATE)) {
		return;
	}

	err = orientation_detector_calibrate();
	if (err) {
		printk("Could not calibrate accelerometer device: %d\n", err);
	}
}

/**@brief Initialize environment sensors. */
static void env_sensor_init(void)
{
	for (int i = 0; i < ARRAY_SIZE(env_sensors); i++) {
		env_sensors[i]->dev =
			device_get_binding(env_sensors[i]->dev_name);
		__ASSERT(env_sensors[i]->dev, "Could not get device %s\n",
			env_sensors[i]->dev_name);

		env_cloud_data[i].type = env_sensors[i]->type;
		env_cloud_data[i].tag = 0x1;
	}
}

static void button_sensor_init(void)
{
	button_cloud_data.type = NRF_CLOUD_SENSOR_BUTTON;
	button_cloud_data.tag = 0x1;
}

#if CONFIG_MODEM_INFO
/**brief Initialize LTE status containers. */
static void modem_data_init(void)
{
	int err;
	err = modem_info_init();
	if (err) {
		printk("Modem info could not be established: %d\n", err);
		return;
	}

	signal_strength_cloud_data.type = NRF_CLOUD_LTE_LINK_RSRP;
	signal_strength_cloud_data.tag = 0x1;

	device_cloud_data.type = NRF_CLOUD_DEVICE_INFO;
	device_cloud_data.tag = 0x1;

	k_work_submit(&device_status_work);

	modem_info_rsrp_register(modem_rsrp_handler);
}
#endif /* CONFIG_MODEM_INFO */

/**@brief Initializes the sensors that are used by the application. */
static void sensors_init(void)
{
	accelerometer_init();
	gps_init();
	flip_detection_init();
	env_sensor_init();
#if CONFIG_MODEM_INFO
	modem_data_init();
#endif /* CONFIG_MODEM_INFO */
	if (IS_ENABLED(CONFIG_CLOUD_BUTTON)) {
		button_sensor_init();
	}

	gps_cloud_data.type = NRF_CLOUD_SENSOR_GPS;
	gps_cloud_data.tag = 0x1;
	gps_cloud_data.data.ptr = nmea_data.str;
	gps_cloud_data.data.len = nmea_data.len;

	flip_cloud_data.type = NRF_CLOUD_SENSOR_FLIP;

	/* Send sensor data after initialization, as it may be a long time until
	 * next time if the application is in power optimized mode.
	 */
	k_work_submit(&send_gps_data_work);
	env_data_send();
}

void button_handler(struct ui_evt evt)
{
	if (IS_ENABLED(CONFIG_CLOUD_BUTTON) &&
	   (evt.button == CONFIG_CLOUD_BUTTON_INPUT)) {
		button_send(evt.type == UI_EVT_BUTTON_ACTIVE ? 1 : 0);
	}

	if (IS_ENABLED(CONFIG_ACCEL_USE_SIM) && (evt.button == FLIP_INPUT)) {
		flip_send(NULL);
	}

#if defined(CONFIG_LTE_LINK_CONTROL)
	if ((evt.button == UI_SWITCH_2) &&
	    IS_ENABLED(CONFIG_POWER_OPTIMIZATION_ENABLE)) {
		int err;

		if (evt.type == UI_EVT_BUTTON_ACTIVE) {
			err = lte_lc_edrx_req(false);
			if (err) {
				error_handler(ERROR_LTE_LC, err);
			}
			err = lte_lc_psm_req(true);
			if (err) {
				error_handler(ERROR_LTE_LC, err);
			}
		} else {
			err = lte_lc_psm_req(false);
			if (err) {
				error_handler(ERROR_LTE_LC, err);
			}
			err = lte_lc_edrx_req(true);
			if (err) {
				error_handler(ERROR_LTE_LC, err);
			}
		}
	}
#endif /* defined(CONFIG_LTE_LINK_CONTROL) */
}

void main(void)
{
	k_sleep(1000);
	printk("Application started\n");
	ui_init(button_handler);
	
	/* Servo initialization. cycle value is low time */
	ui_nmos_pwm_set(DOOR_PWM_NMOS_ID, PWM_PERIOD, 
						PWM_PERIOD - DOOR_PWM_WIDTH_CLOSE);
	ui_nmos_pwm_set(LOCK_PWM_NMOS_ID, PWM_PERIOD, 
						PWM_PERIOD - LOCK_PWM_WIDTH_LOCK);
    k_usleep(2 * PWM_PERIOD); /* wait for stable PWM before powering servos */
	ui_nmos_write(DOOR_PWR_NMOS_ID, 1);
	ui_nmos_write(LOCK_PWR_NMOS_ID, 1);

	work_init();
	cloud_init();
	modem_configure();
	cloud_connect(NULL);

	while (true) {
		nrf_cloud_process();
		input_process();
		k_sleep(K_MSEC(10));
		/* Put CPU to idle to save power */
		k_cpu_idle();
	}
}
