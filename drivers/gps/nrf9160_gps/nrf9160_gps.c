/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <drivers/gpio.h>
#include <drivers/gps.h>
#include <init.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <logging/log.h>
#include <nrf_socket.h>
#include <net/socket.h>
#ifdef CONFIG_NRF9160_GPS_HANDLE_MODEM_CONFIGURATION
#include <at_cmd.h>
#include <at_cmd_parser/at_cmd_parser.h>
#include <lte_lc.h>
#endif

LOG_MODULE_REGISTER(nrf9160_gps, CONFIG_NRF9160_GPS_LOG_LEVEL);

#ifdef CONFIG_NRF9160_GPS_HANDLE_MODEM_CONFIGURATION
#define AT_CMD_LEN(x)			(sizeof(x) - 1)
#define AT_XSYSTEMMODE_REQUEST		"AT%XSYSTEMMODE?"
#define AT_XSYSTEMMODE_RESPONSE		"%XSYSTEMMODE:"
#define AT_XSYSTEMMODE_PROTO		"AT%%XSYSTEMMODE=%d,%d,%d,%d"
#define AT_XSYSTEMMODE_PARAMS_COUNT	5
#define AT_XSYSTEMMODE_GPS_PARAM_INDEX	3
#define AT_CFUN_REQUEST			"AT+CFUN?"
#define AT_CFUN_RESPONSE		"+CFUN:"
#define AT_CFUN_0			"AT+CFUN=0"
#define AT_CFUN_1			"AT+CFUN=1"
#define FUNCTIONAL_MODE_ENABLED		1
#endif

/* Aligned strings describing sattelite states based on flags */
#define sv_used_str(x) ((x)?"    used":"not used")
#define sv_unhealthy_str(x) ((x)?"not healthy":"    healthy")

struct gps_drv_data {
	gps_event_handler_t handler;
	atomic_t gps_is_active;
	int socket;
	K_THREAD_STACK_MEMBER(thread_stack,
			      CONFIG_NRF9160_GPS_THREAD_STACK_SIZE);
	struct k_thread thread;
	k_tid_t thread_id;
	struct k_sem thread_run_sem;
};

static u64_t fix_timestamp;

static nrf_gnss_agps_data_type_t type_lookup_gps2socket[] = {
	[GPS_AGPS_UTC_PARAMETERS]	= NRF_GNSS_AGPS_UTC_PARAMETERS,
	[GPS_AGPS_EPHEMERIDES]		= NRF_GNSS_AGPS_EPHEMERIDES,
	[GPS_AGPS_ALMANAC]		= NRF_GNSS_AGPS_ALMANAC,
	[GPS_AGPS_KLOBUCHAR_CORRECTION]
		= NRF_GNSS_AGPS_KLOBUCHAR_IONOSPHERIC_CORRECTION,
	[GPS_AGPS_NEQUICK_CORRECTION]
		= NRF_GNSS_AGPS_NEQUICK_IONOSPHERIC_CORRECTION,
	[GPS_AGPS_GPS_SYSTEM_CLOCK_AND_TOWS]
		= NRF_GNSS_AGPS_GPS_SYSTEM_CLOCK_AND_TOWS,
	[GPS_AGPS_LOCATION]		= NRF_GNSS_AGPS_LOCATION,
	[GPS_AGPS_INTEGRITY]		= NRF_GNSS_AGPS_INTEGRITY,
};

static void copy_pvt(struct gps_pvt *dest, nrf_gnss_pvt_data_frame_t *src)
{
	dest->latitude = src->latitude;
	dest->longitude = src->longitude;
	dest->altitude = src->altitude;
	dest->accuracy = src->accuracy;
	dest->speed = src->speed;
	dest->heading = src->heading;
	dest->datetime.year = src->datetime.year;
	dest->datetime.month = src->datetime.month;
	dest->datetime.day = src->datetime.day;
	dest->datetime.hour = src->datetime.hour;
	dest->datetime.minute = src->datetime.minute;
	dest->datetime.seconds = src->datetime.seconds;
	dest->datetime.ms = src->datetime.ms;
	dest->pdop = src->pdop;
	dest->hdop = src->hdop;
	dest->vdop = src->vdop;
	dest->tdop = src->tdop;

	for (size_t i = 0;
	     i < MIN(NRF_GNSS_MAX_SATELLITES, GPS_PVT_MAX_SV_COUNT); i++) {
		dest->sv[i].sv = src->sv[i].sv;
		dest->sv[i].cn0 = src->sv[i].cn0;
		dest->sv[i].elevation = src->sv[i].elevation;
		dest->sv[i].azimuth = src->sv[i].azimuth;
		dest->sv[i].signal = src->sv[i].signal;
	}
}

static bool is_fix(nrf_gnss_pvt_data_frame_t *pvt)
{
	return ((pvt->flags & NRF_GNSS_PVT_FLAG_FIX_VALID_BIT)
		== NRF_GNSS_PVT_FLAG_FIX_VALID_BIT);
}

/**@brief Checks if GPS operation is blocked due to insufficient time windows */
static bool gps_is_blocked(nrf_gnss_pvt_data_frame_t *pvt)
{
/* TODO: Use bitmask from nrf_socket.h when it's available. */
#define PVT_FLAG_OP_BLOCKED_TIME_WINDOW BIT(4)

	return ((pvt->flags & PVT_FLAG_OP_BLOCKED_TIME_WINDOW)
		== PVT_FLAG_OP_BLOCKED_TIME_WINDOW);
}

/**@brief Checks if PVT frame is invalid due to missed processing deadline */
static bool pvt_deadline_missed(nrf_gnss_pvt_data_frame_t *pvt)
{
/* TODO: Use bitmask from nrf_socket.h when it's available. */
#define PVT_FLAG_DEADLINE_MISSED BIT(3)

	return ((pvt->flags & PVT_FLAG_DEADLINE_MISSED)
		== PVT_FLAG_DEADLINE_MISSED);
}

static void print_satellite_stats(nrf_gnss_data_frame_t *pvt_data)
{
	u8_t  n_tracked = 0;
	u8_t  n_used = 0;
	u8_t  n_unhealthy = 0;

	for (int i = 0; i < NRF_GNSS_MAX_SATELLITES; ++i) {
		u8_t sv = pvt_data->pvt.sv[i].sv;
		bool used = (pvt_data->pvt.sv[i].flags &
			     NRF_GNSS_SV_FLAG_USED_IN_FIX) ? true : false;
		bool unhealthy = (pvt_data->pvt.sv[i].flags &
				  NRF_GNSS_SV_FLAG_UNHEALTHY) ? true : false;

		if (sv) { /* SV number 0 indicates no satellite */
			n_tracked++;
			if (used) {
				n_used++;
			}
			if (unhealthy) {
				n_unhealthy++;
			}

			LOG_DBG("Tracking SV %2u: %s, %s", sv,
				sv_used_str(used),
				sv_unhealthy_str(unhealthy));
		}
	}

	LOG_DBG("Tracking: %d Using: %d Unhealthy: %d", n_tracked,
							n_used,
							n_unhealthy);
	LOG_DBG("Seconds since last fix %lld",
			(k_uptime_get() - fix_timestamp) / 1000);
}

static void notify_event(struct device *dev, struct gps_event *evt)
{
	struct gps_drv_data *drv_data = dev->driver_data;

	if (atomic_get(&drv_data->gps_is_active) && drv_data->handler) {
		drv_data->handler(dev, evt);
	}
}

static void gps_thread(int dev_ptr)
{
	struct device *dev = INT_TO_POINTER(dev_ptr);
	struct gps_drv_data *drv_data = dev->driver_data;
	int len;
	bool operation_blocked = false;
	bool has_fix = false;

wait:
	k_sem_take(&drv_data->thread_run_sem, K_FOREVER);

	while (true) {
		nrf_gnss_data_frame_t raw_gps_data;
		struct gps_event evt = {0};

		len = recv(drv_data->socket, &raw_gps_data,
			   sizeof(nrf_gnss_data_frame_t), 0);
		if (len <= 0) {
			/* Is the GPS stopped, causing this error? */
			if (!atomic_get(&drv_data->gps_is_active)) {
				goto wait;
			}

			LOG_ERR("recv() returned error: %d", len);

			continue;
		}

		switch (raw_gps_data.data_id) {
		case NRF_GNSS_PVT_DATA_ID:
			has_fix = false;

			if (gps_is_blocked(&raw_gps_data.pvt)) {
				if (operation_blocked) {
					/* Avoid spamming the logs and app. */
					continue;
				}

				/* If LTE is used alongside GPS, PSM, eDRX or
				 * DRX needs to be enabled for the GPS to
				 * operate. If PSM is used, the GPS will
				 * normally operate when active time expires.
				 */
				LOG_DBG("Waiting for time window to operate");

				operation_blocked = true;
				evt.type = GPS_EVT_NO_TIME_WINDOW;

				notify_event(dev, &evt);

				continue;
			} else if (operation_blocked) {
				/* GPS has been unblocked. */
				LOG_DBG("GPS has time window to operate");

				operation_blocked = false;
				evt.type = GPS_EVT_HAS_TIME_WINDOW;

				notify_event(dev, &evt);
			}

			if (pvt_deadline_missed(&raw_gps_data.pvt)) {
				LOG_DBG("Invalid PVT frame, discarding");
				continue;
			}

			copy_pvt(&evt.pvt, &raw_gps_data.pvt);

			if (is_fix(&raw_gps_data.pvt)) {
				LOG_DBG("PVT: Position fix");

				evt.type = GPS_EVT_PVT_FIX;
				fix_timestamp = k_uptime_get();
				has_fix = true;
			} else {
				evt.type = GPS_EVT_PVT;
			}

			notify_event(dev, &evt);
			print_satellite_stats(&raw_gps_data);

			break;
		case NRF_GNSS_NMEA_DATA_ID:
			if (operation_blocked) {
				continue;
			}

			memcpy(evt.nmea.buf, raw_gps_data.nmea, len);

			/* Don't count null temrinator. */
			evt.nmea.len = len - 1;

			if (has_fix) {
				LOG_DBG("NMEA: Position fix");

				evt.type = GPS_EVT_NMEA_FIX;
			} else {
				evt.type = GPS_EVT_NMEA;
			}

			notify_event(dev, &evt);
			break;
		case NRF_GNSS_AGPS_DATA_ID:
			LOG_DBG("A-GPS data needed message received");

			evt.type = GPS_EVT_AGPS_DATA_NEEDED;
			evt.agps_request.sv_mask_ephe =
				raw_gps_data.agps.sv_mask_ephe;
			evt.agps_request.sv_mask_alm =
				raw_gps_data.agps.sv_mask_alm;
			evt.agps_request.utc =
				raw_gps_data.agps.data_flags & BIT(0) ? 1 : 0;
			evt.agps_request.klobuchar =
				raw_gps_data.agps.data_flags & BIT(1) ? 1 : 0;
			evt.agps_request.nequick =
				raw_gps_data.agps.data_flags & BIT(2) ? 1 : 0;
			evt.agps_request.system_time_tow =
				raw_gps_data.agps.data_flags & BIT(3) ? 1 : 0;
			evt.agps_request.position =
				raw_gps_data.agps.data_flags & BIT(4) ? 1 : 0;
			evt.agps_request.integrity =
				raw_gps_data.agps.data_flags & BIT(5) ? 1 : 0;

			notify_event(dev, &evt);
			continue;
		default:
			continue;
		}
	}
}

static int init_thread(struct device *dev)
{
	struct gps_drv_data *drv_data = dev->driver_data;

	drv_data->thread_id = k_thread_create(
			&drv_data->thread, drv_data->thread_stack,
			K_THREAD_STACK_SIZEOF(drv_data->thread_stack),
			(k_thread_entry_t)gps_thread, dev, NULL, NULL,
			K_PRIO_PREEMPT(CONFIG_NRF9160_GPS_THREAD_PRIORITY),
			0, 0);

	return 0;
}

#ifdef CONFIG_NRF9160_GPS_HANDLE_MODEM_CONFIGURATION
static int enable_gps(struct device *dev)
{
	int err;
	enum lte_lc_system_mode system_mode;
	enum lte_lc_func_mode functional_mode;

	err = lte_lc_system_mode_get(&system_mode);
	if (err) {
		LOG_ERR("Could not get modem system mode, error: %d", err);
		return err;
	}

	if ((system_mode != LTE_LC_SYSTEM_MODE_GPS) &&
	    (system_mode != LTE_LC_SYSTEM_MODE_LTEM_GPS) &&
	    (system_mode != LTE_LC_SYSTEM_MODE_NBIOT_GPS)) {
		enum lte_lc_system_mode new_mode = LTE_LC_SYSTEM_MODE_GPS;

		if (system_mode == LTE_LC_SYSTEM_MODE_LTEM) {
			new_mode = LTE_LC_SYSTEM_MODE_LTEM_GPS;
		} else if (system_mode == LTE_LC_SYSTEM_MODE_NBIOT) {
			new_mode = LTE_LC_SYSTEM_MODE_NBIOT_GPS;
		}

		LOG_DBG("GPS mode is not enabled, attempting to enable it");

		err = lte_lc_system_mode_set(new_mode);
		if (err) {
			LOG_ERR("Could not enable GPS mode, error: %d", err);
			return err;
		}
	}

	LOG_DBG("GPS mode is enabled");

	err = lte_lc_func_mode_get(&functional_mode);
	if (err) {
		LOG_ERR("Could not get modem's functional mode, error: %d",
			err);
		return err;
	}

	if (functional_mode != LTE_LC_FUNC_MODE_NORMAL) {
		LOG_ERR("GPS is not supported in current functional mode");
		return -EIO;
	}

	return err;
}
#endif

static int start(struct device *dev, struct gps_config *cfg)
{
	int retval;
	struct gps_drv_data *drv_data = dev->driver_data;
	nrf_gnss_fix_retry_t fix_retry;
	nrf_gnss_fix_interval_t fix_interval;
	nrf_gnss_nmea_mask_t nmea_mask = 0;
	nrf_gnss_delete_mask_t delete_mask = 0;

	if (cfg) {
		if (cfg->delete_agps_data) {
			delete_mask = 0xFF;
		}

		if (cfg->timeout < 0) {
			cfg->timeout = 0;
		}

		switch (cfg->nav_mode) {
		case GPS_NAV_MODE_SINGLE_FIX:
			fix_interval = 0;
			fix_retry = cfg->timeout;
			break;
		case GPS_NAV_MODE_CONTINUOUS:
			fix_retry = 0;
			fix_interval = 1;
			break;
		case GPS_NAV_MODE_PERIODIC:
			if (cfg->interval < 10) {
				LOG_ERR("Minimum periodic interval is 10 sec");
				return -EINVAL;
			}

			fix_retry = cfg->timeout;
			fix_interval = cfg->interval;
			break;
		default:
			LOG_ERR("Invalid mode %d, GPS will not start",
				cfg->nav_mode);
			return -EINVAL;
		}
	}

#ifdef CONFIG_NRF9160_GPS_NMEA_GSV
	nmea_mask |= NRF_GNSS_NMEA_GSV_MASK;
#endif
#ifdef CONFIG_NRF9160_GPS_NMEA_GSA
	nmea_mask |= NRF_GNSS_NMEA_GSA_MASK;
#endif
#ifdef CONFIG_NRF9160_GPS_NMEA_GLL
	nmea_mask |= NRF_GNSS_NMEA_GLL_MASK;
#endif
#ifdef CONFIG_NRF9160_GPS_NMEA_GGA
	nmea_mask |= NRF_GNSS_NMEA_GGA_MASK;
#endif
#ifdef CONFIG_NRF9160_GPS_NMEA_RMC
	nmea_mask |= NRF_GNSS_NMEA_RMC_MASK;
#endif

#ifdef CONFIG_NRF9160_GPS_HANDLE_MODEM_CONFIGURATION
	if (enable_gps(dev) != 0) {
		LOG_ERR("Failed to enable GPS");
		return -EIO;
	}
#endif
	retval = nrf_setsockopt(drv_data->socket,
				NRF_SOL_GNSS,
				NRF_SO_GNSS_FIX_RETRY,
				&fix_retry,
				sizeof(fix_retry));
	if (retval != 0) {
		LOG_ERR("Failed to set fix retry value");
		return -EIO;
	}

	retval = nrf_setsockopt(drv_data->socket,
				NRF_SOL_GNSS,
				NRF_SO_GNSS_FIX_INTERVAL,
				&fix_interval,
				sizeof(fix_interval));

	if (retval != 0) {
		LOG_ERR("Failed to set fix interval value");
		return -EIO;
	}

	retval = nrf_setsockopt(drv_data->socket,
				NRF_SOL_GNSS,
				NRF_SO_GNSS_NMEA_MASK,
				&nmea_mask,
				sizeof(nmea_mask));

	if (retval != 0) {
		LOG_ERR("Failed to set nmea mask");
		return -EIO;
	}

	if (cfg->power_mode != GPS_POWER_MODE_DISABLED) {
		nrf_gnss_power_save_mode_t mode = GPS_POWER_MODE_DISABLED;

		if (cfg->power_mode == GPS_POWER_MODE_PERFORMANCE) {
			mode = NRF_GNSS_PSM_DUTY_CYCLING_PERFORMANCE;
		} else if (cfg->power_mode == GPS_POWER_MODE_SAVE) {
			mode = NRF_GNSS_PSM_DUTY_CYCLING_POWER;
		}

		retval = nrf_setsockopt(drv_data->socket,
					NRF_SOL_GNSS,
					NRF_SO_GNSS_POWER_SAVE_MODE,
					&mode,
					sizeof(mode));

		if (retval != 0) {
			LOG_ERR("Failed to start GPS");
			return -EIO;
		}
	}

	retval = nrf_setsockopt(drv_data->socket,
				NRF_SOL_GNSS,
				NRF_SO_GNSS_START,
				&delete_mask,
				sizeof(delete_mask));

	if (retval != 0) {
		LOG_ERR("Failed to start GPS");
		return -EIO;
	}

	atomic_set(&drv_data->gps_is_active, 1);
	k_sem_give(&drv_data->thread_run_sem);

	LOG_DBG("GPS operational");

	return retval;
}

static int init(struct device *dev, gps_event_handler_t handler)
{
	struct gps_drv_data *drv_data = dev->driver_data;
	int err;

	if (handler == NULL) {
		LOG_ERR("No event handler provided");
		return -EINVAL;
	}

	drv_data->handler = handler;

	if (drv_data->socket < 0) {
		drv_data->socket = nrf_socket(NRF_AF_LOCAL, NRF_SOCK_DGRAM,
					  NRF_PROTO_GNSS);

		if (drv_data->socket >= 0) {
			LOG_DBG("GPS socket created, fd: %d", drv_data->socket);
		} else {
			LOG_ERR("Could not init socket (err: %d)",
				drv_data->socket);
			return -EIO;
		}
	}

	k_sem_init(&drv_data->thread_run_sem, 0, 1);

	err = init_thread(dev);
	if (err) {
		LOG_ERR("Could not initialize GPS thread, error: %d",
			err);
		return err;
	}

	return 0;
}

static int deinit(struct device *dev)
{
	struct gps_drv_data *drv_data = dev->driver_data;

	LOG_DBG("Closing GPS socket");
	close(drv_data->socket);

	drv_data->socket = -1;
	drv_data->handler = NULL;

	LOG_DBG("Terminating GPS thread");
	k_thread_abort(drv_data->thread_id);

	return 0;
}

static int setup(struct device *dev)
{
	int err;
	struct gps_drv_data *drv_data = dev->driver_data;

	drv_data->socket = -1;

	atomic_set(&drv_data->gps_is_active, 0);

#if CONFIG_NRF9160_GPS_SET_MAGPIO
	err = at_cmd_write(CONFIG_NRF9160_GPS_MAGPIO_STRING,
				NULL, 0, NULL);
	if (err) {
		LOG_ERR("Could not confiugure MAGPIO, error: %d", err);
		return err;
	}

	LOG_DBG("MAGPIO set: %s",
		log_strdup(CONFIG_NRF9160_GPS_MAGPIO_STRING));
#endif /* CONFIG_NRF9160_GPS_SET_MAGPIO */

#if CONFIG_NRF9160_GPS_SET_COEX0
	err = at_cmd_write(CONFIG_NRF9160_GPS_COEX0_STRING,
				NULL, 0, NULL);
	if (err) {
		LOG_ERR("Could not confiugure COEX0, error: %d", err);
		return err;
	}

	LOG_DBG("COEX0 set: %s",
		log_strdup(CONFIG_NRF9160_GPS_COEX0_STRING));
#endif /* CONFIG_NRF9160_GPS_SET_COEX0 */

	return 0;
}

static int stop(struct device *dev)
{
	struct gps_drv_data *drv_data = dev->driver_data;
	int retval;

	LOG_DBG("Stopping GPS");
	atomic_set(&drv_data->gps_is_active, 0);

	retval = nrf_setsockopt(drv_data->socket,
				NRF_SOL_GNSS,
				NRF_SO_GNSS_STOP,
				NULL,
				0);
	if (retval != 0) {
		LOG_ERR("Failed to stop GPS");
		return -EIO;
	}

	return 0;
}

static int agps_write(struct device *dev, enum gps_agps_type type, void *data,
		      size_t data_len)
{
	int err;
	struct gps_drv_data *drv_data = dev->driver_data;
	nrf_gnss_agps_data_type_t data_type = type_lookup_gps2socket[type];

	err = nrf_sendto(drv_data->socket, data, data_len, 0, &data_type,
			 sizeof(data_type));
	if (err < 0) {
		LOG_ERR("Failed to send A-GPS data to modem, errno: %d", errno);
		return -errno;
	}

	LOG_DBG("Sent A-GPS data to modem, type: %d", type);

	return 0;
}

static struct gps_drv_data gps_drv_data;

static const struct gps_driver_api gps_api_funcs = {
	.init = init,
	.deinit = deinit,
	.start = start,
	.stop = stop,
	.agps_write = agps_write,
};

DEVICE_AND_API_INIT(nrf9160_gps, CONFIG_NRF9160_GPS_DEV_NAME, setup,
		    &gps_drv_data, NULL, APPLICATION,
		    CONFIG_NRF9160_GPS_INIT_PRIO, &gps_api_funcs);
