/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <nrf_socket.h>
#include <net/socket.h>
#include <stdio.h>

#define AT_XSYSTEMMODE	"AT\%XSYSTEMMODE=0,0,1,0"
#define AT_CFUN		"AT+CFUN=1"

#define AT_IMEI		"AT+CGSN"

#ifdef CONFIG_BOARD_NRF9160_PCA10090NS
#define AT_MAGPIO      "AT\%XMAGPIO=1,0,0,1,1,1574,1577"
#define AT_COEX0      "AT\%XCOEX0=1,1,1570,1580"
#endif

static const char     at_commands[][31]  = {
				AT_XSYSTEMMODE,
#ifdef CONFIG_BOARD_NRF9160_PCA10090NS
				AT_MAGPIO,
				AT_COEX0,
#endif
				AT_CFUN
			};

static int            fd;

static char           nmea_strings[10][NRF_GNSS_NMEA_MAX_LEN];
static u32_t          nmea_string_cnt;

static bool           got_first_fix;
static bool 	      new_pvt_data;
static u64_t          fix_timestamp;
nrf_gnss_data_frame_t last_pvt;

static char imei[15];

void bsd_recoverable_error_handler(uint32_t error)
{
	printf("Err: %d\n", error);
}

void bsd_irrecoverable_error_handler(uint32_t error)
{
	printf("Irrecoverable: %d\n", error);
}

static int enable_gps(void)
{
	int  at_sock;
	int  bytes_sent;
	int  bytes_received;
	char buf[2];

	at_sock = socket(AF_LTE, 0, NPROTO_AT);
	if (at_sock < 0) {
		return -1;
	}

	for (int i = 0; i < ARRAY_SIZE(at_commands); i++) {
		bytes_sent = send(at_sock, at_commands[i],
				  strlen(at_commands[i]), 0);

		if (bytes_sent < 0) {
			close(at_sock);
			return -1;
		}

		do {
			bytes_received = recv(at_sock, buf, 2, 0);
		} while (bytes_received == 0);

		if (memcmp(buf, "OK", 2) != 0) {
			close(at_sock);
			return -1;
		}
	}

	close(at_sock);

	return 0;
}

static int get_imei(char *buf)
{
	int  at_sock;
	int  bytes_sent;
	int  bytes_received;

	at_sock = socket(AF_LTE, 0, NPROTO_AT);
	if (at_sock < 0) {
		return -1;
	}

	bytes_sent = send(at_sock, AT_IMEI,
				strlen(AT_IMEI), 0);

	if (bytes_sent < 0) {
		close(at_sock);
		return -1;
	}

	do {
		bytes_received = recv(at_sock, buf, 15, 0);
	} while (bytes_received == 0);
	printk("%d:%s\n", bytes_received, buf);
	close(at_sock);
	return 0;
}

static int init_app(void)
{
	u16_t fix_retry     = 0;
	u16_t fix_interval  = 1;
	u16_t nmea_mask     = NRF_GNSS_NMEA_GSV_MASK |
			      NRF_GNSS_NMEA_GSA_MASK |
			      NRF_GNSS_NMEA_GLL_MASK |
			      NRF_GNSS_NMEA_GGA_MASK |
			      NRF_GNSS_NMEA_RMC_MASK;
	nrf_gnss_elevation_mask_t elevation_mask = 0;
	nrf_gnss_delete_mask_t  delete_mask  = 0;
	int   retval;

	if (get_imei(imei) != 0) {
		printk("get imei fail\n");
	}

	if (enable_gps() != 0) {
		printk("Failed to enable GPS\n");
		return -1;
	}

	fd = nrf_socket(NRF_AF_LOCAL, NRF_SOCK_DGRAM, NRF_PROTO_GNSS);

	if (fd >= 0) {
		printk("Socket created\n");
	} else {
		printk("Could not init socket (err: %d)\n", fd);
		return -1;
	}

	retval = nrf_setsockopt(fd,
				NRF_SOL_GNSS,
				NRF_SO_GNSS_FIX_RETRY,
				&fix_retry,
				sizeof(uint16_t));

	if (retval != 0) {
		printk("Failed to set fix retry value\n");
		return -1;
	}

	retval = nrf_setsockopt(fd,
				NRF_SOL_GNSS,
				NRF_SO_GNSS_FIX_INTERVAL,
				&fix_interval,
				sizeof(uint16_t));

	if (retval != 0) {
		printk("Failed to set fix interval value\n");
		return -1;
	}

	retval = nrf_setsockopt(fd,
				NRF_SOL_GNSS,
				NRF_SO_GNSS_NMEA_MASK,
				&nmea_mask,
				sizeof(uint16_t));

	if (retval != 0) {
		printk("Failed to set nmea mask\n");
		return -1;
	}

	retval = nrf_setsockopt(fd,
				NRF_SOL_GNSS,
				NRF_SO_GNSS_ELEVATION_MASK,
				&elevation_mask,
				sizeof(elevation_mask));

	if (retval != 0) {
		printk("Failed to set elevation mask: %d\n",retval);
		return -1;
	}

	retval = nrf_setsockopt(fd,
				NRF_SOL_GNSS,
				NRF_SO_GNSS_START,
				&delete_mask,
				sizeof(delete_mask));

	if (retval != 0) {
		printk("Failed to start GPS: %d\n",retval);
		return -1;
	}

	return 0;
}

static void print_satellite_stats(nrf_gnss_data_frame_t *pvt_data)
{
	u8_t  tracked          = 0;
	u8_t  in_fix           = 0;
	u8_t  unhealthy        = 0;

	for (int i = 0; i < NRF_GNSS_MAX_SATELLITES; ++i) {

		if ((pvt_data->pvt.sv[i].sv > 0) &&
		    (pvt_data->pvt.sv[i].sv < 33)) {

			tracked++;

			if (pvt_data->pvt.sv[i].flags &
					NRF_GNSS_PVT_FLAG_FIX_VALID_BIT) {
				in_fix++;
			}

			if (pvt_data->pvt.sv[i].flags &
					NRF_GNSS_SV_FLAG_UNHEALTHY) {
				unhealthy++;
			}
		}
	}

	printk("Tracking: %d Using: %d Unhealthy: %d", tracked,
						       in_fix,
						       unhealthy);

	printk("\nSeconds since last fix %lld\n",
			(k_uptime_get() - fix_timestamp) / 1000);
}

static void print_pvt_csv(nrf_gnss_data_frame_t *pvt_data)
{
	printf("%s,", imei);
	printf("%d,", k_uptime_get_32());
	printf("%f,", pvt_data->pvt.longitude);
	printf("%f,", pvt_data->pvt.latitude);
	printf("%f,", pvt_data->pvt.altitude);
	printf("%f,", pvt_data->pvt.accuracy);
	printf("%f,", pvt_data->pvt.speed);
	printf("%f,", pvt_data->pvt.heading);
	printf("%02u-%02u-%02u,", pvt_data->pvt.datetime.day,
					       pvt_data->pvt.datetime.month,
					       pvt_data->pvt.datetime.year);
	printf("%02u:%02u:%02u.%03u,", pvt_data->pvt.datetime.hour,
					       pvt_data->pvt.datetime.minute,
					      pvt_data->pvt.datetime.seconds,
					      pvt_data->pvt.datetime.ms);
	printf("%f,", pvt_data->pvt.pdop);
	printf("%f,", pvt_data->pvt.hdop);
	printf("%f,", pvt_data->pvt.vdop);
	printf("%f,", pvt_data->pvt.tdop);
	printf("%d,", pvt_data->pvt.flags);

	for (size_t i = 0; i < NRF_GNSS_MAX_SATELLITES; i++) {
		printf("%d,", pvt_data->pvt.sv[i].sv);
		printf("%d,", pvt_data->pvt.sv[i].cn0);
		printf("%d,", pvt_data->pvt.sv[i].elevation);
		printf("%d,", pvt_data->pvt.sv[i].azimuth);
		printf("%d,", pvt_data->pvt.sv[i].flags);
		printf("%d,", pvt_data->pvt.sv[i].signal);
	}
	printf("\n");
}

static void print_pvt_data(nrf_gnss_data_frame_t *pvt_data)
{
	printf("timestamp: %f,", pvt_data->pvt.longitude);
	printf("longitude: %f,", pvt_data->pvt.longitude);
	printf("latitude: %f,", pvt_data->pvt.latitude);
	printf("altitude: %f,", pvt_data->pvt.altitude);
	printf("accuracy: %f,", pvt_data->pvt.accuracy);
	printf("speed: %f,", pvt_data->pvt.speed);
	printf("heading: %f,", pvt_data->pvt.heading);
	printf("date: %02u-%02u-%02u,", pvt_data->pvt.datetime.day,
					       pvt_data->pvt.datetime.month,
					       pvt_data->pvt.datetime.year);
	printf("time: %02u:%02u:%02u,", pvt_data->pvt.datetime.hour,
					       pvt_data->pvt.datetime.minute,
					      pvt_data->pvt.datetime.seconds);
	printf("pdop: %f,", pvt_data->pvt.pdop);
	printf("hdop: %f,", pvt_data->pvt.hdop);
	printf("vdop: %f,", pvt_data->pvt.vdop);
	printf("tdop: %f,", pvt_data->pvt.tdop);
	printf("flags: %d,\n", pvt_data->pvt.flags);

	for (size_t i = 0; i < NRF_GNSS_MAX_SATELLITES; i++) {
		printf("sv: %d,", pvt_data->pvt.sv[i].sv);
		printf("cn0: %d,", pvt_data->pvt.sv[i].cn0);
		printf("elevation: %d,", pvt_data->pvt.sv[i].elevation);
		printf("azimuth: %d,", pvt_data->pvt.sv[i].azimuth);
		printf("flags: %d,", pvt_data->pvt.sv[i].flags);
		printf("signal: %d\n", pvt_data->pvt.sv[i].signal);
	}
	printf("\n");
}

static void print_nmea_data(void)
{
	printk("NMEA strings:\n");

	for (int i = 0; i < nmea_string_cnt; ++i) {
		printk("%s\n", nmea_strings[i]);
	}
}

int process_gps_data(nrf_gnss_data_frame_t *gps_data)
{
	int retval;

	retval = nrf_recv(fd, gps_data, sizeof(nrf_gnss_data_frame_t), 0);

	if (retval > 0) {

		switch (gps_data->data_id) {
		case NRF_GNSS_PVT_DATA_ID:

			memcpy(&last_pvt, gps_data, sizeof(nrf_gnss_data_frame_t));
			if ((gps_data->pvt.flags &
				NRF_GNSS_PVT_FLAG_FIX_VALID_BIT)
				== NRF_GNSS_PVT_FLAG_FIX_VALID_BIT) {

				if (!got_first_fix) {
					got_first_fix = true;
				}
			}

			new_pvt_data = true;
			break;

		case NRF_GNSS_NMEA_DATA_ID:
			if (nmea_string_cnt < 10) {
				memcpy(nmea_strings[nmea_string_cnt++],
				       gps_data->nmea,
				       retval);
			}
			break;

		default:
			break;
		}
	}

	return retval;
}

int main(void)
{
	nrf_gnss_data_frame_t gps_data;
	bool printed_fix_time = false;
	u32_t gps_started;

	printk("Staring GPS application\n");

	if (init_app() != 0) {
		return -1;
	}

	gps_started = k_uptime_get_32();

	printk("Getting GPS data...\n");
	printk("imei,timestamp,longitude,latitude,altitude,accuracy,speed,heading,date,time,pdop,hdop,vdop,tdop,flags,sv0_sv,sv0_cn0,sv0_elevation,sv0_azimuth,sv0_flags,sv0_signal,sv1_sv,sv1_cn0,sv1_elevation,sv1_azimuth,sv1_flags,sv1_signal,sv2_sv,sv2_cn0,sv2_elevation,sv2_azimuth,sv2_flags,sv2_signal,sv3_sv,sv3_cn0,sv3_elevation,sv3_azimuth,sv3_flags,sv3_signal,sv4_sv,sv4_cn0,sv4_elevation,sv4_azimuth,sv4_flags,sv4_signal,sv5_sv,sv5_cn0,sv5_elevation,sv5_azimuth,sv5_flags,sv5_signal,sv6_sv,sv6_cn0,sv6_elevation,sv6_azimuth,sv6_flags,sv6_signal,sv7_sv,sv7_cn0,sv7_elevation,sv7_azimuth,sv7_flags,sv7_signal,sv8_sv,sv8_cn0,sv8_elevation,sv8_azimuth,sv8_flags,sv8_signal,sv9_sv,sv9_cn0,sv9_elevation,sv9_azimuth,sv9_flags,sv9_signal,sv10_sv,sv10_cn0,sv10_elevation,sv10_azimuth,sv10_flags,sv10_signal,sv11_sv,sv11_cn0,sv11_elevation,sv11_azimuth,sv11_flags,sv11_signal\n");

	while (1) {

		process_gps_data(&gps_data);

		if (new_pvt_data) {
			new_pvt_data = false;
			print_pvt_csv(&gps_data);
			//print_pvt_data(&gps_data);
		}

		if (got_first_fix && !printed_fix_time) {
			printed_fix_time = true;
			printk("***** First fix time: %d\n",
				k_uptime_get_32() - gps_started);
		}
	}

	return 0;
}

