/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>


#include <app_event_manager.h>
#include <drivers/hwinfo.h>

#define MODULE main
#include "module_state_event.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(MODULE);

#define USB_SERIALNUMBER_TEMPLATE "THINGY91_%04X%08X"

static uint8_t usb_serial_str[] = "THINGY91_12PLACEHLDRS";

/* Overriding weak function to set iSerialNumber at runtime. */
uint8_t *usb_update_sn_string_descriptor(void)
{
	uint32_t hwid[2];
	int hwlen;

	memset(hwid, 0, sizeof(hwid));

	hwlen = hwinfo_get_device_id((uint8_t *)hwid, sizeof(hwid));
	if (hwlen > 0) {
		snprintk(usb_serial_str, sizeof(usb_serial_str), USB_SERIALNUMBER_TEMPLATE,
					(hwid[1] & 0x0000FFFF)|0x0000C000,
					hwid[0]);
	}

	return usb_serial_str;
}

void main(void)
{
	if (app_event_manager_init()) {
		LOG_ERR("Application Event Manager not initialized");
	} else {
		module_set_state(MODULE_STATE_READY);
	}
}
