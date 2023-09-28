/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>

static int npm1300_init(void)
{
#if defined(CONFIG_WIFI)
	// Give the WiFi PMIC time to start
	k_sleep(K_MSEC(5));
#endif /* defined(CONFIG_WIFI) */
	return 0;
}

SYS_INIT(npm1300_init, POST_KERNEL, CONFIG_NPM1300_INIT_PRIORITY);
