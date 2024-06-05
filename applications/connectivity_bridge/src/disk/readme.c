/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>

#if defined(CONFIG_SECURE_BOOT)
#include <fw_info.h>
#include <bl_storage.h>
#endif

#define MODULE file_readme
#include "fs_event.h"

static const char file_contents[] = {
#include "readme.h"
};

#if CONFIG_FS_FATFS_LFN
#define FILE_NAME         "README.txt"
#else
#define FILE_NAME         "README.TXT"
#endif
#define FILE_CONTENTS     file_contents
#define FILE_CONTENTS_LEN strlen(file_contents)

#if defined(CONFIG_SECURE_BOOT)
#define NSIB_VERSION_PREFIX "Updatable bootloader versions:"
static const struct fw_info *s0_info;
static const struct fw_info *s1_info;
static char nsib_version_line[128];
#endif

static bool app_event_handler(const struct app_event_header *aeh)
{
	if (is_fs_event(aeh)) {
		const struct fs_event *event =
			cast_fs_event(aeh);

		if (event->req == FS_REQUEST_CREATE_FILE) {
			int err;
			int len;

			err = fs_event_helper_file_write(
				event->mnt_point,
				FILE_NAME,
				FILE_CONTENTS,
				FILE_CONTENTS_LEN);

			__ASSERT_NO_MSG(err == 0);

/* If NSIB is used, add the updatable bootloader version in each slot to the readme file */
#if defined(CONFIG_SECURE_BOOT)
			s0_info = fw_info_find(s0_address_read());
			s1_info = fw_info_find(s1_address_read());
			len = snprintf(nsib_version_line, sizeof(nsib_version_line), "%s S0: %u, S1: %u\n",
				NSIB_VERSION_PREFIX, s0_info->version, s1_info->version);

			err = fs_event_helper_file_write(
				event->mnt_point,
				FILE_NAME,
				nsib_version_line,
				len);
			__ASSERT_NO_MSG(err == 0);
#endif
		}

		return false;
	}

	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE(MODULE, fs_event);
