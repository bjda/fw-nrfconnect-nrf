/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <stdio.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/fs/fcb.h>
#include <zephyr/storage/flash_map.h>
#include <modem/nrf_modem_lib.h>
#include <modem/at_cmd_custom.h>

#include <stdlib.h>
#define FLASH_TEST_ID EXTERNAL_FLASH
#define FLASH_TEST_NAME external_flash

#define EXT_FLASH_DEVICE DEVICE_DT_GET(DT_ALIAS(FLASH_TEST_NAME))
#define FLASH_TEST_OFFSET FLASH_AREA_OFFSET(FLASH_TEST_NAME)
#define FLASH_TEST_SIZE FLASH_AREA_SIZE(FLASH_TEST_NAME)
#define BUF_SIZE 1024
#define TRACE_MAGIC_INITIALIZED 0xA7F2C1B8 /* Arbitrary */

static const struct flash_area *flash_test_area;
static const struct device *flash_dev;
static struct flash_sector flash_test_sectors[512];
static const struct flash_parameters *fparam;

static struct fcb test_fcb = {
	.f_flags = FCB_FLAGS_CRC_DISABLED,
};

static struct fcb_entry loc_write;
static struct fcb_entry loc_read;

static uint8_t flash_buf[BUF_SIZE]; /* 1k write buffer */
static uint8_t read_buf[BUF_SIZE]; /* 1k read buffer */

static bool flash_ready; /* Whether flash FCB has been initialized*/
static bool stress_flash; /* Whether to stress the flash */
K_SEM_DEFINE(stress_flash_trigger, 0, 1); /* Giving triggers flash testing */

/* To strictly comply with UART timing, enable external XTAL oscillator */
void enable_xtal(void)
{
	struct onoff_manager *clk_mgr;
	static struct onoff_client cli = {};

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	sys_notify_init_spinwait(&cli.notify);
	(void)onoff_request(clk_mgr, &cli);
}

static int app_flash_init()
{
	int err;
	uint32_t start_time, end_time, spent_time;

	// Open flash area
	err = flash_area_open(FIXED_PARTITION_ID(FLASH_TEST_ID), &flash_test_area);
	printk("flash_area_open: %d\n", err);

	err = flash_area_has_driver(flash_test_area);
	printk("flash_area_has_driver: %d\n", err);

	flash_dev = flash_area_get_device(flash_test_area);
	printk("flash_area_get_device: %p\n", flash_dev);

	// Initialize (erase) flash
	printk("flash_area_erase start\n");
	start_time = k_uptime_get_32();
	err = flash_area_erase(flash_test_area, 0, FLASH_TEST_SIZE);
	end_time = k_uptime_get_32();
	spent_time = end_time - start_time;
	printk("flash_area_erase erased %u bytes in %u ms.\n", FLASH_TEST_SIZE, spent_time);
	printk("The mass erase speed was %u kB/s. Now that's efficiency!\n", 
		   FLASH_TEST_SIZE / spent_time);

	// Get info for FCB
	uint32_t f_sector_cnt = sizeof(flash_test_sectors) / sizeof(struct flash_sector);
	printk("f_sector_cnt: %u\n", f_sector_cnt);

	fparam = flash_get_parameters(flash_dev);

	err = flash_area_get_sectors(FIXED_PARTITION_ID(FLASH_TEST_ID), &f_sector_cnt, flash_test_sectors);
	printk("flash_area_get_sectors: %d\n", err);
	printk("Sectors: %d, first sector: %p, sector size: %d\n",
		f_sector_cnt, flash_test_sectors, flash_test_sectors[0].fs_size);

	// Ignore sectors > 255, FCB is limited to this
	f_sector_cnt = MIN(f_sector_cnt, 255);
    
	// Initialize FCB
	test_fcb.f_magic = TRACE_MAGIC_INITIALIZED;
	test_fcb.f_erase_value = fparam->erase_value;
	test_fcb.f_sector_cnt = f_sector_cnt;
	printk("test_fcb.f_sector_cnt: %d\n", test_fcb.f_sector_cnt);
	test_fcb.f_sectors = flash_test_sectors;
	err = fcb_init(FIXED_PARTITION_ID(FLASH_TEST_ID), &test_fcb);
	printk("fcb_init: %d\n", err);

	// Initialize flash buffer
	for (size_t i = 0; i < BUF_SIZE; i++) {
		flash_buf[i] = rand(); // Arbitrary
	}

	flash_ready = true;
	return 0;
}

static int app_flash_write()
{
	int err;
	uint32_t start_time, end_time, spent_time;

	// Overwrite buffer
	int64_t bufs_written = 0;
	printk("Starting to write to flash\n");
	start_time = k_uptime_get_32();
	while(stress_flash){
		err = fcb_append(&test_fcb, sizeof(flash_buf), &loc_write);
		//printk("fcb_append: %d\n", err);
		if (err == -ENOSPC) {
			fcb_rotate(&test_fcb);
			continue;
		}
		__ASSERT_NO_MSG(err == 0);
		err = flash_area_write(test_fcb.fap, FCB_ENTRY_FA_DATA_OFF(loc_write),
							   &flash_buf, sizeof(flash_buf));
		//printk("flash_area_write: %d\n", err);
		err = fcb_append_finish(&test_fcb, &loc_write);
		//printk("fcb_append_finish: %d\n", err);
		bufs_written ++;
		if (bufs_written % (63*16) == 0){
			//printk("Wrote %lld bytes. Current sector: %ld Current offset:%u\n",
			//	   bufs_written*BUF_SIZE, loc_write.fe_sector->fs_off, loc_write.fe_elem_off);
		}
	}
	end_time = k_uptime_get_32();
	spent_time = end_time - start_time;

	printk("Done rewriting. Wrote %lld bytes in %u ms\n", bufs_written*BUF_SIZE, spent_time);
	printk("The rewrite speed was %lld kB/s. Now that's efficiency!\n", 
		   (bufs_written*BUF_SIZE) / spent_time);
	return 0;
}

static int app_flash_read()
{
	int err;

	// Read data
	err = fcb_getnext(&test_fcb, &loc_read);
	printk("fcb_getnext: %d\n", err);
	__ASSERT_NO_MSG(err == 0);
	__ASSERT_NO_MSG(sizeof(read_buf) == loc_read.fe_data_len);
	err = flash_area_read(test_fcb.fap, FCB_ENTRY_FA_DATA_OFF(loc_read), read_buf, loc_read.fe_data_len);
	printk("flash_area_read: %d\n", err);
	printk("Read %u bytes. Current sector: %ld Current offset:%u\n",
			loc_read.fe_data_len, loc_read.fe_sector->fs_off, loc_read.fe_elem_off);
	printk("Read data[0]: %u\n", read_buf[0]);
	return 0;	
}

static int at_xflashtest_on_cb(char *buf, size_t len, char *at_cmd)
{
	if (stress_flash || !flash_ready) {
		return at_cmd_custom_respond(buf, len, "ERROR\r\n");
	}
	stress_flash = true;
	k_sem_give(&stress_flash_trigger);
	return at_cmd_custom_respond(buf, len, "OK\r\n");
}
AT_CMD_CUSTOM(XFLASHTEST_ON, "AT%XFLASHTEST=1", at_xflashtest_on_cb);

static int at_xflashtest_off_cb(char *buf, size_t len, char *at_cmd)
{
	if (!stress_flash) {
		return at_cmd_custom_respond(buf, len, "ERROR\r\n");
	}
	stress_flash = false;
	return at_cmd_custom_respond(buf, len, "OK\r\n");
}
AT_CMD_CUSTOM(XFLASHTEST_OFF, "AT%XFLASHTEST=0", at_xflashtest_off_cb);

int main()
{
	int err;

	printk("The AT host sample started\n");

	err = nrf_modem_lib_init();
	if (err) {
		printk("Modem library initialization failed, error: %d\n", err);
		return 0;
	}
	enable_xtal();
	app_flash_init();
	printk("Ready\n");

	while (true) {
		k_sem_take(&stress_flash_trigger, K_FOREVER);
		printk("Starting flash stressing\n");
		app_flash_write();
		printk("Ended flash stressing\n");
	}

	return 0;
}
