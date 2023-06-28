/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <stdio.h>
#include <string.h>
#include <modem/nrf_modem_lib.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

/* To strictly comply with UART timing, enable external XTAL oscillator */
void enable_xtal(void)
{
	struct onoff_manager *clk_mgr;
	static struct onoff_client cli = {};

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	sys_notify_init_spinwait(&cli.notify);
	(void)onoff_request(clk_mgr, &cli);
}

void pin_config(const struct device *dev, gpio_pin_t pin){
	
}

int main(void)
{
	int err;

	printk("The AT host sample started\n");

	err = nrf_modem_lib_init();
	if (err) {
		printk("Modem library initialization failed, error: %d\n", err);
		return 0;
	}
	enable_xtal();

	// Get device
	const struct device *gpio_dev = device_get_binding(DT_NODE_FULL_NAME(DT_NODELABEL(gpio0)));
	if (!gpio_dev) {
		printk("Cannot get device\n");
	}

	// 4,5,18,19,20 externally pulled up?

	// Configure pullup and read
	for (gpio_pin_t i = 0; i < 32; i++) {
		if (
			i == 0 || i == 1 || i == 2 || i == 3 	// Skip UART0
			|| i == 6 || i == 7						// Skip UART1 FC lines
			|| i == 8 || i == 9						// Skip i2c2
			|| i == 13 || i == 14 || i == 15		// Skip spi3
			|| i == 28								// Skip WIFI EN (ext. pulldown)
			) {
			continue;
		}
		err = gpio_pin_configure(gpio_dev, i, GPIO_INPUT | GPIO_ACTIVE_HIGH | GPIO_PULL_UP);
		if (err) {
			printk("Cannot configure pin %d\n", i);
		}
		
		k_sleep(K_MSEC(10));
		int level = gpio_pin_get_raw(gpio_dev, i);
		printk("Read pin %2d: %d", i, level);
		if (!level) printk(" <<< POSSIBLE SHORT");
		printk("\n");
	}

	printk("Ready\n");

	return 0;
}
