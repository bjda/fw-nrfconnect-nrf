/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <stdio.h>
#include <ctype.h>
#include <logging/log.h>
#include <uart.h>
#include <string.h>
#include <init.h>
#include <at_cmd.h>

LOG_MODULE_REGISTER(at_host, CONFIG_AT_HOST_LOG_LEVEL);

/* Stack definition for AT host workqueue */
#define AT_HOST_STACK_SIZE 512
K_THREAD_STACK_DEFINE(at_host_stack_area, AT_HOST_STACK_SIZE);

#define CONFIG_UART_0_NAME      "UART_0"
#define CONFIG_UART_1_NAME      "UART_1"
#define CONFIG_UART_2_NAME      "UART_2"

#define INVALID_DESCRIPTOR      -1

#define OK_STR    "OK\r\n"
#define ERROR_STR "ERROR\r\n"

#if CONFIG_AT_HOST_CMD_MAX_LEN > CONFIG_AT_CMD_RESPONSE_MAX_LEN
#define AT_BUF_SIZE CONFIG_AT_HOST_CMD_MAX_LEN
#else
#define AT_BUF_SIZE CONFIG_AT_CMD_RESPONSE_MAX_LEN
#endif

/** @brief Termination Modes. */
enum term_modes {
	MODE_NULL_TERM, /**< Null Termination */
	MODE_CR,        /**< CR Termination */
	MODE_LF,        /**< LF Termination */
	MODE_CR_LF,     /**< CR+LF Termination */
	MODE_COUNT      /* Counter of term_modes */
};


/** @brief UARTs. */
enum select_uart {
	UART_0,
	UART_1,
	UART_2
};

static enum term_modes term_mode;
static struct device *uart_dev;
static char at_buf[AT_BUF_SIZE]; /* AT command and modem response buffer */
static struct k_work_q at_host_work_q;
static struct k_work cmd_send_work;



static inline void write_uart_string(char *str, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		uart_poll_out(uart_dev, str[i]);
	}
}

static void response_handler(char *response)
{
	int len = strlen(response);

	/* Forward the data over UART */
	if (len > 0) {
		write_uart_string(response, len);
	}
}

static void cmd_send(struct k_work *work)
{
	size_t            chars;
	char              str[15];
	enum at_cmd_state state;
	int               err;

	ARG_UNUSED(work);

	err = at_cmd_write(at_buf, at_buf,
			   CONFIG_AT_CMD_RESPONSE_MAX_LEN, &state);
	if (err < 0) {
		LOG_ERR("Error while processing AT command: %d", err);
		state = AT_CMD_ERROR;
	}

	/* Handle the various error responses from modem */
	switch (state) {
	case AT_CMD_OK:
		write_uart_string(at_buf, strlen(at_buf));
		write_uart_string(OK_STR, sizeof(OK_STR) - 1);
		break;
	case AT_CMD_ERROR:
		write_uart_string(ERROR_STR, sizeof(ERROR_STR) - 1);
		break;
	case AT_CMD_ERROR_CMS:
		chars = sprintf(str, "+CMS: %d\r\n", err);
		write_uart_string(str, ++chars);
		break;
	case AT_CMD_ERROR_CME:
		chars = sprintf(str, "+CME: %d\r\n", err);
		write_uart_string(str, ++chars);
		break;
	default:
		break;
	}

	uart_irq_rx_enable(uart_dev);
}

static void uart_rx_handler(u8_t character)
{
	static bool inside_quotes;
	static size_t at_cmd_len;

	/* Handle control characters */
	switch (character) {
	case 0x08: /* Backspace. */
		/* Fall through. */
	case 0x7F: /* DEL character */
		if (at_cmd_len > 0) {
			at_cmd_len--;
		}
		return;
	}

	/* Handle termination sequence, if outside quotes */
	if (!inside_quotes) {
		switch (term_mode) {
		case MODE_NULL_TERM:
			if (character == '\0') {
				goto send;
			}
			break;
		case MODE_CR:
			if (character == '\r') {
				goto send;
			}
			break;
		case MODE_LF:
			if (character == '\n') {
				goto send;
			}
			break;
		case MODE_CR_LF:
			if ((at_buf[at_cmd_len - 1] == '\r') &&
			    (character == '\n')) {
				at_cmd_len--; /* Remove preceding CR */
				goto send;
			}
			break;
		default:
			LOG_ERR("Invalid termination mode: %d", term_mode);
			break;
		}
	}

	/* Detect AT command buffer overflow, leaving space for null */
	if (at_cmd_len + 1 > CONFIG_AT_HOST_CMD_MAX_LEN - 1) {
		LOG_ERR("Buffer overflow, dropping '%c'\n", character);
		return;
	}

	/* Write character to AT buffer */
	at_buf[at_cmd_len] = character;
	at_cmd_len++;

	/* Handle special written character */
	if (character == '"') {
		inside_quotes = !inside_quotes;
	}

	return;
send:
	uart_irq_rx_disable(uart_dev); /* Block the UART to protect buffer */
	at_buf[at_cmd_len] = '\0'; /* Terminate the command string */

	/* Reset UART handler state */
	inside_quotes = false;
	at_cmd_len = 0;

	k_work_submit_to_queue(&at_host_work_q, &cmd_send_work);
}

static void isr(struct device *dev)
{
	u8_t character;

	uart_irq_update(dev);

	if (!uart_irq_rx_ready(dev)) {
		return;
	}

	while (uart_fifo_read(dev, &character, 1)) {
		uart_rx_handler(character);
	}
}

static int at_uart_init(char *uart_dev_name)
{
	int err;
	u8_t dummy;

	uart_dev = device_get_binding(uart_dev_name);
	if (uart_dev == NULL) {
		LOG_ERR("Cannot bind %s\n", uart_dev_name);
		return -EINVAL;
	}

	u32_t start_time = k_uptime_get_32();

	/* Wait for the UART line to become valid */
	do {
		err = uart_err_check(uart_dev);
		if (err) {
			if (k_uptime_get_32() - start_time >
			    CONFIG_AT_HOST_UART_INIT_TIMEOUT) {
				return -EIO;
			}

			LOG_ERR("UART check failed: %d. "
				"Dropping buffer and retrying.", err);

			while (uart_fifo_read(uart_dev, &dummy, 1)) {
				/* Do nothing with the data */
			}
			k_sleep(10);
		}
	} while (err);

	uart_irq_callback_set(uart_dev, isr);
	return err;
}

static int at_host_init(struct device *arg)
{
	char *uart_dev_name;
	int err;
	enum select_uart uart_id = CONFIG_AT_HOST_UART;
	enum term_modes mode = CONFIG_AT_HOST_TERMINATION;

	ARG_UNUSED(arg);

	/* Choosing the termination mode */
	if (mode < MODE_COUNT) {
		term_mode = mode;
	} else {
		return -EINVAL;
	}

	/* Choose which UART to use */
	switch (uart_id) {
	case UART_0:
		uart_dev_name = CONFIG_UART_0_NAME;
		break;
	case UART_1:
		uart_dev_name = CONFIG_UART_1_NAME;
		break;
	case UART_2:
		uart_dev_name = CONFIG_UART_2_NAME;
		break;
	default:
		LOG_ERR("Unknown UART instance %d", uart_id);
		return -EINVAL;
	}

	at_cmd_set_notification_handler(response_handler);

	/* Initialize the UART module */
	err = at_uart_init(uart_dev_name);
	if (err) {
		LOG_ERR("UART could not be initialized: %d", err);
		return -EFAULT;
	}

	k_work_init(&cmd_send_work, cmd_send);
	k_work_q_start(&at_host_work_q, at_host_stack_area,
		       K_THREAD_STACK_SIZEOF(at_host_stack_area),
		       CONFIG_AT_HOST_THREAD_PRIO);
	uart_irq_rx_enable(uart_dev);

	return err;
}

SYS_INIT(at_host_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
