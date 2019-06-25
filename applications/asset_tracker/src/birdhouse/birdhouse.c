/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

/* Constants for servo operation. All time units are us. */
#include <assert.h>
#include "birdhouse.h"

#include "ui.h"

#define PWM_PERIOD 20000
#define DOOR_PWM_NMOS_ID UI_NMOS_1
#define LOCK_PWM_NMOS_ID UI_NMOS_2
#define DOOR_PWR_NMOS_ID UI_NMOS_3
#define LOCK_PWR_NMOS_ID UI_NMOS_4
#define DOOR_PWM_WIDTH_CLOSE 1930
#define DOOR_PWM_WIDTH_OPEN 1020
#define LOCK_PWM_WIDTH_LOCK 1930
#define LOCK_PWM_WIDTH_UNLOCK 1020

/* Time to wait for servo to operate */
#define DOOR_OPERATION_TIME_MS 500
#define LOCK_OPERATION_TIME_MS 500
#define BOTH_OPERATION_TIME_MS 500

/* Time to wait for servo to be inoperable after power loss */
#define DOOR_DISSIPATE_TIME_MS 50
#define LOCK_DISSIPATE_TIME_MS 50
#define BOTH_DISSIPATE_TIME_MS 50

static enum state_t {LOCKED, CLOSED, OPEN} state;

void operate_door(u32_t target_width){
	ui_nmos_pwm_set(DOOR_PWM_NMOS_ID, PWM_PERIOD, 
					PWM_PERIOD - target_width);
	k_usleep(2 * PWM_PERIOD); /* wait for stable PWM before powering servo */
	ui_nmos_write(DOOR_PWR_NMOS_ID, 1);
	k_sleep(DOOR_OPERATION_TIME_MS);
	ui_nmos_write(DOOR_PWR_NMOS_ID, 0);
	k_usleep(DOOR_DISSIPATE_TIME_MS);
	ui_nmos_write(DOOR_PWM_NMOS_ID, 0);
}

void operate_lock(u32_t target_width){
	ui_nmos_pwm_set(LOCK_PWM_NMOS_ID, PWM_PERIOD, 
					PWM_PERIOD - target_width);
	k_usleep(2 * PWM_PERIOD); /* wait for stable PWM before powering servo */
	ui_nmos_write(LOCK_PWR_NMOS_ID, 1);
	k_sleep(LOCK_OPERATION_TIME_MS);
	ui_nmos_write(LOCK_PWR_NMOS_ID, 0);
	k_usleep(LOCK_DISSIPATE_TIME_MS);
	ui_nmos_write(LOCK_PWM_NMOS_ID, 0);
}

void internal_close(){
	operate_door(DOOR_PWM_WIDTH_CLOSE);
	state = CLOSED;
}

void internal_open(){
	operate_door(DOOR_PWM_WIDTH_OPEN);
	state = OPEN;
}

void internal_lock(){
	operate_lock(LOCK_PWM_WIDTH_LOCK);
	state = LOCKED;
}

void internal_unlock(){
	operate_lock(LOCK_PWM_WIDTH_UNLOCK);
	state = CLOSED; // The unlocked door is still closed
}

int birdhouse_lock(){
	switch (state) {
		case LOCKED:
		break;
		case CLOSED:
			internal_lock();
		break;
		case OPEN:
			internal_close();
			internal_lock();
		break;
		default:
			return -ENOTRECOVERABLE;
	}
	return 0;
}

int birdhouse_unlock(){
	switch (state) {
		case LOCKED:
			internal_unlock();
		break;
		case CLOSED:
		break;
		case OPEN:
		break;
		default:
			return -ENOTRECOVERABLE;
	}
	return 0;
}

int birdhouse_close(){
	switch (state) {
		case LOCKED:
		break;
		case CLOSED:
		break;
		case OPEN:
			internal_close();
		break;
		default:
			return -ENOTRECOVERABLE;
	}
	return 0;
}

int birdhouse_open(){
	switch (state) {
		case LOCKED:
			internal_unlock();
			internal_open();
		break;
		case CLOSED:
			internal_open();
		break;
		case OPEN:
		break;
		default:
			return -ENOTRECOVERABLE;
	}
	return 0;
}

int birdhouse_init(){
	/* Servo initialization. cycle value is low time */
	ui_nmos_pwm_set(DOOR_PWM_NMOS_ID, PWM_PERIOD, 
						PWM_PERIOD - DOOR_PWM_WIDTH_CLOSE);
	ui_nmos_pwm_set(LOCK_PWM_NMOS_ID, PWM_PERIOD, 
						PWM_PERIOD - LOCK_PWM_WIDTH_LOCK);
    k_usleep(2 * PWM_PERIOD); /* wait for stable PWM before powering servos */
	ui_nmos_write(DOOR_PWR_NMOS_ID, 1);
	ui_nmos_write(LOCK_PWR_NMOS_ID, 1);
	k_sleep(BOTH_OPERATION_TIME_MS);
	ui_nmos_write(DOOR_PWR_NMOS_ID, 0);
	ui_nmos_write(LOCK_PWR_NMOS_ID, 0);
	k_usleep(LOCK_DISSIPATE_TIME_MS);
	ui_nmos_write(DOOR_PWM_NMOS_ID, 0);
	ui_nmos_write(LOCK_PWM_NMOS_ID, 0);
	state = LOCKED;
	return 0;
}