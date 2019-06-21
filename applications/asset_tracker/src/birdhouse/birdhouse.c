/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

/* Constants for servo operation. All time units are us. */
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
#define DOOR_DISSIPATE_TIME_MS 500
#define LOCK_DISSIPATE_TIME_MS 500
#define BOTH_DISSIPATE_TIME_MS 500

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

void birdhouse_close(){
	operate_door(DOOR_PWM_WIDTH_CLOSE);
}

void birdhouse_open(){
	operate_door(DOOR_PWM_WIDTH_OPEN);
}

void birdhouse_lock(){
	operate_lock(LOCK_PWM_WIDTH_LOCK);
}

void birdhouse_unlock(){
	operate_lock(LOCK_PWM_WIDTH_UNLOCK);
}

void birdhouse_init(){
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
}