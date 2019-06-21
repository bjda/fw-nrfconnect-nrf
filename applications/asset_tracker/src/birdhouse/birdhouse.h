/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */
/**@file
 *
 * @brief   Orientation detector module.
 *
 * Module that uses accelerometer data to detect its orientation.
 */

#ifndef BIRDHOUSE_H__
#define BIRDHOUSE_H__

#ifdef __cplusplus
extern "C" {
#endif

void birdhouse_close();
void birdhouse_open();
void birdhouse_lock();
void birdhouse_unlock();
void birdhouse_init();

#ifdef __cplusplus
}
#endif

#endif /* BIRDHOUSE_H__ */
