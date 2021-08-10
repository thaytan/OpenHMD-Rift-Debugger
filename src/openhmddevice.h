/*
 * Copyright 2021 Jan Schmidt <thaytan@noraisin.net>
 * SPDX-License-Identifier: MIT
 */
#ifndef OPENHMD_DEVICE_H__
#define OPENHMD_DEVICE_H__

#include <gdnative_api_struct.gen.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct openhmd_device {

} openhmd_device;

void openhmd_device_register(void *p_handle);

#ifdef __cplusplus
}
#endif

#endif

