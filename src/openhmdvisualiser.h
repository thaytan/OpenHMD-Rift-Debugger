/*
 * Copyright 2021 Jan Schmidt <thaytan@noraisin.net>
 * SPDX-License-Identifier: MIT
 */
#ifndef OPENHMD_VISUALISER_H__
#define OPENHMD_VISUALISER_H__

#include <gdnative_api_struct.gen.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct openhmd_visualiser {
    char data[256];
} openhmd_visualiser;

void openhmd_visualiser_register(void *p_handle);

#ifdef __cplusplus
}
#endif

#endif
