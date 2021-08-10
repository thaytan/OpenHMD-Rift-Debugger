/*
 * Copyright 2021 Jan Schmidt <thaytan@noraisin.net>
 * SPDX-License-Identifier: MIT
 *
 * The device object provides info about a single device
 * at each step:
 *
 *  device parameters (focal length, camera matrix)
 *  Position + Orientation (pose)
 *  Current video frame
 *  Extracted Blob positions
 *
 */
#include <string.h>
#include <stdio.h>

#include "module.h"
#include "openhmddevice.h"

void *openhmd_device_constructor(godot_object *p_instance, void *p_method_data);
void openhmd_device_destructor(godot_object *p_instance, void *p_method_data, void *p_user_data);

void openhmd_device_register(void *p_handle)
{
    godot_instance_create_func create = { NULL, NULL, NULL };
    create.create_func = &openhmd_device_constructor;

    godot_instance_destroy_func destroy = { NULL, NULL, NULL };
    destroy.destroy_func = &openhmd_device_destructor;

    nativescript_api->godot_nativescript_register_class(p_handle, "OpenHMDDevice", "Reference",
            create, destroy);
}

void *openhmd_device_constructor(godot_object *p_instance, void *p_method_data) {
    openhmd_device *device = api->godot_alloc(sizeof(openhmd_device));
    return device;
}

void openhmd_device_destructor(godot_object *p_instance, void *p_method_data, void *p_user_data) {
    api->godot_free(p_user_data);
}
