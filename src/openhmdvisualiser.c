#include <string.h>
#include <stdio.h>

#include "module.h"
#include "openhmdvisualiser.h"

void *openhmd_visualiser_constructor(godot_object *p_instance, void *p_method_data);
void openhmd_visualiser_destructor(godot_object *p_instance, void *p_method_data, void *p_user_data);
godot_variant openhmd_visualiser_load_file(godot_object *p_instance, void *p_method_data,
        void *p_user_data, int p_num_args, godot_variant **p_args);

godot_variant openhmd_visualiser_get_data(godot_object *p_instance, void *p_method_data,
        void *p_user_data, int p_num_args, godot_variant **p_args);

void openhmd_visualiser_register(void *p_handle)
{
    godot_instance_create_func create = { NULL, NULL, NULL };
    create.create_func = &openhmd_visualiser_constructor;

    godot_instance_destroy_func destroy = { NULL, NULL, NULL };
    destroy.destroy_func = &openhmd_visualiser_destructor;

    nativescript_api->godot_nativescript_register_class(p_handle, "OpenHMDVisualiser", "Reference",
            create, destroy);

    {
      godot_instance_method method_data = { &openhmd_visualiser_load_file, NULL, NULL };
      godot_method_attributes attributes = { GODOT_METHOD_RPC_MODE_DISABLED };

      nativescript_api->godot_nativescript_register_method(p_handle, "OpenHMDVisualiser", "load_file",
              attributes, method_data);
    }
}

void *openhmd_visualiser_constructor(godot_object *p_instance, void *p_method_data) {
    openhmd_visualiser *hmdvis = api->godot_alloc(sizeof(openhmd_visualiser));
    strcpy(hmdvis->data, "World from GDNative!");

    return hmdvis;
}

void openhmd_visualiser_destructor(godot_object *p_instance, void *p_method_data, void *p_user_data) {
    api->godot_free(p_user_data);
}

void openhmd_visualiser_report_error(openhmd_visualiser *hmdvis, const char *err_str) {
    /* FIXME: Report error up to the UI */
    printf("ERROR: %s\n", err_str);
}

GDCALLINGCONV godot_variant openhmd_visualiser_load_file(godot_object *p_instance, void *p_method_data,
        void *p_user_data, int p_num_args, godot_variant **p_args) {
    openhmd_visualiser *hmdvis = (openhmd_visualiser *)p_user_data;

    godot_string filename;
    godot_char_string filename_chars;
    godot_variant ret;

    if (p_num_args == 0) {
        openhmd_visualiser_report_error(hmdvis, "Invalid argument");
        api->godot_variant_new_bool(&ret, false);
        goto done;
    }
    else if (api->godot_variant_get_type(p_args[0]) != GODOT_VARIANT_TYPE_STRING) {
        openhmd_visualiser_report_error(hmdvis, "Invalid argument");
        api->godot_variant_new_bool(&ret, false);
        goto done;
    }

    filename = api->godot_variant_as_string(p_args[0]);
    filename_chars = api->godot_string_utf8(&filename);

    /* FIXME: Load the file here and set TRUE/FALSE for the result */
    printf("Loading file %s", api->godot_char_string_get_data(&filename_chars));

    api->godot_char_string_destroy(&filename_chars);
    api->godot_string_destroy(&filename);
    api->godot_variant_new_bool(&ret, true);
done:
    return ret;
}

