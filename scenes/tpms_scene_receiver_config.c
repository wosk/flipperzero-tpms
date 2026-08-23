#include "../tpms_app_i.h"

enum TPMSSettingIndex {
    TPMSSettingIndexFrequency,
    TPMSSettingIndexHopping,
    TPMSSettingIndexModulation,
    TPMSSettingIndexRadio,
    TPMSSettingIndexMinRssi,
    TPMSSettingIndexLock,
};

#define HOPPING_COUNT 2
const char* const hopping_text[HOPPING_COUNT] = {
    "OFF",
    "ON",
};
const uint32_t hopping_value[HOPPING_COUNT] = {
    TPMSHopperStateOFF,
    TPMSHopperStateRunnig,
};

#define RADIO_COUNT 3
const char* const radio_text[RADIO_COUNT] = {
    "Auto",
    "Internal",
    "External",
};
const SubGhzRadioDeviceType radio_value[RADIO_COUNT] = {
    SubGhzRadioDeviceTypeAuto,
    SubGhzRadioDeviceTypeInternal,
    SubGhzRadioDeviceTypeExternalCC1101,
};

#define RSSI_FILTER_COUNT 7
const char* const rssi_filter_text[RSSI_FILTER_COUNT] = {
    "Off",
    "-100 dBm",
    "-95 dBm",
    "-90 dBm",
    "-85 dBm",
    "-80 dBm",
    "-75 dBm",
};
const int16_t rssi_filter_value[RSSI_FILTER_COUNT] = {
    TPMS_RSSI_FILTER_OFF,
    -100,
    -95,
    -90,
    -85,
    -80,
    -75,
};

static void tpms_scene_receiver_config_frequency_to_string(
    char* text_buf,
    size_t text_buf_size,
    uint32_t frequency) {
    snprintf(
        text_buf,
        text_buf_size,
        "%03lu.%02lu",
        (unsigned long)(frequency / 1000000U),
        (unsigned long)((frequency % 1000000U) / 10000U));
}

uint8_t tpms_scene_receiver_config_next_frequency(const uint32_t value, void* context) {
    furi_assert(context);
    TPMSApp* app = context;

    for(uint8_t i = 0; i < subghz_setting_get_frequency_count(app->setting); i++) {
        if(value == subghz_setting_get_frequency(app->setting, i)) {
            return i;
        }
    }

    if(value != tpms_preset_get_default_frequency()) {
        for(uint8_t i = 0; i < subghz_setting_get_frequency_count(app->setting); i++) {
            if(tpms_preset_get_default_frequency() == subghz_setting_get_frequency(app->setting, i)) {
                return i;
            }
        }
    }

    return subghz_setting_get_frequency_default_index(app->setting);
}

uint8_t tpms_scene_receiver_config_next_preset(const char* preset_name, void* context) {
    furi_assert(context);
    UNUSED(context);
    for(uint8_t i = 0; i < tpms_preset_count(); i++) {
        const TPMSPresetDefinition* definition = tpms_preset_get_by_index(i);
        if(definition && !strcmp(definition->name, preset_name)) {
            return i;
        }
    }

    return 0;
}

static uint8_t tpms_scene_receiver_config_radio_value_index(SubGhzRadioDeviceType value) {
    for(uint8_t i = 0; i < RADIO_COUNT; i++) {
        if(radio_value[i] == value) {
            return i;
        }
    }

    return 0;
}

static uint8_t tpms_scene_receiver_config_rssi_filter_value_index(int16_t value) {
    for(uint8_t i = 0; i < RSSI_FILTER_COUNT; i++) {
        if(rssi_filter_value[i] == value) {
            return i;
        }
    }

    return 0;
}

uint8_t tpms_scene_receiver_config_hopper_value_index(
    const uint32_t value,
    const uint32_t values[],
    uint8_t values_count,
    void* context) {
    furi_assert(context);
    UNUSED(values_count);
    TPMSApp* app = context;

    if(value == values[0]) {
        return 0;
    } else {
        variable_item_set_current_value_text(
            (VariableItem*)scene_manager_get_scene_state(
                app->scene_manager, TPMSSceneReceiverConfig),
            " -----");
        return 1;
    }
}

static void tpms_scene_receiver_config_set_frequency(VariableItem* item) {
    TPMSApp* app = variable_item_get_context(item);
    uint8_t index = variable_item_get_current_value_index(item);

    if(app->txrx->hopper_state == TPMSHopperStateOFF) {
        char text_buf[10] = {0};
        tpms_scene_receiver_config_frequency_to_string(
            text_buf, sizeof(text_buf), subghz_setting_get_frequency(app->setting, index));
        variable_item_set_current_value_text(item, text_buf);
        app->txrx->preset->frequency = subghz_setting_get_frequency(app->setting, index);
    } else {
        variable_item_set_current_value_index(
            item, subghz_setting_get_frequency_default_index(app->setting));
    }
}

static void tpms_scene_receiver_config_set_preset(VariableItem* item) {
    TPMSApp* app = variable_item_get_context(item);
    uint8_t index = variable_item_get_current_value_index(item);
    const TPMSPresetDefinition* definition = tpms_preset_get_by_index(index);
    furi_assert(definition);

    variable_item_set_current_value_text(item, definition->name);
    tpms_preset_init(app, definition->name, app->txrx->preset->frequency, NULL, 0);
}

static void tpms_scene_receiver_config_set_radio(VariableItem* item) {
    TPMSApp* app = variable_item_get_context(item);
    uint8_t index = variable_item_get_current_value_index(item);

    variable_item_set_current_value_text(item, radio_text[index]);
    app->txrx->radio_device_type = radio_value[index];
}

static void tpms_scene_receiver_config_set_rssi_filter(VariableItem* item) {
    TPMSApp* app = variable_item_get_context(item);
    uint8_t index = variable_item_get_current_value_index(item);

    variable_item_set_current_value_text(item, rssi_filter_text[index]);
    app->txrx->rssi_filter_dbm = rssi_filter_value[index];
}

static void tpms_scene_receiver_config_set_hopping_running(VariableItem* item) {
    TPMSApp* app = variable_item_get_context(item);
    uint8_t index = variable_item_get_current_value_index(item);

    variable_item_set_current_value_text(item, hopping_text[index]);
    if(hopping_value[index] == TPMSHopperStateOFF) {
        char text_buf[10] = {0};
        tpms_scene_receiver_config_frequency_to_string(
            text_buf, sizeof(text_buf), tpms_preset_get_default_frequency());
        variable_item_set_current_value_text(
            (VariableItem*)scene_manager_get_scene_state(
                app->scene_manager, TPMSSceneReceiverConfig),
            text_buf);
        app->txrx->preset->frequency = tpms_preset_get_default_frequency();
        variable_item_set_current_value_index(
            (VariableItem*)scene_manager_get_scene_state(
                app->scene_manager, TPMSSceneReceiverConfig),
            tpms_scene_receiver_config_next_frequency(tpms_preset_get_default_frequency(), app));
    } else {
        variable_item_set_current_value_text(
            (VariableItem*)scene_manager_get_scene_state(
                app->scene_manager, TPMSSceneReceiverConfig),
            " -----");
        variable_item_set_current_value_index(
            (VariableItem*)scene_manager_get_scene_state(
                app->scene_manager, TPMSSceneReceiverConfig),
            subghz_setting_get_frequency_default_index(app->setting));
    }

    app->txrx->hopper_state = hopping_value[index];
}

static void tpms_scene_receiver_config_var_list_enter_callback(void* context, uint32_t index) {
    furi_assert(context);
    TPMSApp* app = context;
    if(index == TPMSSettingIndexLock) {
        view_dispatcher_send_custom_event(app->view_dispatcher, TPMSCustomEventSceneSettingLock);
    }
}

void tpms_scene_receiver_config_on_enter(void* context) {
    TPMSApp* app = context;
    VariableItem* item;
    uint8_t value_index;

    item = variable_item_list_add(
        app->variable_item_list,
        "Frequency:",
        subghz_setting_get_frequency_count(app->setting),
        tpms_scene_receiver_config_set_frequency,
        app);
    value_index = tpms_scene_receiver_config_next_frequency(app->txrx->preset->frequency, app);
    scene_manager_set_scene_state(app->scene_manager, TPMSSceneReceiverConfig, (uint32_t)item);
    variable_item_set_current_value_index(item, value_index);
    char text_buf[10] = {0};
    tpms_scene_receiver_config_frequency_to_string(
        text_buf, sizeof(text_buf), subghz_setting_get_frequency(app->setting, value_index));
    variable_item_set_current_value_text(item, text_buf);

    item = variable_item_list_add(
        app->variable_item_list,
        "Hopping:",
        HOPPING_COUNT,
        tpms_scene_receiver_config_set_hopping_running,
        app);
    value_index = tpms_scene_receiver_config_hopper_value_index(
        app->txrx->hopper_state, hopping_value, HOPPING_COUNT, app);
    variable_item_set_current_value_index(item, value_index);
    variable_item_set_current_value_text(item, hopping_text[value_index]);

    item = variable_item_list_add(
        app->variable_item_list,
        "Modulation:",
        tpms_preset_count(),
        tpms_scene_receiver_config_set_preset,
        app);
    value_index =
        tpms_scene_receiver_config_next_preset(furi_string_get_cstr(app->txrx->preset->name), app);
    variable_item_set_current_value_index(item, value_index);
    variable_item_set_current_value_text(item, tpms_preset_get_by_index(value_index)->name);

    item = variable_item_list_add(
        app->variable_item_list, "Radio:", RADIO_COUNT, tpms_scene_receiver_config_set_radio, app);
    value_index = tpms_scene_receiver_config_radio_value_index(app->txrx->radio_device_type);
    variable_item_set_current_value_index(item, value_index);
    variable_item_set_current_value_text(item, radio_text[value_index]);

    item = variable_item_list_add(
        app->variable_item_list,
        "Min RSSI:",
        RSSI_FILTER_COUNT,
        tpms_scene_receiver_config_set_rssi_filter,
        app);
    value_index = tpms_scene_receiver_config_rssi_filter_value_index(app->txrx->rssi_filter_dbm);
    variable_item_set_current_value_index(item, value_index);
    variable_item_set_current_value_text(item, rssi_filter_text[value_index]);

    variable_item_list_add(app->variable_item_list, "Lock Keyboard", 1, NULL, NULL);
    variable_item_list_set_enter_callback(
        app->variable_item_list, tpms_scene_receiver_config_var_list_enter_callback, app);

    view_dispatcher_switch_to_view(app->view_dispatcher, TPMSViewVariableItemList);
}

bool tpms_scene_receiver_config_on_event(void* context, SceneManagerEvent event) {
    TPMSApp* app = context;
    bool consumed = false;

    if(event.type == SceneManagerEventTypeCustom) {
        if(event.event == TPMSCustomEventSceneSettingLock) {
            app->lock = TPMSLockOn;
            scene_manager_previous_scene(app->scene_manager);
            consumed = true;
        }
    }
    return consumed;
}

void tpms_scene_receiver_config_on_exit(void* context) {
    TPMSApp* app = context;
    variable_item_list_set_selected_item(app->variable_item_list, 0);
    variable_item_list_reset(app->variable_item_list);
}
