#include "tpms_preset.h"

#include <string.h>

static uint8_t tpms_preset_toyota_pmvc2x_data[] = {
    0x02, 0x0D, 0x0B, 0x06, 0x08, 0x32, 0x07, 0x04, 0x14, 0x00, 0x13, 0x02,
    0x12, 0x04, 0x11, 0x93, 0x10, 0x59, 0x15, 0x41, 0x18, 0x18, 0x19, 0x16,
    0x1D, 0x91, 0x1C, 0x00, 0x1B, 0x07, 0x20, 0xFB, 0x22, 0x10, 0x21, 0x56,
    0x00, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const TPMSPresetDefinition tpms_preset_definitions[] = {
    {
        .name = TPMS_PRESET_TOYOTA_PMVC2X_NAME,
        .short_name = "FM",
        .radio_preset = FuriHalSubGhzPresetCustom,
        .custom_data = tpms_preset_toyota_pmvc2x_data,
        .custom_data_size = sizeof(tpms_preset_toyota_pmvc2x_data),
    },
    {
        .name = "AM270",
        .short_name = "AM",
        .radio_preset = FuriHalSubGhzPresetOok270Async,
    },
    {
        .name = "AM650",
        .short_name = "AM",
        .radio_preset = FuriHalSubGhzPresetOok650Async,
    },
    {
        .name = "FM238",
        .short_name = "FM",
        .radio_preset = FuriHalSubGhzPreset2FSKDev238Async,
    },
    {
        .name = "FM12K",
        .short_name = "FM",
        .radio_preset = FuriHalSubGhzPreset2FSKDev12KAsync,
    },
    {
        .name = "FM476",
        .short_name = "FM",
        .radio_preset = FuriHalSubGhzPreset2FSKDev476Async,
    },
};

size_t tpms_preset_count(void) {
    return COUNT_OF(tpms_preset_definitions);
}

const TPMSPresetDefinition* tpms_preset_get_by_index(size_t index) {
    if(index >= tpms_preset_count()) {
        return NULL;
    }

    return &tpms_preset_definitions[index];
}

const TPMSPresetDefinition* tpms_preset_get_by_name(const char* name) {
    if(name == NULL) {
        return NULL;
    }

    for(size_t index = 0; index < tpms_preset_count(); index++) {
        const TPMSPresetDefinition* definition = &tpms_preset_definitions[index];
        if(strcmp(definition->name, name) == 0) {
            return definition;
        }
    }

    return NULL;
}

const TPMSPresetDefinition* tpms_preset_get_default(void) {
    return &tpms_preset_definitions[0];
}

const char* tpms_preset_get_short_name(const char* name) {
    const TPMSPresetDefinition* definition = tpms_preset_get_by_name(name);
    return definition ? definition->short_name : "CU";
}

FuriHalSubGhzPreset tpms_preset_get_radio_preset(const char* name) {
    const TPMSPresetDefinition* definition = tpms_preset_get_by_name(name);
    return definition ? definition->radio_preset : FuriHalSubGhzPresetCustom;
}

uint32_t tpms_preset_get_default_frequency(void) {
    return TPMS_PRESET_TOYOTA_PMVC2X_FREQUENCY;
}

bool tpms_preset_init_radio(
    SubGhzRadioPreset* preset,
    const char* preset_name,
    uint32_t frequency,
    uint8_t* preset_data,
    size_t preset_data_size) {
    furi_assert(preset);
    furi_assert(preset_name);

    const TPMSPresetDefinition* definition = tpms_preset_get_by_name(preset_name);

    furi_string_set(preset->name, definition ? definition->name : preset_name);
    preset->frequency = frequency;

    if(definition && definition->radio_preset == FuriHalSubGhzPresetCustom) {
        preset->data = (uint8_t*)definition->custom_data;
        preset->data_size = definition->custom_data_size;
    } else if(definition) {
        preset->data = NULL;
        preset->data_size = 0;
    } else {
        preset->data = preset_data;
        preset->data_size = preset_data_size;
    }

    return true;
}
