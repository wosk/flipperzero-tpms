#pragma once

#include <lib/subghz/devices/preset.h>
#include <lib/subghz/types.h>

typedef struct {
    const char* name;
    const char* short_name;
    FuriHalSubGhzPreset radio_preset;
    const uint8_t* custom_data;
    size_t custom_data_size;
} TPMSPresetDefinition;

#define TPMS_PRESET_TOYOTA_PMVC2X_NAME "Toyota PMV-C2x"
#define TPMS_PRESET_TOYOTA_PMVC2X_FREQUENCY 433920000U

size_t tpms_preset_count(void);
const TPMSPresetDefinition* tpms_preset_get_by_index(size_t index);
const TPMSPresetDefinition* tpms_preset_get_by_name(const char* name);
const TPMSPresetDefinition* tpms_preset_get_default(void);
const char* tpms_preset_get_short_name(const char* name);
FuriHalSubGhzPreset tpms_preset_get_radio_preset(const char* name);
uint32_t tpms_preset_get_default_frequency(void);
bool tpms_preset_init_radio(
    SubGhzRadioPreset* preset,
    const char* preset_name,
    uint32_t frequency,
    uint8_t* preset_data,
    size_t preset_data_size);
