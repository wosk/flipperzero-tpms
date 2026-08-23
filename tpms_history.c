#include "tpms_history.h"
#include <flipper_format/flipper_format_i.h>
#include <lib/toolbox/stream/stream.h>
#include <lib/subghz/receiver.h>
#include "protocols/tpms_generic.h"
#include "protocols/toyota_pmvc2x.h"

#include <furi.h>
#include <string.h>

#define TPMS_HISTORY_MAX 50
#define TAG "TPMSHistory"

typedef struct {
    FuriString* item_str;
    FuriString* type_label;
    FuriString* id_label;
    FlipperFormat* flipper_string;
    uint8_t type;
    uint32_t id;
    SubGhzRadioPreset* preset;
} TPMSHistoryItem;

ARRAY_DEF(TPMSHistoryItemArray, TPMSHistoryItem, M_POD_OPLIST)

#define M_OPL_TPMSHistoryItemArray_t() ARRAY_OPLIST(TPMSHistoryItemArray, M_POD_OPLIST)

typedef struct {
    TPMSHistoryItemArray_t data;
} TPMSHistoryStruct;

struct TPMSHistory {
    uint32_t last_update_timestamp;
    uint16_t last_index_write;
    uint8_t code_last_hash_data;
    FuriString* tmp_string;
    TPMSHistoryStruct* history;
};

static void tpms_history_build_labels(TPMSHistory* instance, TPMSHistoryItem* item) {
    furi_assert(instance);
    furi_assert(item);

    furi_string_reset(instance->tmp_string);
    if(flipper_format_rewind(item->flipper_string) &&
       flipper_format_read_string(item->flipper_string, "Protocol", instance->tmp_string)) {
        if(strcmp(furi_string_get_cstr(instance->tmp_string), TPMS_PROTOCOL_TOYOTA_PMVC2X_NAME) == 0) {
            furi_string_set_str(item->type_label, "Toyota-C2x");
        } else {
            furi_string_set(item->type_label, instance->tmp_string);
        }
    } else {
        furi_string_set_str(item->type_label, "TPMS");
    }

    furi_string_printf(item->id_label, "%08lX", (unsigned long)item->id);
    furi_string_printf(
        item->item_str,
        "%s %s",
        furi_string_get_cstr(item->type_label),
        furi_string_get_cstr(item->id_label));
}

static void tpms_history_move_item_to_front(TPMSHistory* instance, size_t index) {
    furi_assert(instance);
    if(index == 0 || index >= TPMSHistoryItemArray_size(instance->history->data)) {
        return;
    }

    TPMSHistoryItem item = *TPMSHistoryItemArray_get(instance->history->data, index);
    for(size_t i = index; i > 0; i--) {
        *TPMSHistoryItemArray_get(instance->history->data, i) =
            *TPMSHistoryItemArray_get(instance->history->data, i - 1);
    }
    *TPMSHistoryItemArray_get(instance->history->data, 0) = item;
}

TPMSHistory* tpms_history_alloc(void) {
    TPMSHistory* instance = malloc(sizeof(TPMSHistory));
    instance->tmp_string = furi_string_alloc();
    instance->history = malloc(sizeof(TPMSHistoryStruct));
    TPMSHistoryItemArray_init(instance->history->data);
    return instance;
}

void tpms_history_free(TPMSHistory* instance) {
    furi_assert(instance);
    furi_string_free(instance->tmp_string);
    for
        M_EACH(item, instance->history->data, TPMSHistoryItemArray_t) {
            furi_string_free(item->item_str);
            furi_string_free(item->type_label);
            furi_string_free(item->id_label);
            furi_string_free(item->preset->name);
            free(item->preset);
            flipper_format_free(item->flipper_string);
            item->type = 0;
        }
    TPMSHistoryItemArray_clear(instance->history->data);
    free(instance->history);
    free(instance);
}

uint32_t tpms_history_get_frequency(TPMSHistory* instance, uint16_t idx) {
    furi_assert(instance);
    TPMSHistoryItem* item = TPMSHistoryItemArray_get(instance->history->data, idx);
    return item->preset->frequency;
}

SubGhzRadioPreset* tpms_history_get_radio_preset(TPMSHistory* instance, uint16_t idx) {
    furi_assert(instance);
    TPMSHistoryItem* item = TPMSHistoryItemArray_get(instance->history->data, idx);
    return item->preset;
}

const char* tpms_history_get_preset(TPMSHistory* instance, uint16_t idx) {
    furi_assert(instance);
    TPMSHistoryItem* item = TPMSHistoryItemArray_get(instance->history->data, idx);
    return furi_string_get_cstr(item->preset->name);
}

void tpms_history_reset(TPMSHistory* instance) {
    furi_assert(instance);
    furi_string_reset(instance->tmp_string);
    for
        M_EACH(item, instance->history->data, TPMSHistoryItemArray_t) {
            furi_string_free(item->item_str);
            furi_string_free(item->type_label);
            furi_string_free(item->id_label);
            furi_string_free(item->preset->name);
            free(item->preset);
            flipper_format_free(item->flipper_string);
            item->type = 0;
        }
    TPMSHistoryItemArray_reset(instance->history->data);
    instance->last_index_write = 0;
    instance->code_last_hash_data = 0;
}

uint16_t tpms_history_get_item(TPMSHistory* instance) {
    furi_assert(instance);
    return instance->last_index_write;
}

uint8_t tpms_history_get_type_protocol(TPMSHistory* instance, uint16_t idx) {
    furi_assert(instance);
    TPMSHistoryItem* item = TPMSHistoryItemArray_get(instance->history->data, idx);
    return item->type;
}

const char* tpms_history_get_protocol_name(TPMSHistory* instance, uint16_t idx) {
    furi_assert(instance);
    TPMSHistoryItem* item = TPMSHistoryItemArray_get(instance->history->data, idx);
    flipper_format_rewind(item->flipper_string);
    if(!flipper_format_read_string(item->flipper_string, "Protocol", instance->tmp_string)) {
        FURI_LOG_E(TAG, "Missing Protocol");
        furi_string_reset(instance->tmp_string);
    }
    return furi_string_get_cstr(instance->tmp_string);
}

FlipperFormat* tpms_history_get_raw_data(TPMSHistory* instance, uint16_t idx) {
    furi_assert(instance);
    TPMSHistoryItem* item = TPMSHistoryItemArray_get(instance->history->data, idx);
    if(item->flipper_string) {
        return item->flipper_string;
    } else {
        return NULL;
    }
}
bool tpms_history_get_text_space_left(TPMSHistory* instance, FuriString* output) {
    furi_assert(instance);
    if(instance->last_index_write == TPMS_HISTORY_MAX) {
        if(output != NULL) furi_string_printf(output, "Memory is FULL");
        return true;
    }
    if(output != NULL)
        furi_string_printf(output, "%02u/%02u", instance->last_index_write, TPMS_HISTORY_MAX);
    return false;
}

void tpms_history_get_text_item_menu(TPMSHistory* instance, FuriString* output, uint16_t idx) {
    TPMSHistoryItem* item = TPMSHistoryItemArray_get(instance->history->data, idx);
    furi_string_set(output, item->item_str);
}

const char* tpms_history_get_type_label(TPMSHistory* instance, uint16_t idx) {
    furi_assert(instance);
    TPMSHistoryItem* item = TPMSHistoryItemArray_get(instance->history->data, idx);
    return furi_string_get_cstr(item->type_label);
}

const char* tpms_history_get_id_label(TPMSHistory* instance, uint16_t idx) {
    furi_assert(instance);
    TPMSHistoryItem* item = TPMSHistoryItemArray_get(instance->history->data, idx);
    return furi_string_get_cstr(item->id_label);
}

TPMSHistoryStateAddKey
    tpms_history_add_to_history(TPMSHistory* instance, void* context, SubGhzRadioPreset* preset) {
    furi_assert(instance);
    furi_assert(context);

    if(instance->last_index_write >= TPMS_HISTORY_MAX) return TPMSHistoryStateAddKeyOverflow;

    SubGhzProtocolDecoderBase* decoder_base = context;
    if((instance->code_last_hash_data ==
        subghz_protocol_decoder_base_get_hash_data(decoder_base)) &&
       ((furi_get_tick() - instance->last_update_timestamp) < 500)) {
        instance->last_update_timestamp = furi_get_tick();
        return TPMSHistoryStateAddKeyTimeOut;
    }

    instance->code_last_hash_data = subghz_protocol_decoder_base_get_hash_data(decoder_base);
    instance->last_update_timestamp = furi_get_tick();

    FlipperFormat* fff = flipper_format_string_alloc();
    uint32_t id = 0;
    subghz_protocol_decoder_base_serialize(decoder_base, fff, preset);

    do {
        if(!flipper_format_rewind(fff)) {
            FURI_LOG_E(TAG, "Rewind error");
            break;
        }
        if(!flipper_format_read_uint32(fff, "Id", (uint32_t*)&id, 1)) {
            FURI_LOG_E(TAG, "Missing Id");
            break;
        }
    } while(false);
    flipper_format_free(fff);

    // Update record if found
    bool sensor_found = false;
    for(size_t i = 0; i < TPMSHistoryItemArray_size(instance->history->data); i++) {
        TPMSHistoryItem* item = TPMSHistoryItemArray_get(instance->history->data, i);
        if(item->id == id) {
            sensor_found = true;
            Stream* flipper_string_stream = flipper_format_get_raw_stream(item->flipper_string);
            stream_clean(flipper_string_stream);
            subghz_protocol_decoder_base_serialize(decoder_base, item->flipper_string, preset);
            item->preset->frequency = preset->frequency;
            furi_string_set(item->preset->name, preset->name);
            item->preset->data = preset->data;
            item->preset->data_size = preset->data_size;
            tpms_history_build_labels(instance, item);
            tpms_history_move_item_to_front(instance, i);
            return TPMSHistoryStateAddKeyUpdateData;
        }
    }

    // or add new record
    if(!sensor_found) { //-V547
        TPMSHistoryItem* item = TPMSHistoryItemArray_push_raw(instance->history->data);
        item->preset = malloc(sizeof(SubGhzRadioPreset));
        item->type = decoder_base->protocol->type;
        item->preset->frequency = preset->frequency;
        item->preset->name = furi_string_alloc();
        furi_string_set(item->preset->name, preset->name);
        item->preset->data = preset->data;
        item->preset->data_size = preset->data_size;
        item->id = id;

        item->item_str = furi_string_alloc();
        item->type_label = furi_string_alloc();
        item->id_label = furi_string_alloc();
        item->flipper_string = flipper_format_string_alloc();
        subghz_protocol_decoder_base_serialize(decoder_base, item->flipper_string, preset);
        tpms_history_build_labels(instance, item);
        tpms_history_move_item_to_front(
            instance, TPMSHistoryItemArray_size(instance->history->data) - 1);
        instance->last_index_write++;
        return TPMSHistoryStateAddKeyNewDada;
    }
    return TPMSHistoryStateAddKeyUnknown;
}
