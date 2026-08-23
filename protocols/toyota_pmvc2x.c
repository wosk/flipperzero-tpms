#include "toyota_pmvc2x.h"

#include <stdlib.h>
#include <string.h>

#define TAG "Toyota"

#define TPMS_TOYOTA_PSI_TO_BAR 0.0689476f

static const SubGhzBlockConst tpms_protocol_toyota_pmvc2x_const = {
    .te_short = 48U,
    .te_long = 96U,
    .te_delta = 18U,
    .min_count_bit_for_found = 64U,
};

struct TPMSProtocolDecoderToyotaPMVC2X {
    SubGhzProtocolDecoderBase base;
    SubGhzBlockDecoder decoder;
    TPMSBlockGeneric generic;
    ToyotaPmvc2xCore core;
    uint8_t last_status;
};

struct TPMSProtocolEncoderToyotaPMVC2X {
    SubGhzProtocolEncoderBase base;
    SubGhzProtocolBlockEncoder encoder;
    TPMSBlockGeneric generic;
};

static void tpms_protocol_toyota_pmvc2x_apply_frame(
    TPMSProtocolDecoderToyotaPMVC2X* instance,
    const ToyotaPmvc2xFrame* frame) {
    float pressure_psi = ((float)frame->pressure_raw * 0.25f) - 7.0f;
    float pressure_bar = pressure_psi * TPMS_TOYOTA_PSI_TO_BAR;
    float temperature_c = (float)frame->temperature_raw - 40.0f;

    uint64_t data = 0;
    for(size_t index = 0; index < 8U; index++) {
        data = (data << 8U) | frame->payload[index];
    }

    instance->decoder.decode_data = data;
    instance->decoder.decode_count_bit = 64U;

    instance->generic.data = data;
    instance->generic.data_count_bit = 64U;
    instance->generic.id = frame->id;
    instance->generic.battery_low = TPMS_NO_BATT;
    instance->generic.temperature = temperature_c;
    instance->generic.pressure = pressure_bar;
    instance->last_status = frame->status;

    FURI_LOG_D(
        TAG,
        "id=%08lX status=0x%02X psi=%.2f temp=%.1f",
        (unsigned long)frame->id,
        frame->status,
        (double)pressure_psi,
        (double)temperature_c);
}

const SubGhzProtocolDecoder tpms_protocol_toyota_pmvc2x_decoder = {
    .alloc = tpms_protocol_decoder_toyota_pmvc2x_alloc,
    .free = tpms_protocol_decoder_toyota_pmvc2x_free,
    .feed = tpms_protocol_decoder_toyota_pmvc2x_feed,
    .reset = tpms_protocol_decoder_toyota_pmvc2x_reset,
    .get_hash_data = tpms_protocol_decoder_toyota_pmvc2x_get_hash_data,
    .serialize = tpms_protocol_decoder_toyota_pmvc2x_serialize,
    .deserialize = tpms_protocol_decoder_toyota_pmvc2x_deserialize,
    .get_string = tpms_protocol_decoder_toyota_pmvc2x_get_string,
};

const SubGhzProtocolEncoder tpms_protocol_toyota_pmvc2x_encoder = {
    .alloc = NULL,
    .free = NULL,
    .deserialize = NULL,
    .stop = NULL,
    .yield = NULL,
};

const SubGhzProtocol tpms_protocol_toyota_pmvc2x = {
    .name = TPMS_PROTOCOL_TOYOTA_PMVC2X_NAME,
    .type = SubGhzProtocolTypeStatic,
    .flag = SubGhzProtocolFlag_433 | SubGhzProtocolFlag_315 | SubGhzProtocolFlag_FM |
            SubGhzProtocolFlag_Decodable,
    .decoder = &tpms_protocol_toyota_pmvc2x_decoder,
    .encoder = &tpms_protocol_toyota_pmvc2x_encoder,
};

void* tpms_protocol_decoder_toyota_pmvc2x_alloc(SubGhzEnvironment* environment) {
    UNUSED(environment);
    TPMSProtocolDecoderToyotaPMVC2X* instance = malloc(sizeof(TPMSProtocolDecoderToyotaPMVC2X));
    instance->base.protocol = &tpms_protocol_toyota_pmvc2x;
    instance->generic.protocol_name = instance->base.protocol->name;
    tpms_protocol_decoder_toyota_pmvc2x_reset(instance);
    return instance;
}

void tpms_protocol_decoder_toyota_pmvc2x_free(void* context) {
    furi_assert(context);
    TPMSProtocolDecoderToyotaPMVC2X* instance = context;
    free(instance);
}

void tpms_protocol_decoder_toyota_pmvc2x_reset(void* context) {
    furi_assert(context);
    TPMSProtocolDecoderToyotaPMVC2X* instance = context;
    instance->decoder.parser_step = 0;
    instance->decoder.decode_count_bit = 0;
    instance->decoder.decode_data = 0;
    instance->decoder.te_last = 0;
    instance->last_status = 0;
    toyota_pmvc2x_core_reset(&instance->core);
}

void tpms_protocol_decoder_toyota_pmvc2x_feed(void* context, bool level, uint32_t duration) {
    furi_assert(context);
    TPMSProtocolDecoderToyotaPMVC2X* instance = context;

    if(toyota_pmvc2x_core_feed(&instance->core, level, duration)) {
        ToyotaPmvc2xFrame frame = {0};
        if(toyota_pmvc2x_core_take_frame(&instance->core, &frame)) {
            tpms_protocol_toyota_pmvc2x_apply_frame(instance, &frame);

            if(instance->base.callback) {
                instance->base.callback(&instance->base, instance->base.context);
            }
        }
    }
}

uint8_t tpms_protocol_decoder_toyota_pmvc2x_get_hash_data(void* context) {
    furi_assert(context);
    TPMSProtocolDecoderToyotaPMVC2X* instance = context;
    return subghz_protocol_blocks_get_hash_data(
        &instance->decoder, (instance->decoder.decode_count_bit / 8U) + 1U);
}

SubGhzProtocolStatus tpms_protocol_decoder_toyota_pmvc2x_serialize(
    void* context,
    FlipperFormat* flipper_format,
    SubGhzRadioPreset* preset) {
    furi_assert(context);
    TPMSProtocolDecoderToyotaPMVC2X* instance = context;
    return tpms_block_generic_serialize(&instance->generic, flipper_format, preset);
}

SubGhzProtocolStatus tpms_protocol_decoder_toyota_pmvc2x_deserialize(
    void* context,
    FlipperFormat* flipper_format) {
    furi_assert(context);
    TPMSProtocolDecoderToyotaPMVC2X* instance = context;
    return tpms_block_generic_deserialize_check_count_bit(
        &instance->generic,
        flipper_format,
        tpms_protocol_toyota_pmvc2x_const.min_count_bit_for_found);
}

void tpms_protocol_decoder_toyota_pmvc2x_get_string(void* context, FuriString* output) {
    furi_assert(context);
    TPMSProtocolDecoderToyotaPMVC2X* instance = context;
    furi_string_printf(
        output,
        "%s\r\nId:0x%08lX\r\nStatus:0x%02X\r\nBat:%d\r\nTemp:%2.0f C Bar:%2.1f",
        instance->generic.protocol_name,
        (unsigned long)instance->generic.id,
        instance->last_status,
        instance->generic.battery_low,
        (double)instance->generic.temperature,
        (double)instance->generic.pressure);
}
