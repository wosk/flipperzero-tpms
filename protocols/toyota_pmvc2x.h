#pragma once

#include <lib/subghz/protocols/base.h>
#include <lib/subghz/blocks/const.h>
#include <lib/subghz/blocks/decoder.h>
#include <lib/subghz/blocks/encoder.h>

#include "toyota_pmvc2x_core.h"
#include "tpms_generic.h"

#define TPMS_PROTOCOL_TOYOTA_PMVC2X_NAME "Toyota PMV-C2x"

typedef struct TPMSProtocolDecoderToyotaPMVC2X TPMSProtocolDecoderToyotaPMVC2X;
typedef struct TPMSProtocolEncoderToyotaPMVC2X TPMSProtocolEncoderToyotaPMVC2X;

extern const SubGhzProtocolDecoder tpms_protocol_toyota_pmvc2x_decoder;
extern const SubGhzProtocolEncoder tpms_protocol_toyota_pmvc2x_encoder;
extern const SubGhzProtocol tpms_protocol_toyota_pmvc2x;

void* tpms_protocol_decoder_toyota_pmvc2x_alloc(SubGhzEnvironment* environment);
void tpms_protocol_decoder_toyota_pmvc2x_free(void* context);
void tpms_protocol_decoder_toyota_pmvc2x_reset(void* context);
void tpms_protocol_decoder_toyota_pmvc2x_feed(void* context, bool level, uint32_t duration);
uint8_t tpms_protocol_decoder_toyota_pmvc2x_get_hash_data(void* context);
SubGhzProtocolStatus tpms_protocol_decoder_toyota_pmvc2x_serialize(
    void* context,
    FlipperFormat* flipper_format,
    SubGhzRadioPreset* preset);
SubGhzProtocolStatus tpms_protocol_decoder_toyota_pmvc2x_deserialize(
    void* context,
    FlipperFormat* flipper_format);
void tpms_protocol_decoder_toyota_pmvc2x_get_string(void* context, FuriString* output);
