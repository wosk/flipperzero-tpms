#pragma once

#include <lib/subghz/protocols/base.h>

#include <lib/subghz/blocks/const.h>
#include <lib/subghz/blocks/decoder.h>
#include <lib/subghz/blocks/encoder.h>
#include "tpms_generic.h"
#include <lib/subghz/blocks/math.h>

#define TPMS_PROTOCOL_FORD_NAME "Ford TPMS"

typedef struct TPMSProtocolDecoderFord TPMSProtocolDecoderFord;
typedef struct TPMSProtocolEncoderFord TPMSProtocolEncoderFord;

extern const SubGhzProtocolDecoder tpms_protocol_ford_decoder;
extern const SubGhzProtocolEncoder tpms_protocol_ford_encoder;
extern const SubGhzProtocol tpms_protocol_ford;

/**
 * Allocate TPMSProtocolDecoderFord.
 * @param environment Pointer to a SubGhzEnvironment instance
 * @return TPMSProtocolDecoderFord* pointer to a TPMSProtocolDecoderFord instance
 */
void *tpms_protocol_decoder_ford_alloc(SubGhzEnvironment *environment);

/**
 * Free TPMSProtocolDecoderFord.
 * @param context Pointer to a TPMSProtocolDecoderFord instance
 */
void tpms_protocol_decoder_ford_free(void *context);

/**
 * Reset decoder TPMSProtocolDecoderFord.
 * @param context Pointer to a TPMSProtocolDecoderFord instance
 */
void tpms_protocol_decoder_ford_reset(void *context);

/**
 * Parse a raw sequence of levels and durations received from the air.
 * @param context Pointer to a TPMSProtocolDecoderFord instance
 * @param level Signal level true-high false-low
 * @param duration Duration of this level in, us
 */
void tpms_protocol_decoder_ford_feed(void *context, bool level, uint32_t duration);

/**
 * Getting the hash sum of the last randomly received parcel.
 * @param context Pointer to a TPMSProtocolDecoderFord instance
 * @return hash Hash sum
 */
uint8_t tpms_protocol_decoder_ford_get_hash_data(void *context);

/**
 * Serialize data TPMSProtocolDecoderFord.
 * @param context Pointer to a TPMSProtocolDecoderFord instance
 * @param flipper_format Pointer to a FlipperFormat instance
 * @param preset The modulation on which the signal was received, SubGhzRadioPreset
 * @return status
 */
SubGhzProtocolStatus tpms_protocol_decoder_ford_serialize(
    void *context,
    FlipperFormat *flipper_format,
    SubGhzRadioPreset *preset);

/**
 * Deserialize data TPMSProtocolDecoderFord.
 * @param context Pointer to a TPMSProtocolDecoderFord instance
 * @param flipper_format Pointer to a FlipperFormat instance
 * @return status
 */
SubGhzProtocolStatus
tpms_protocol_decoder_ford_deserialize(void *context, FlipperFormat *flipper_format);

/**
 * Getting a textual representation of the received data.
 * @param context Pointer to a TPMSProtocolDecoderFord instance
 * @param output Resulting text
 */
void tpms_protocol_decoder_ford_get_string(void *context, FuriString *output);
