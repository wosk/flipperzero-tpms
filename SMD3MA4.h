#pragma once

#include <lib/subghz/protocols/base.h>

#include <lib/subghz/blocks/const.h>
#include <lib/subghz/blocks/decoder.h>
#include <lib/subghz/blocks/encoder.h>
#include "tpms_generic.h"
#include <lib/subghz/blocks/math.h>

#define TPMS_PROTOCOL_SMD3MA4_NAME "SMD3MA4"

typedef struct TPMSProtocolDecoderSMD3MA4 TPMSProtocolDecoderSMD3MA4;
typedef struct TPMSProtocolEncoderSMD3MA4 TPMSProtocolEncoderSMD3MA4;

extern const SubGhzProtocolDecoder tpms_protocol_SMD3MA4_decoder;
extern const SubGhzProtocolEncoder tpms_protocol_SMD3MA4_encoder;
extern const SubGhzProtocol tpms_protocol_SMD3MA4;

/**
 * Allocate TPMSProtocolDecoderSMD3MA4.
 * @param environment Pointer to a SubGhzEnvironment instance
 * @return TPMSProtocolDecoderSMD3MA4* pointer to a TPMSProtocolDecoderSMD3MA4 instance
 */
void* tpms_protocol_decoder_SMD3MA4_alloc(SubGhzEnvironment* environment);

/**
 * Free TPMSProtocolDecoderSMD3MA4.
 * @param context Pointer to a TPMSProtocolDecoderSMD3MA4 instance
 */
void tpms_protocol_decoder_SMD3MA4_free(void* context);

/**
 * Reset decoder TPMSProtocolDecoderSMD3MA4.
 * @param context Pointer to a TPMSProtocolDecoderSMD3MA4 instance
 */
void tpms_protocol_decoder_SMD3MA4_reset(void* context);

/**
 * Parse a raw sequence of levels and durations received from the air.
 * @param context Pointer to a TPMSProtocolDecoderSMD3MA4 instance
 * @param level Signal level true-high false-low
 * @param duration Duration of this level in, us
 */
void tpms_protocol_decoder_SMD3MA4_feed(void* context, bool level, uint32_t duration);

/**
 * Getting the hash sum of the last randomly received parcel.
 * @param context Pointer to a TPMSProtocolDecoderSMD3MA4 instance
 * @return hash Hash sum
 */
uint8_t tpms_protocol_decoder_SMD3MA4_get_hash_data(void* context);

/**
 * Serialize data TPMSProtocolDecoderSMD3MA4.
 * @param context Pointer to a TPMSProtocolDecoderSMD3MA4 instance
 * @param flipper_format Pointer to a FlipperFormat instance
 * @param preset The modulation on which the signal was received, SubGhzRadioPreset
 * @return status
 */
SubGhzProtocolStatus tpms_protocol_decoder_SMD3MA4_serialize(
    void* context,
    FlipperFormat* flipper_format,
    SubGhzRadioPreset* preset);

/**
 * Deserialize data TPMSProtocolDecoderSMD3MA4.
 * @param context Pointer to a TPMSProtocolDecoderSMD3MA4 instance
 * @param flipper_format Pointer to a FlipperFormat instance
 * @return status
 */
SubGhzProtocolStatus
    tpms_protocol_decoder_SMD3MA4_deserialize(void* context, FlipperFormat* flipper_format);

/**
 * Getting a textual representation of the received data.
 * @param context Pointer to a TPMSProtocolDecoderSMD3MA4 instance
 * @param output Resulting text
 */
void tpms_protocol_decoder_SMD3MA4_get_string(void* context, FuriString* output);