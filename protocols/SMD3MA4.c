#include "SMD3MA4.h"
#include <lib/toolbox/manchester_decoder.h>

#define TAG "TPMSProtocolSMD3MA4"

/**
SMD3MA4 Schrader TPMS used in Subaru.
Contributed by: RonNiles.

Also Schrader 3039 TPMS for Infiniti, Nissan, Renault.
Contributed by: MotorvateDIY.

Refer to https://github.com/JoeSc/Subaru-TPMS-Spoofing

SCHRADER 3039 TPMS for Infiniti Nissan Renault (407001AY0A) (40700JY00B ?)
- https://catalogue.schradertcom/de-DE/ProductDetails/3039.html
- https://catalogue.schradertpms.com/en-GB/ProductDetails/3039.html
- Art.-Nr. 3039
- OE Art.-Nr: 407001AY0A
- EAN-Code: 5054208000275
- INFINITI, NISSAN, RENAULT (407001AY0A)

Used with:
- Nissan 370Z Z34 until 06/2014
- Infiniti FX until 12/2013
- Infiniti EX P53B (from 2007-10 until 2016-03)
- Infiniti FX (LCV) P53C (from 2008-03 until 2014-08)
- Infiniti FX P53C (from 2008-03 until 2014-08)
- Infiniti G L53A (from 2006-08 until 2013-03)
- Renault Koleos H45 (from 2008-02 until 2013-12)

Data layout:

    ^^^^_^_^_^_^_^_^_^_^_^_^_^_^_^_^^^^_FFFFFFIIIIIIIIIIIII
    IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIPPPPPPPPPPPPPPPPCCCC

- PREAMBLE: 36-bits 0xF5555555E
- F: FLAGS, 3 Manchester encoded bits
- I: ID, 24 Manchester encoded bits
- P: PRESSURE, 8 Manchester encoded bits (PSI * 5)
- C: CHECK, 2 Manchester encoded bits some kind of Parity

NOTE: there is NO temperature data transmitted
TODO: the checksum is unknown

We use OOK_PULSE_PCM to get the bitstream above.
Then we use bitbuffer_manchester_decode() which will alert us to any
bit sequence which is not a valid Manchester transition. This enables a sanity
check on the Manchester pulses which is important for detecting possible
corruption since there is no CRC.

The Manchester bits are encoded as 01 => 0 and 10 => 1, which is
the reverse of bitbuffer_manchester_decode(), so we invert the result.

Example payloads:

    {37}0000000030 {37}1000000020 {37}0800000028 {37}0400000020 {37}0200000028
    {37}0100000020 {37}0080000028 {37}0040000020 {37}0020000028 {37}0010000020
    {37}0008000028 {37}0004000020 {37}0002000028 {37}1400000030 {37}0a00000020
    {37}698e08eb48 {37}698e08ec68 {37}698e08ee60 {37}698e08edf0 {37}098e08edb8
    {37}098e08eca8 {37}098e08eb88 {37}098e08eb78 {37}098e08eb40 {37}098e08eb28
    {37}098e08eae0 {37}098e08eac8 {37}098e08eab0 {37}098e08ea98 {37}098e08ea68
    {37}098e08e8d0 {37}098e08e8b8 {37}098e08e880 {37}098e08e660 {37}098e08e3f8
    {37}698e08e2a0 {37}698e08e1e8 {37}098e08e028 {37}099b56e028 {37}099798e038

*/
#define PREAMBLE_BITS_LEN 3

static const SubGhzBlockConst tpms_protocol_SMD3MA4_const = {
    .te_short = 120,
    .te_long = 240,
    .te_delta = 55, // 50% of te_short due to poor sensitivity
    .min_count_bit_for_found = 33,
};

struct TPMSProtocolDecoderSMD3MA4 {
    SubGhzProtocolDecoderBase base;

    SubGhzBlockDecoder decoder;
    TPMSBlockGeneric generic;

    ManchesterState manchester_saved_state;
    uint16_t header_count;
};

struct TPMSProtocolEncoderSMD3MA4 {
    SubGhzProtocolEncoderBase base;

    SubGhzProtocolBlockEncoder encoder;
    TPMSBlockGeneric generic;
};

typedef enum {
    SMD3MA4DecoderStepReset = 0,
    SMD3MA4DecoderStepCheckPreamble,
    SMD3MA4DecoderStepDecoderData,
    SMD3MA4DecoderStepSaveDuration,
    SMD3MA4DecoderStepCheckDuration,
} SMD3MA4DecoderStep;

const SubGhzProtocolDecoder tpms_protocol_SMD3MA4_decoder = {
    .alloc = tpms_protocol_decoder_SMD3MA4_alloc,
    .free = tpms_protocol_decoder_SMD3MA4_free,

    .feed = tpms_protocol_decoder_SMD3MA4_feed,
    .reset = tpms_protocol_decoder_SMD3MA4_reset,

    .get_hash_data = tpms_protocol_decoder_SMD3MA4_get_hash_data,
    .serialize = tpms_protocol_decoder_SMD3MA4_serialize,
    .deserialize = tpms_protocol_decoder_SMD3MA4_deserialize,
    .get_string = tpms_protocol_decoder_SMD3MA4_get_string,
};

const SubGhzProtocolEncoder tpms_protocol_SMD3MA4_encoder = {
    .alloc = NULL,
    .free = NULL,

    .deserialize = NULL,
    .stop = NULL,
    .yield = NULL,
};

const SubGhzProtocol tpms_protocol_SMD3MA4 = {
    .name = TPMS_PROTOCOL_SMD3MA4_NAME,
    .type = SubGhzProtocolTypeStatic,
    .flag = SubGhzProtocolFlag_433 | SubGhzProtocolFlag_315 | SubGhzProtocolFlag_AM |
            SubGhzProtocolFlag_Decodable,

    .decoder = &tpms_protocol_SMD3MA4_decoder,
    .encoder = &tpms_protocol_SMD3MA4_encoder,
};

void* tpms_protocol_decoder_SMD3MA4_alloc(SubGhzEnvironment* environment) {
    UNUSED(environment);
    TPMSProtocolDecoderSMD3MA4* instance = malloc(sizeof(TPMSProtocolDecoderSMD3MA4));
    instance->base.protocol = &tpms_protocol_SMD3MA4;
    instance->generic.protocol_name = instance->base.protocol->name;
    return instance;
}

void tpms_protocol_decoder_SMD3MA4_free(void* context) {
    furi_assert(context);
    TPMSProtocolDecoderSMD3MA4* instance = context;
    free(instance);
}

void tpms_protocol_decoder_SMD3MA4_reset(void* context) {
    furi_assert(context);
    TPMSProtocolDecoderSMD3MA4* instance = context;
    instance->decoder.parser_step = SMD3MA4DecoderStepReset;
}

// static bool tpms_protocol_SMD3MA4_check_crc(TPMSProtocolDecoderSMD3MA4* instance) {
//     Check latest 2 parity bits
//     return (crc == (instance->decoder.decode_data & 0xFF));
// }

/**
 * Analysis of received data
 * @param instance Pointer to a TPMSBlockGeneric* instance
 */
static void tpms_protocol_SMD3MA4_analyze(TPMSBlockGeneric* instance) {
    instance->id = instance->data >> 24;

    // TODO locate and fix
    instance->battery_low = TPMS_NO_BATT;

    instance->temperature = ((instance->data >> 6) & 0xFF);
    instance->pressure = (((instance->data >> 5) & 0xFF) - 32) * 0.2 * 0.069;
}

static ManchesterEvent level_and_duration_to_event(bool level, uint32_t duration) {
    bool is_long = false;

    if(DURATION_DIFF(duration, tpms_protocol_SMD3MA4_const.te_long) <
       tpms_protocol_SMD3MA4_const.te_delta) {
        is_long = true;
    } else if(
        DURATION_DIFF(duration, tpms_protocol_SMD3MA4_const.te_short) <
        tpms_protocol_SMD3MA4_const.te_delta) {
        is_long = false;
    } else {
        return ManchesterEventReset;
    }

    if(level)
        return is_long ? ManchesterEventLongHigh : ManchesterEventShortHigh;
    else
        return is_long ? ManchesterEventLongLow : ManchesterEventShortLow;
}

void tpms_protocol_decoder_SMD3MA4_feed(void* context, bool level, uint32_t duration) {
    furi_assert(context);
    bool bit = false;
    bool have_bit = false;
    TPMSProtocolDecoderSMD3MA4* instance = context;

    // low-level bit sequence decoding
    if(instance->decoder.parser_step != SMD3MA4DecoderStepReset) {
        ManchesterEvent event = level_and_duration_to_event(level, duration);

        if(event == ManchesterEventReset) {
            if((instance->decoder.parser_step == SMD3MA4DecoderStepDecoderData) &&
               instance->decoder.decode_count_bit) {
                FURI_LOG_D(TAG, "%d-%ld", level, duration);
                FURI_LOG_D(
                    TAG,
                    "reset accumulated %d bits: %llx",
                    instance->decoder.decode_count_bit,
                    instance->decoder.decode_data);
            }

            instance->decoder.parser_step = SMD3MA4DecoderStepReset;
        } else {
            have_bit = manchester_advance(
                instance->manchester_saved_state, event, &instance->manchester_saved_state, &bit);
            if(!have_bit) return;

            // Invert value, due to signal is Manchester II and decoder is Manchester I
            bit = !bit;
        }
    }

    switch(instance->decoder.parser_step) {
    case SMD3MA4DecoderStepReset:
        // wait for start ~480us pulse
        if((level) && (DURATION_DIFF(duration, tpms_protocol_SMD3MA4_const.te_long * 2) <
                       tpms_protocol_SMD3MA4_const.te_delta)) {
            instance->decoder.parser_step = SMD3MA4DecoderStepCheckPreamble;
            instance->header_count = 0;
            instance->decoder.decode_data = 0;
            instance->decoder.decode_count_bit = 0;

            // First will be short space, so set correct initial state for machine
            // https://clearwater.com.au/images/rc5/rc5-state-machine.gif
            instance->manchester_saved_state = ManchesterStateStart1;
        }
        break;
    case SMD3MA4DecoderStepCheckPreamble:
        if(bit != 0) {
            instance->decoder.parser_step = SMD3MA4DecoderStepReset;
            break;
        }

        instance->header_count++;
        if(instance->header_count == PREAMBLE_BITS_LEN)
            instance->decoder.parser_step = SMD3MA4DecoderStepDecoderData;
        break;

    case SMD3MA4DecoderStepDecoderData:
        subghz_protocol_blocks_add_bit(&instance->decoder, bit);
        if(instance->decoder.decode_count_bit ==
           tpms_protocol_SMD3MA4_const.min_count_bit_for_found) {
            FURI_LOG_D(TAG, "%016llx", instance->decoder.decode_data);

            instance->generic.data = instance->decoder.decode_data;
            instance->generic.data_count_bit = instance->decoder.decode_count_bit;
            tpms_protocol_SMD3MA4_analyze(&instance->generic);
            if(instance->base.callback)
                instance->base.callback(&instance->base, instance->base.context);

            instance->decoder.parser_step = SMD3MA4DecoderStepReset;
        }
        break;
    }
}

uint8_t tpms_protocol_decoder_SMD3MA4_get_hash_data(void* context) {
    furi_assert(context);
    TPMSProtocolDecoderSMD3MA4* instance = context;
    return subghz_protocol_blocks_get_hash_data(
        &instance->decoder, (instance->decoder.decode_count_bit / 8) + 1);
}

SubGhzProtocolStatus tpms_protocol_decoder_SMD3MA4_serialize(
    void* context,
    FlipperFormat* flipper_format,
    SubGhzRadioPreset* preset) {
    furi_assert(context);
    TPMSProtocolDecoderSMD3MA4* instance = context;
    return tpms_block_generic_serialize(&instance->generic, flipper_format, preset);
}

SubGhzProtocolStatus
    tpms_protocol_decoder_SMD3MA4_deserialize(void* context, FlipperFormat* flipper_format) {
    furi_assert(context);
    TPMSProtocolDecoderSMD3MA4* instance = context;
    return tpms_block_generic_deserialize_check_count_bit(
        &instance->generic, flipper_format, tpms_protocol_SMD3MA4_const.min_count_bit_for_found);
}

void tpms_protocol_decoder_SMD3MA4_get_string(void* context, FuriString* output) {
    furi_assert(context);
    TPMSProtocolDecoderSMD3MA4* instance = context;
    furi_string_printf(
        output,
        "%s\r\n"
        "Id:0x%08lX\r\n"
        "Bat:%d\r\n"
        "Temp:%2.0f C Bar:%2.1f",
        instance->generic.protocol_name,
        instance->generic.id,
        instance->generic.battery_low,
        (double)instance->generic.temperature,
        (double)instance->generic.pressure);
}
