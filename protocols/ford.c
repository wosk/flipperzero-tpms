#include "ford.h"
#include <lib/toolbox/manchester_decoder.h>

#define TAG "Ford"

/**
 * Help
 * https://github.com/merbanan/rtl_433/blob/master/src/devices/tpms_ford.c
 *
 * FSK 8 byte Manchester encoded TPMS with simple checksum.
 * Seen on Ford Fiesta, Focus, Kuga, Escape, Transit...
 *
 * Seen on 315.00 MHz (United States).
 *
 * Seen on 433.92 MHz.
 * Likely VDO-Sensors, Type "S180084730Z", built by "Continental Automotive GmbH".
 *
 * Typically a transmission is sent 4 times.  Sometimes the T/P values
 * differ (slightly) among those.
 *
 * Sensor has 3 modes:
 *   moving: while being driven
 *   atrest: once after stopping, and every 6h thereafter (for months)
 *   learn: 12 transmissions, caused by using learn tool
 *
 * Packet nibbles:
 *
 *     II II II II PP TT FF CC
 *
 * - I = ID
 * - P = Pressure, as PSI * 4
 * - T = Temperature, as C + 56, except:
 *       When 0x80 is on, value is not temperature, meaning the full 8
 *       bits is not temperature, and the lower 7 bits is also not
 *       temperature.  Pattern of low 7 bits in this case seems more like
 *       codepoints than a measurement.
 * - F = Flags:
 *       0x80 not seen
 *       0x40 ON for vehicle moving
 *         Is strongly correlated with 0x80 being set in TT
 *       0x20: 9th bit of pressure.  Seen on Transit very high pressure, otherwise not.
 *       0x10: not seen
 *
 *       0x08: ON for learn
 *       0x04: ON for moving (0x08 and 0x04 both OFF for at rest)
 *       0x02: ~always NOT 0x01 (meaning of 0x3 not understood, but MOVING
 *             tends to have 0x02)
 *       0x01: about 19% of samples
 * - C = Checksum, SUM bytes 0 to 6 = byte 7
 */

#define PREAMBLE_PATTERN \
    {                    \
        0xaa, 0xa9       \
    }
#define PREAMBLE_BITS_LEN 16

static const SubGhzBlockConst tpms_protocol_ford_const = {
    .te_short = 52,
    .te_long = 104,
    .te_delta = 150,
    .min_count_bit_for_found = 64,
};

struct TPMSProtocolDecoderFord
{
    SubGhzProtocolDecoderBase base;

    SubGhzBlockDecoder decoder;
    TPMSBlockGeneric generic;

    ManchesterState manchester_saved_state;
    uint16_t header_count;
};

struct TPMSProtocolEncoderFord
{
    SubGhzProtocolEncoderBase base;

    SubGhzProtocolBlockEncoder encoder;
    TPMSBlockGeneric generic;
};

typedef enum
{
    FordDecoderStepReset = 0,
    FordDecoderStepCheckPreamble,
    FordDecoderStepDecoderData,
    FordDecoderStepSaveDuration,
    FordDecoderStepCheckDuration,
} FordDecoderStep;

const SubGhzProtocolDecoder tpms_protocol_ford_decoder = {
    .alloc = tpms_protocol_decoder_ford_alloc,
    .free = tpms_protocol_decoder_ford_free,

    .feed = tpms_protocol_decoder_ford_feed,
    .reset = tpms_protocol_decoder_ford_reset,

    .get_hash_data = tpms_protocol_decoder_ford_get_hash_data,
    .serialize = tpms_protocol_decoder_ford_serialize,
    .deserialize = tpms_protocol_decoder_ford_deserialize,
    .get_string = tpms_protocol_decoder_ford_get_string,
};

const SubGhzProtocolEncoder tpms_protocol_ford_encoder = {
    .alloc = NULL,
    .free = NULL,

    .deserialize = NULL,
    .stop = NULL,
    .yield = NULL,
};

const SubGhzProtocol tpms_protocol_ford = {
    .name = TPMS_PROTOCOL_FORD_NAME,
    .type = SubGhzProtocolTypeStatic,
    .flag = SubGhzProtocolFlag_433 | SubGhzProtocolFlag_315 | SubGhzProtocolFlag_FM |
            SubGhzProtocolFlag_Decodable,

    .decoder = &tpms_protocol_ford_decoder,
    .encoder = &tpms_protocol_ford_encoder,
};

void *tpms_protocol_decoder_ford_alloc(SubGhzEnvironment *environment)
{
    UNUSED(environment);
    TPMSProtocolDecoderFord *instance = malloc(sizeof(TPMSProtocolDecoderFord));
    instance->base.protocol = &tpms_protocol_ford;
    instance->generic.protocol_name = instance->base.protocol->name;
    return instance;
}

void tpms_protocol_decoder_ford_free(void *context)
{
    furi_assert(context);
    TPMSProtocolDecoderFord *instance = context;
    free(instance);
}

void tpms_protocol_decoder_ford_reset(void *context)
{
    furi_assert(context);
    TPMSProtocolDecoderFord *instance = context;
    instance->decoder.parser_step = FordDecoderStepReset;
}

static bool tpms_protocol_ford_check_crc(TPMSProtocolDecoderFord *instance)
{
    if (!instance->decoder.decode_data)
        return false;

    uint8_t *b = (uint8_t *)&instance->decoder.decode_data;
    FURI_LOG_D(TAG, "checksum in data from decoder: %02x", b[7]);

    uint8_t checksum = (b[0] + b[1] + b[2] + b[3] + b[4] + b[5] + b[6]) & 0xFF;
    FURI_LOG_D(TAG, "calculated checksum: %02x", checksum);

    return (checksum == b[7]);
}

/**
 * Analysis of received data
 * @param instance Pointer to a TPMSBlockGeneric* instance
 */
static void tpms_protocol_ford_analyze(TPMSBlockGeneric *instance)
{
    // TODO
    instance->id = instance->data >> 32;

    instance->battery_low = TPMS_NO_BATT;

    instance->temperature = ((instance->data >> 24) & 0xFF) - 56;
    instance->pressure = ((instance->data >> 8) & 0xFF) * 0.25f * 0.069;
}

static ManchesterEvent level_and_duration_to_event(bool level, uint32_t duration)
{
    bool is_long = false;

    if (DURATION_DIFF(duration, tpms_protocol_ford_const.te_long) <
        tpms_protocol_ford_const.te_delta)
    {
        is_long = true;
    }
    else if (
        DURATION_DIFF(duration, tpms_protocol_ford_const.te_short) <
        tpms_protocol_ford_const.te_delta)
    {
        is_long = false;
    }
    else
    {
        return ManchesterEventReset;
    }

    if (level)
        return is_long ? ManchesterEventLongHigh : ManchesterEventShortHigh;
    else
        return is_long ? ManchesterEventLongLow : ManchesterEventShortLow;
}

// ...

void tpms_protocol_decoder_ford_feed(void *context, bool level, uint32_t duration)
{
    // TODO
    furi_assert(context);
    TPMSProtocolDecoderFord *instance = context;
    bool bit = false;
    bool have_bit = false;

    // low-level bit sequence decoding
    if (instance->decoder.parser_step != FordDecoderStepReset)
    {
        ManchesterEvent event = level_and_duration_to_event(level, duration);

        if (event == ManchesterEventReset)
        {
            if ((instance->decoder.parser_step == FordDecoderStepDecoderData) &&
                instance->decoder.decode_count_bit)
            {
                // FURI_LOG_D(TAG, "%d-%ld", level, duration);
                FURI_LOG_D(
                    TAG,
                    "reset accumulated %d bits: %llx",
                    instance->decoder.decode_count_bit,
                    instance->decoder.decode_data);
            }

            instance->decoder.parser_step = FordDecoderStepReset;
        }
        else
        {
            have_bit = manchester_advance(
                instance->manchester_saved_state, event, &instance->manchester_saved_state, &bit);
            if (!have_bit)
                return;

            // Invert value, due to signal is Manchester II and decoder is Manchester I
            bit = !bit;
        }
    }

    switch (instance->decoder.parser_step)
    {
    case FordDecoderStepReset:
        if ((!level) && (DURATION_DIFF(duration, tpms_protocol_ford_const.te_long * 2) <
                         tpms_protocol_ford_const.te_delta))
        {
            instance->decoder.parser_step = FordDecoderStepCheckPreamble;
            instance->decoder.decode_data = 0;
            instance->decoder.decode_count_bit = 0;
            instance->manchester_saved_state = ManchesterStateStart1; // Initialize Manchester state
        }
        break;

    case FordDecoderStepCheckPreamble:
        if (bit != 0)
        {
            instance->decoder.parser_step = FordDecoderStepReset;
            break;
        }

        instance->header_count++;
        if (instance->header_count == PREAMBLE_BITS_LEN)
        {
            instance->decoder.parser_step = FordDecoderStepDecoderData;
        }
        break;

    case FordDecoderStepDecoderData:
        subghz_protocol_blocks_add_bit(&instance->decoder, bit);
        if (instance->decoder.decode_count_bit ==
            tpms_protocol_ford_const.min_count_bit_for_found)
        {
            FURI_LOG_D(TAG, "%016llx", instance->decoder.decode_data);
            if (!tpms_protocol_ford_check_crc(instance))
            {
                FURI_LOG_D(TAG, "CRC mismatch drop");
            }
            else
            {
                instance->generic.data = instance->decoder.decode_data;
                instance->generic.data_count_bit = instance->decoder.decode_count_bit;
                tpms_protocol_ford_analyze(&instance->generic);
                if (instance->base.callback)
                {
                    instance->base.callback(&instance->base, instance->base.context);
                }
            }
            instance->decoder.parser_step = FordDecoderStepReset;
        }
        break;
    }
}

uint8_t tpms_protocol_decoder_ford_get_hash_data(void *context)
{
    furi_assert(context);
    TPMSProtocolDecoderFord *instance = context;
    return subghz_protocol_blocks_get_hash_data(
        &instance->decoder, (instance->decoder.decode_count_bit / 8) + 1);
}

SubGhzProtocolStatus tpms_protocol_decoder_ford_serialize(
    void *context,
    FlipperFormat *flipper_format,
    SubGhzRadioPreset *preset)
{
    furi_assert(context);
    TPMSProtocolDecoderFord *instance = context;
    return tpms_block_generic_serialize(&instance->generic, flipper_format, preset);
}

SubGhzProtocolStatus
tpms_protocol_decoder_ford_deserialize(void *context, FlipperFormat *flipper_format)
{
    furi_assert(context);
    TPMSProtocolDecoderFord *instance = context;
    return tpms_block_generic_deserialize_check_count_bit(
        &instance->generic, flipper_format, tpms_protocol_ford_const.min_count_bit_for_found);
}

void tpms_protocol_decoder_ford_get_string(void *context, FuriString *output)
{
    // TODO
    furi_assert(context);
    TPMSProtocolDecoderFord *instance = context;
    furi_string_printf(
        output,
        "%s\r\n"
        "Id:0x^%08lX\r\n"
        "Bat:%d\r\n"
        "Temp:%2.0f C Bar:%2.1f",
        instance->generic.protocol_name,
        instance->generic.id,
        instance->generic.battery_low,
        (double)instance->generic.temperature,
        (double)instance->generic.pressure);
}
