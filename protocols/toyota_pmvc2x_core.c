#include "toyota_pmvc2x_core.h"

#include <string.h>

#define COUNT_OF(x) (sizeof(x) / sizeof((x)[0]))

#define TOYOTA_PMVC2X_HALF_BIT_US 48U
#define TOYOTA_PMVC2X_MAX_UNITS_PER_PULSE 6U
#define TOYOTA_PMVC2X_PATTERN_MIN_LEN 8U
#define TOYOTA_PMVC2X_PAYLOAD_BITS 72U
#define TOYOTA_PMVC2X_ENCODED_BITS (TOYOTA_PMVC2X_PAYLOAD_BITS * 2U)
#define TOYOTA_PMVC2X_MIN_FRAME_BITS (TOYOTA_PMVC2X_PATTERN_MIN_LEN + TOYOTA_PMVC2X_ENCODED_BITS)
#define TOYOTA_PMVC2X_GAP_THRESHOLD_US 320U
#define TOYOTA_PMVC2X_TOLERANCE_US 18U
#define TOYOTA_PMVC2X_TOLERANCE_EXTRA_US 12U

typedef struct {
    const uint8_t* bits;
    uint8_t length;
} ToyotaPmvc2xSyncPattern;

static const uint8_t toyota_pmvc2x_sync_pattern_full[] = {1, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 0};
static const uint8_t toyota_pmvc2x_sync_pattern_short0[] = {0, 0, 1, 1, 1, 1, 0, 0};
static const uint8_t toyota_pmvc2x_sync_pattern_short1[] = {0, 0, 1, 1, 1, 1, 1, 0, 0};
static const uint8_t toyota_pmvc2x_sync_pattern_short2[] = {0, 0, 1, 1, 1, 1, 0, 1};
static const uint8_t toyota_pmvc2x_sync_pattern_short3[] = {0, 0, 1, 1, 1, 1, 1, 0, 1};

static const ToyotaPmvc2xSyncPattern toyota_pmvc2x_sync_patterns[] = {
    {toyota_pmvc2x_sync_pattern_full, COUNT_OF(toyota_pmvc2x_sync_pattern_full)},
    {toyota_pmvc2x_sync_pattern_short0, COUNT_OF(toyota_pmvc2x_sync_pattern_short0)},
    {toyota_pmvc2x_sync_pattern_short1, COUNT_OF(toyota_pmvc2x_sync_pattern_short1)},
    {toyota_pmvc2x_sync_pattern_short2, COUNT_OF(toyota_pmvc2x_sync_pattern_short2)},
    {toyota_pmvc2x_sync_pattern_short3, COUNT_OF(toyota_pmvc2x_sync_pattern_short3)},
};

static uint32_t toyota_pmvc2x_core_diff_u32(uint32_t a, uint32_t b) {
    return (a > b) ? (a - b) : (b - a);
}

static void toyota_pmvc2x_core_reset_buffer(ToyotaPmvc2xCore* instance) {
    instance->raw_count = 0;
}

static uint8_t toyota_pmvc2x_core_bit_at(
    const ToyotaPmvc2xCore* instance,
    size_t index,
    bool invert) {
    uint8_t bit = instance->raw_bits[index];
    return invert ? (bit ^ 1U) : bit;
}

static bool toyota_pmvc2x_core_store_bit(ToyotaPmvc2xCore* instance, uint8_t bit) {
    if(instance->raw_count >= COUNT_OF(instance->raw_bits)) {
        return false;
    }

    instance->raw_bits[instance->raw_count++] = bit ? 1U : 0U;
    return true;
}

static bool toyota_pmvc2x_core_append_level(
    ToyotaPmvc2xCore* instance,
    bool level,
    uint32_t duration_us) {
    if(duration_us < (TOYOTA_PMVC2X_HALF_BIT_US / 2U)) {
        return true;
    }

    uint32_t count = (duration_us + (TOYOTA_PMVC2X_HALF_BIT_US / 2U)) / TOYOTA_PMVC2X_HALF_BIT_US;
    if(count == 0U) {
        count = 1U;
    }

    if(count > TOYOTA_PMVC2X_MAX_UNITS_PER_PULSE) {
        return false;
    }

    uint32_t expected = count * TOYOTA_PMVC2X_HALF_BIT_US;
    uint32_t tolerance = (TOYOTA_PMVC2X_TOLERANCE_US * count) + TOYOTA_PMVC2X_TOLERANCE_EXTRA_US;
    if(toyota_pmvc2x_core_diff_u32(duration_us, expected) > tolerance) {
        return false;
    }

    for(uint32_t index = 0; index < count; index++) {
        if(!toyota_pmvc2x_core_store_bit(instance, level ? 1U : 0U)) {
            return false;
        }
    }

    return true;
}

static bool toyota_pmvc2x_core_match_pattern(
    const ToyotaPmvc2xCore* instance,
    size_t offset,
    bool invert,
    const ToyotaPmvc2xSyncPattern* pattern) {
    if(offset + pattern->length > instance->raw_count) {
        return false;
    }

    for(size_t index = 0; index < pattern->length; index++) {
        if(toyota_pmvc2x_core_bit_at(instance, offset + index, invert) != pattern->bits[index]) {
            return false;
        }
    }

    return true;
}

static void toyota_pmvc2x_core_append_payload_bit(uint8_t* buffer, size_t* bit_count, uint8_t bit) {
    if(*bit_count >= TOYOTA_PMVC2X_PAYLOAD_BITS) {
        return;
    }

    size_t byte_index = *bit_count / 8U;
    size_t bit_index = *bit_count % 8U;
    if(bit_index == 0U) {
        buffer[byte_index] = 0;
    }

    buffer[byte_index] = (buffer[byte_index] << 1U) | (bit & 1U);
    (*bit_count)++;
}

static bool toyota_pmvc2x_core_diff_manchester(
    const ToyotaPmvc2xCore* instance,
    size_t start,
    bool invert,
    uint8_t* payload) {
    size_t len = instance->raw_count;
    size_t pos = start;
    uint8_t bit1 = 0;
    uint8_t bit2 = 0;
    size_t out_bits = 0;

    while((pos + 1U) < len && out_bits < TOYOTA_PMVC2X_PAYLOAD_BITS) {
        bit1 = toyota_pmvc2x_core_bit_at(instance, pos++, invert);
        if(pos >= len) {
            return false;
        }

        bit2 = toyota_pmvc2x_core_bit_at(instance, pos++, invert);
        if(pos >= len) {
            return false;
        }

        uint8_t bit3 = toyota_pmvc2x_core_bit_at(instance, pos, invert);

        if(bit1 != bit2) {
            if(bit2 != bit3) {
                toyota_pmvc2x_core_append_payload_bit(payload, &out_bits, 0U);
            } else {
                bit2 = bit1;
                pos -= 1U;
                break;
            }
        } else {
            bit2 = 1U - bit1;
            if(pos < 2U) {
                return false;
            }
            pos -= 2U;
            break;
        }
    }

    while((pos + 1U) < len && out_bits < TOYOTA_PMVC2X_PAYLOAD_BITS) {
        bit1 = toyota_pmvc2x_core_bit_at(instance, pos++, invert);
        if(bit1 == bit2) {
            break;
        }

        if(pos >= len) {
            return false;
        }

        bit2 = toyota_pmvc2x_core_bit_at(instance, pos++, invert);
        toyota_pmvc2x_core_append_payload_bit(payload, &out_bits, (bit1 == bit2) ? 1U : 0U);
    }

    return out_bits >= TOYOTA_PMVC2X_PAYLOAD_BITS;
}

static uint8_t toyota_pmvc2x_core_crc8(
    const uint8_t* message,
    size_t size,
    uint8_t polynomial,
    uint8_t init) {
    uint8_t remainder = init;

    for(size_t byte = 0; byte < size; byte++) {
        remainder ^= message[byte];
        for(uint8_t bit = 0; bit < 8U; bit++) {
            if(remainder & 0x80U) {
                remainder = (uint8_t)((remainder << 1U) ^ polynomial);
            } else {
                remainder = (uint8_t)(remainder << 1U);
            }
        }
    }

    return remainder;
}

static void toyota_pmvc2x_core_store_frame(ToyotaPmvc2xCore* instance, const uint8_t* payload) {
    ToyotaPmvc2xFrame* frame = &instance->frame;
    memcpy(frame->payload, payload, TOYOTA_PMVC2X_PAYLOAD_BYTES);
    frame->id = ((uint32_t)payload[0] << 24U) | ((uint32_t)payload[1] << 16U) |
                ((uint32_t)payload[2] << 8U) | (uint32_t)payload[3];
    frame->status = (uint8_t)((payload[4] & 0x80U) | (payload[6] & 0x7FU));
    frame->pressure_raw = (uint8_t)(((payload[4] & 0x7FU) << 1U) | (payload[5] >> 7U));
    frame->temperature_raw = (uint8_t)(((payload[5] & 0x7FU) << 1U) | (payload[6] >> 7U));
    frame->crc_ok = true;
    instance->frame_pending = true;
}

static bool toyota_pmvc2x_core_decode_from(
    ToyotaPmvc2xCore* instance,
    size_t start,
    bool invert) {
    if(start + TOYOTA_PMVC2X_ENCODED_BITS > instance->raw_count) {
        return false;
    }

    uint8_t payload[TOYOTA_PMVC2X_PAYLOAD_BYTES] = {0};
    if(!toyota_pmvc2x_core_diff_manchester(instance, start, invert, payload)) {
        return false;
    }

    if(toyota_pmvc2x_core_crc8(payload, 8U, 0x07U, 0x80U) != payload[8]) {
        return false;
    }

    uint8_t pressure_raw = (uint8_t)(((payload[4] & 0x7FU) << 1U) | (payload[5] >> 7U));
    uint8_t pressure_check = (uint8_t)(payload[7] ^ 0xFFU);
    if(pressure_raw != pressure_check) {
        return false;
    }

    toyota_pmvc2x_core_store_frame(instance, payload);
    return true;
}

static bool toyota_pmvc2x_core_try_decode(ToyotaPmvc2xCore* instance) {
    if(instance->raw_count < TOYOTA_PMVC2X_MIN_FRAME_BITS) {
        return false;
    }

    for(size_t pattern_index = 0; pattern_index < COUNT_OF(toyota_pmvc2x_sync_patterns);
        pattern_index++) {
        const ToyotaPmvc2xSyncPattern* pattern = &toyota_pmvc2x_sync_patterns[pattern_index];
        if(instance->raw_count < (pattern->length + TOYOTA_PMVC2X_ENCODED_BITS)) {
            continue;
        }

        size_t limit = instance->raw_count - (pattern->length + TOYOTA_PMVC2X_ENCODED_BITS) + 1U;
        for(size_t offset = 0; offset < limit; offset++) {
            bool matched = toyota_pmvc2x_core_match_pattern(instance, offset, false, pattern);
            bool matched_invert = toyota_pmvc2x_core_match_pattern(instance, offset, true, pattern);
            if(!(matched || matched_invert)) {
                continue;
            }

            bool invert_options[2] = {false, true};
            size_t option_count = (matched && matched_invert) ? 2U : 1U;
            if(!matched) {
                invert_options[0] = true;
            }

            size_t base_start = offset + pattern->length - 1U;
            for(size_t option = 0; option < option_count; option++) {
                bool invert = invert_options[option];
                if((invert && !matched_invert) || (!invert && !matched)) {
                    continue;
                }

                for(int shift = -1; shift <= 2; shift++) {
                    int adjusted = (int)base_start + shift;
                    if(adjusted < 0) {
                        continue;
                    }

                    if(toyota_pmvc2x_core_decode_from(instance, (size_t)adjusted, invert)) {
                        return true;
                    }
                }
            }
        }
    }

    return false;
}

void toyota_pmvc2x_core_reset(ToyotaPmvc2xCore* instance) {
    if(instance == NULL) {
        return;
    }

    memset(&instance->frame, 0, sizeof(instance->frame));
    instance->frame_pending = false;
    toyota_pmvc2x_core_reset_buffer(instance);
}

bool toyota_pmvc2x_core_feed(ToyotaPmvc2xCore* instance, bool level, uint32_t duration_us) {
    if(instance == NULL) {
        return false;
    }

    if(duration_us > TOYOTA_PMVC2X_GAP_THRESHOLD_US) {
        if(toyota_pmvc2x_core_try_decode(instance)) {
            toyota_pmvc2x_core_reset_buffer(instance);
            return true;
        } else if(instance->raw_count > TOYOTA_PMVC2X_PATTERN_MIN_LEN) {
            size_t keep = TOYOTA_PMVC2X_PATTERN_MIN_LEN / 2U;
            if(keep > instance->raw_count) {
                keep = instance->raw_count;
            }
            if(keep > 0U) {
                memmove(
                    instance->raw_bits,
                    instance->raw_bits + (instance->raw_count - keep),
                    keep);
            }
            instance->raw_count = keep;
        } else {
            toyota_pmvc2x_core_reset_buffer(instance);
        }
        return false;
    }

    if(!toyota_pmvc2x_core_append_level(instance, level, duration_us)) {
        toyota_pmvc2x_core_reset_buffer(instance);
        return false;
    }

    if(instance->raw_count >= TOYOTA_PMVC2X_MIN_FRAME_BITS &&
       toyota_pmvc2x_core_try_decode(instance)) {
        toyota_pmvc2x_core_reset_buffer(instance);
        return true;
    }

    if(instance->raw_count >= COUNT_OF(instance->raw_bits)) {
        toyota_pmvc2x_core_reset_buffer(instance);
    }

    return false;
}

bool toyota_pmvc2x_core_take_frame(ToyotaPmvc2xCore* instance, ToyotaPmvc2xFrame* frame) {
    if((instance == NULL) || !instance->frame_pending) {
        return false;
    }

    if(frame != NULL) {
        *frame = instance->frame;
    }

    instance->frame_pending = false;
    return true;
}
