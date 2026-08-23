#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define TOYOTA_PMVC2X_PAYLOAD_BYTES 9U

typedef struct {
    uint32_t id;
    uint8_t status;
    uint8_t pressure_raw;
    uint8_t temperature_raw;
    bool crc_ok;
    uint8_t payload[TOYOTA_PMVC2X_PAYLOAD_BYTES];
} ToyotaPmvc2xFrame;

typedef struct {
    uint8_t raw_bits[256];
    size_t raw_count;
    bool frame_pending;
    ToyotaPmvc2xFrame frame;
} ToyotaPmvc2xCore;

void toyota_pmvc2x_core_reset(ToyotaPmvc2xCore* instance);
bool toyota_pmvc2x_core_feed(ToyotaPmvc2xCore* instance, bool level, uint32_t duration_us);
bool toyota_pmvc2x_core_take_frame(ToyotaPmvc2xCore* instance, ToyotaPmvc2xFrame* frame);
