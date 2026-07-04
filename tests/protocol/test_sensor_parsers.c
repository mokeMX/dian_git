#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "a02yyuw.h"
#include "bu_uwb.h"
#include "fsr_adc.h"

static void test_a02yyuw_valid_frame(void)
{
    const uint8_t frame[] = {0xFF, 0x07, 0xA1, 0xA7};
    a02yyuw_reading_t reading = {0};

    assert(a02yyuw_parse_frame(frame, sizeof(frame), &reading));
    assert(reading.valid);
    assert(reading.distance_mm == 1953);
}

static void test_a02yyuw_rejects_bad_checksum(void)
{
    const uint8_t frame[] = {0xFF, 0x07, 0xA1, 0x00};
    a02yyuw_reading_t reading = {0};

    assert(!a02yyuw_parse_frame(frame, sizeof(frame), &reading));
}

static void test_bu_uwb_distance_line(void)
{
    bu_uwb_distance_t reading = {0};

    assert(bu_uwb_parse_distance_line("distance: 0.340000", &reading));
    assert(reading.valid);
    assert(fabsf(reading.distance_m - 0.34f) < 0.0001f);
    assert(reading.distance_mm == 340);
}

static void test_bu_uwb_twr_line(void)
{
    bu_uwb_twr_reading_t reading = {0};

    assert(bu_uwb_parse_twr_line(
        "JS006E{\"TWR\": {\"a16\":\"1081\",\"R\":96,\"T\":846149,\"D\":18,"
        "\"P\":-133,\"Xcm\":-14,\"Ycm\":10,\"O\":0,\"V\":49152,"
        "\"X\":0,\"Y\":0,\"Z\":0}}",
        &reading));
    assert(reading.valid);
    assert(strcmp(reading.frame_id, "JS006E") == 0);
    assert(strcmp(reading.anchor_id, "1081") == 0);
    assert(reading.r == 96);
    assert(reading.timestamp == 846149);
    assert(reading.distance_cm == 18);
    assert(reading.p == -133);
    assert(reading.x_cm == -14);
    assert(reading.y_cm == 10);
    assert(reading.validity == 49152);

    assert(!bu_uwb_parse_twr_line(
        "JS006E{\"TWR\": {\"a16\":\"1081\",\"R\":246,\"T\":974794,\"D\":12,"
        "\"P\":-143,\"Xcm\":-14,\"Ycm\":8,\"O\":0,\"V\":49\":0,"
        "\"X\":0,\"Y\":0,\"Z\":0}}",
        &reading));
}

static void test_fsr_linear_calibration(void)
{
    const fsr_adc_calibration_t cal = {
        .slope_v_per_kg = 0.0004f,
        .offset_v = 0.0749f,
        .min_kg = 0.0f,
        .max_kg = 6.0f,
    };

    assert(fabsf(fsr_adc_voltage_to_weight_kg(0.0753f, &cal) - 1.0f) < 0.001f);
    assert(fsr_adc_voltage_to_weight_kg(0.0100f, &cal) == 0.0f);
    assert(fsr_adc_voltage_to_weight_kg(9.0000f, &cal) == 6.0f);
}

int main(void)
{
    test_a02yyuw_valid_frame();
    test_a02yyuw_rejects_bad_checksum();
    test_bu_uwb_distance_line();
    test_bu_uwb_twr_line();
    test_fsr_linear_calibration();
    puts("sensor parser tests passed");
    return 0;
}
