#include <Arduino.h>
// #ifndef UNITY_INCLUDE_CONFIG_H
#include <unity.h>
// #endif
#include "SimpleCAN.h"

bool isExtendedFrame = true;
uint32_t frame______same = 0b11111111111111111110110100000;
uint32_t frame_____match = 0b11111111111111111110110100001;
uint32_t frame_no__match = 0b11111110111111111110110100001;
uint32_t acceptance_code = 0b11111111111111111110110100000;
uint32_t acceptance_mask = 0b11111111111111111111111110010;

// uint32_t frame______same = 0b10110100000;
// uint32_t acceptance_code = 0b10110100000;
// uint32_t acceptance_mask = 0b11111110010;

#include "../helper.h"

void setup()
{
    delay(1000);
    UNITY_BEGIN();
    // frame identifier identical filter
    RUN_TEST(data_frame_same_id_filter_any);
    RUN_TEST(data_frame_same_id_filter_data);
#if defined(ARDUINO_ARCH_ESP32)
    RUN_TEST(data_frame_same_id_filter_remote);
#endif
    RUN_TEST(remote_frame_same_id_filter_any);
    RUN_TEST(remote_frame_same_id_filter_remote);
#if defined(ARDUINO_ARCH_ESP32)
    RUN_TEST(remote_frame_same_id_filter_data);
#endif
    // frame identifier matches filter
    RUN_TEST(data_frame_matches_id_filter_both);
    RUN_TEST(data_frame_matches_id_filter_data);
#if defined(ARDUINO_ARCH_ESP32)
    RUN_TEST(data_frame_matches_id_filter_remote);
#endif
    RUN_TEST(remote_frame_matches_id_filter_both);
    RUN_TEST(remote_frame_matches_id_filter_remote);
#if defined(ARDUINO_ARCH_ESP32)
    RUN_TEST(remote_frame_matches_id_filter_data);
#endif

    // frame identifier matches filter
    RUN_TEST(data_frame_no_match_id_filter_both);
    RUN_TEST(data_frame_no_match_id_filter_data);
#if defined(ARDUINO_ARCH_ESP32)
    RUN_TEST(data_frame_no_match_id_filter_remote);
#endif
    RUN_TEST(remote_frame_no_match_id_filter_both);
    RUN_TEST(remote_frame_no_match_idfilter_remote);
#if defined(ARDUINO_ARCH_ESP32)
    RUN_TEST(remote_frame_no_match_id_filter_data);
#endif
    RUN_TEST(data_frame_same_id_not_loopback);
    UNITY_END();
}

void loop()
{
}