
#include <Arduino.h>
#include "simplecan.h"

#include <unity.h>

#include "../helper.h"

bool isExtendedFrame = false;
uint32_t frame______same = 0b10110100000;
uint32_t frame_____match = 0b10110100001;
uint32_t frame_no__match = 0b11110100001;
uint32_t acceptance_code = 0b10110100000;
uint32_t acceptance_mask = 0b11111110010;

void setup()
{
    delay(1000);

    UNITY_BEGIN();
    // frame identifier identical filter
    RUN_TEST(test_data_frame_identical_identifier_ignore_rtr_bit);
    RUN_TEST(test_data_frame_identical_identifier_rtr_bit_is_data_for_filter_and_frame);
#if defined(ARDUINO_ARCH_ESP32)
    RUN_TEST(test_data_frame_identical_identifier_rtr_bit_not_matching);
#endif
    RUN_TEST(test_remote_frame_identical_identifier_ignore_rtr_bit);
    RUN_TEST(test_remote_frame_identical_identifier_rtr_bit_is_remote_for_filter_and_frame);
#if defined(ARDUINO_ARCH_ESP32)
    RUN_TEST(test_remote_frame_identical_identifier_rtr_bit_not_matching);
#endif
    // frame identifier matches filter
    RUN_TEST(test_data_frame_matches_identifier_ignore_rtr_bit);
    RUN_TEST(test_data_frame_matches_identifier_rtr_bit_is_data_for_filter_and_frame);
#if defined(ARDUINO_ARCH_ESP32)
    RUN_TEST(test_data_frame_matches_identifier_rtr_bit_not_matching);
#endif
    RUN_TEST(test_remote_frame_matches_identifier_ignore_rtr_bit);
    RUN_TEST(test_remote_frame_matches_identifier_rtr_bit_is_remote_for_filter_and_frame);
#if defined(ARDUINO_ARCH_ESP32)
    RUN_TEST(test_remote_frame_matches_identifier_rtr_bit_not_matching);
#endif

    // frame identifier matches filter
    RUN_TEST(test_data_frame_no_match_identifier_ignore_rtr_bit);
    RUN_TEST(test_data_frame_no_match_identifier_rtr_bit_is_data_for_filter_and_frame);
#if defined(ARDUINO_ARCH_ESP32)
    RUN_TEST(test_data_frame_no_match_identifier_rtr_bit_not_matching);
#endif
    RUN_TEST(test_remote_frame_no_match_identifier_ignore_rtr_bit);
    RUN_TEST(test_remote_frame_no_match_identifier_rtr_bit_is_remote_for_filter_and_frame);
#if defined(ARDUINO_ARCH_ESP32)
    RUN_TEST(test_remote_frame_no_match_identifier_rtr_bit_not_matching);
#endif
    UNITY_END();
}

void loop()
{
}
