#pragma once

#include "simplecan.h"
// #include <unity_internals.h>
#define EXPECT_MESSAGE_ACCEPTED true
#define EXPECT_MESSAGE_REJECTED false

// namespace TestSimpleCAN
// {

extern bool isExtendedFrame;
extern uint32_t frame______same;
extern uint32_t frame_____match;
extern uint32_t frame_no__match;
extern uint32_t acceptance_code;
extern uint32_t acceptance_mask;

void test_loopback_write_then_read(Can *can, uint8_t testIndex, uint32_t identifier, bool isExtended, bool isRtrFrame, bool expectMessageAccepted)
{

    char test_name[80];
    CanFrame txFrame = {};
    CanFrame rxFrame = {};
    uint32_t randomNumber = 0;

#ifndef ARDUINO_ARCH_ESP32
    snprintf(test_name, sizeof(test_name), "test_loopback_write_then_read | rx_queue_is_initially_empty | %d", testIndex);
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, can->available(), test_name);
    UNITY_TEST_ASSERT_EQUAL_INT16(0, can->available(), 1, test_name);
#endif

    txFrame.identifier = identifier;
    txFrame.isExtended = isExtended;
    txFrame.isRTR = isRtrFrame;

    if (isRtrFrame)
    {
        txFrame.dataLength = 4; // The spec allows rtr to have length > 0!!
    }
    else
    {
        txFrame.dataLength = 4;
        randomNumber = random();
        txFrame.data[0] = (randomNumber >> 24) & 0xFF; // Extract the most significant byte
        txFrame.data[1] = (randomNumber >> 16) & 0xFF; // Extract the next byte
        txFrame.data[2] = (randomNumber >> 8) & 0xFF;  // Extract the second least significant byte
        txFrame.data[3] = randomNumber & 0xFF;
    }

    can->writeFrame(&txFrame);
    delay(10);

    if (expectMessageAccepted)
    {
#ifndef ARDUINO_ARCH_ESP32
        snprintf(test_name, sizeof(test_name), "test_loopback_write_then_read | accepted_has_message | %d", testIndex);
        TEST_ASSERT_EQUAL_INT_MESSAGE(1, can->available(), test_name);
#endif
        snprintf(test_name, sizeof(test_name), "test_loopback_write_then_read | test_read_status_is_ok | %d", testIndex);
        TEST_ASSERT_EQUAL_INT_MESSAGE(CAN_OK, can->readFrame(&rxFrame), test_name);
        snprintf(test_name, sizeof(test_name), "test_loopback_write_then_read | test_identifier_is_same | %d", testIndex);
        TEST_ASSERT_EQUAL_INT_MESSAGE(identifier, rxFrame.identifier, test_name);
        snprintf(test_name, sizeof(test_name), "test_loopback_write_then_read | test_is_extended | %d", testIndex);
        TEST_ASSERT_EQUAL_INT_MESSAGE(isExtended, rxFrame.isExtended, test_name);
        snprintf(test_name, sizeof(test_name), "test_loopback_write_then_read | test_is_length=4 | %d", testIndex);
        TEST_ASSERT_EQUAL_INT_MESSAGE(4, rxFrame.dataLength, test_name);
        if (!isRtrFrame)
        {
            long rx_data = (rxFrame.data[0] << 24) + (rxFrame.data[1] << 16) + (rxFrame.data[2] << 8) + rxFrame.data[3];
            snprintf(test_name, sizeof(test_name), "test_loopback_write_then_read | test_data_is_same | %d", testIndex);
            TEST_ASSERT_EQUAL_INT_MESSAGE(randomNumber, rx_data, test_name);
        }
        snprintf(test_name, sizeof(test_name), "test_loopback_write_then_read | test_rtr | %d", testIndex);
        TEST_ASSERT_EQUAL_INT_MESSAGE(isRtrFrame, rxFrame.isRTR, test_name);
        delay(10);
#ifndef ARDUINO_ARCH_ESP32

        snprintf(test_name, sizeof(test_name), "test_loopback_write_then_read | rx_queue_cleared | %d", testIndex);
        TEST_ASSERT_EQUAL_INT_MESSAGE(0, can->available(), test_name);
#endif
    }
    else
    {
#ifndef ARDUINO_ARCH_ESP32

        snprintf(test_name, sizeof(test_name), "test_loopback_write_then_read | rejected_no_message | %d", testIndex);
        TEST_ASSERT_EQUAL_INT_MESSAGE(0, can->available(), test_name);
#endif
    }
}

void test_filter(uint8_t testIndex, uint32_t acceptance_code, uint32_t mask, uint32_t identifier, bool isExtended, bool maskRtrBit, bool filterRtrBit, bool isRtrFrame, bool expectPass)
{
#if defined(ARDUINO_ARCH_ESP32)
    Can can(GPIO_NUM_4, GPIO_NUM_5);
#elif defined(HAL_CAN_MODULE_ENABLED)
    Can can(PB_8, PB_9, NC);
#elif defined(HAL_FDCAN_MODULE_ENABLED)
    Can can(PA_11, PB_9, PB_4);
#else
#error "No CAN module is enabled, expecting a define for ARDUINO_ARCH_ESP32 | HAL_CAN_MODULE_ENABLED | HAL_FDCAN_MODULE_ENABLED"
#endif

    FilterType filterType = isExtended ? FILTER_MASK_EXTENDED : FILTER_MASK_STANDARD;

    can.init(500000, CanMode::CAN_LOOPBACK);
    can.filter(filterType, acceptance_code, mask, maskRtrBit, filterRtrBit);

    can.start();
    delay(10);
    test_loopback_write_then_read(&can, testIndex, identifier, isExtended, isRtrFrame, expectPass);
    delay(10);
    can.stop();
    delay(10);
    can.deinit();
}

void test_data_frame_identical_identifier_ignore_rtr_bit()
{

    bool matchRtrBit = false;
    bool filterRtrBit = false;
    bool isRtrFrame = false;

    bool EXPECT_OUTCOME = EXPECT_MESSAGE_ACCEPTED;

    test_filter(1, acceptance_code, acceptance_mask, frame______same, isExtendedFrame, matchRtrBit, filterRtrBit, isRtrFrame, EXPECT_OUTCOME);
}

void test_data_frame_identical_identifier_rtr_bit_is_data_for_filter_and_frame()
{
    bool matchRtrBit = true;
    bool filterRtrBit = false;
    bool isRtrFrame = false;

    bool EXPECT_OUTCOME = EXPECT_MESSAGE_ACCEPTED;

    test_filter(2, acceptance_code, acceptance_mask, frame______same, isExtendedFrame, matchRtrBit, filterRtrBit, isRtrFrame, EXPECT_OUTCOME);
}

void test_data_frame_identical_identifier_rtr_bit_not_matching()
{
    bool maskRtrBit = true;
    bool filterRtrBit = true;
    bool isRtrFrame = false;

    bool EXPECT_OUTCOME = EXPECT_MESSAGE_REJECTED;

    test_filter(3, acceptance_code, acceptance_mask, frame______same, isExtendedFrame, maskRtrBit, filterRtrBit, isRtrFrame, EXPECT_OUTCOME);
}

void test_remote_frame_identical_identifier_ignore_rtr_bit()
{
    bool matchRtrBit = false;
    bool filterRtrBit = false;
    bool isRtrFrame = true;

    bool EXPECT_OUTCOME = EXPECT_MESSAGE_ACCEPTED;

    test_filter(4, acceptance_code, acceptance_mask, frame______same, isExtendedFrame, matchRtrBit, filterRtrBit, isRtrFrame, EXPECT_OUTCOME);
}

void test_remote_frame_identical_identifier_rtr_bit_is_remote_for_filter_and_frame()
{
    bool matchRtrBit = true;
    bool filterRtrBit = true;
    bool isRtrFrame = true;

    bool EXPECT_OUTCOME = EXPECT_MESSAGE_ACCEPTED;

    test_filter(5, acceptance_code, acceptance_mask, frame______same, isExtendedFrame, matchRtrBit, filterRtrBit, isRtrFrame, EXPECT_OUTCOME);
}

void test_remote_frame_identical_identifier_rtr_bit_not_matching()
{
    bool maskRtrBit = true;
    bool filterRtrBit = false;
    bool isRtrFrame = true;

    bool EXPECT_OUTCOME = EXPECT_MESSAGE_REJECTED;

    test_filter(6, acceptance_code, acceptance_mask, frame______same, isExtendedFrame, maskRtrBit, filterRtrBit, isRtrFrame, EXPECT_OUTCOME);
}

/*
filter and frame identifiers are not identical
*/
void test_data_frame_matches_identifier_ignore_rtr_bit()
{
    bool matchRtrBit = false;
    bool filterRtrBit = false;
    bool isRtrFrame = false;

    bool EXPECT_OUTCOME = EXPECT_MESSAGE_ACCEPTED;

    test_filter(1, acceptance_code, acceptance_mask, frame_____match, isExtendedFrame, matchRtrBit, filterRtrBit, isRtrFrame, EXPECT_OUTCOME);
}

void test_data_frame_matches_identifier_rtr_bit_is_data_for_filter_and_frame()
{
    bool matchRtrBit = true;
    bool filterRtrBit = false;
    bool isRtrFrame = false;

    bool EXPECT_OUTCOME = EXPECT_MESSAGE_ACCEPTED;

    test_filter(2, acceptance_code, acceptance_mask, frame_____match, isExtendedFrame, matchRtrBit, filterRtrBit, isRtrFrame, EXPECT_OUTCOME);
}

void test_data_frame_matches_identifier_rtr_bit_not_matching()
{
    bool maskRtrBit = true;
    bool filterRtrBit = true;
    bool isRtrFrame = false;

    bool EXPECT_OUTCOME = EXPECT_MESSAGE_REJECTED;

    test_filter(3, acceptance_code, acceptance_mask, frame_____match, isExtendedFrame, maskRtrBit, filterRtrBit, isRtrFrame, EXPECT_OUTCOME);
}

void test_remote_frame_matches_identifier_ignore_rtr_bit()
{
    bool matchRtrBit = false;
    bool filterRtrBit = false;
    bool isRtrFrame = true;

    bool EXPECT_OUTCOME = EXPECT_MESSAGE_ACCEPTED;

    test_filter(4, acceptance_code, acceptance_mask, frame_____match, isExtendedFrame, matchRtrBit, filterRtrBit, isRtrFrame, EXPECT_OUTCOME);
}

void test_remote_frame_matches_identifier_rtr_bit_is_remote_for_filter_and_frame()
{
    bool matchRtrBit = true;
    bool filterRtrBit = true;
    bool isRtrFrame = true;

    bool EXPECT_OUTCOME = EXPECT_MESSAGE_ACCEPTED;

    test_filter(5, acceptance_code, acceptance_mask, frame_____match, isExtendedFrame, matchRtrBit, filterRtrBit, isRtrFrame, EXPECT_OUTCOME);
}

void test_remote_frame_matches_identifier_rtr_bit_not_matching()
{
    bool maskRtrBit = true;
    bool filterRtrBit = false;
    bool isRtrFrame = true;

    bool EXPECT_OUTCOME = EXPECT_MESSAGE_REJECTED;

    test_filter(6, acceptance_code, acceptance_mask, frame_____match, isExtendedFrame, maskRtrBit, filterRtrBit, isRtrFrame, EXPECT_OUTCOME);
}

/*
filter and frame identifiers do not match
*/
void test_data_frame_no_match_identifier_ignore_rtr_bit()
{

    bool matchRtrBit = false;
    bool filterRtrBit = false;
    bool isRtrFrame = false;

    bool EXPECT_OUTCOME = EXPECT_MESSAGE_REJECTED;

    test_filter(1, acceptance_code, acceptance_mask, frame_no__match, isExtendedFrame, matchRtrBit, filterRtrBit, isRtrFrame, EXPECT_OUTCOME);
}

void test_data_frame_no_match_identifier_rtr_bit_is_data_for_filter_and_frame()
{

    bool matchRtrBit = true;
    bool filterRtrBit = false;
    bool isRtrFrame = false;

    bool EXPECT_OUTCOME = EXPECT_MESSAGE_REJECTED;

    test_filter(2, acceptance_code, acceptance_mask, frame_no__match, isExtendedFrame, matchRtrBit, filterRtrBit, isRtrFrame, EXPECT_OUTCOME);
}

void test_data_frame_no_match_identifier_rtr_bit_not_matching()
{

    bool maskRtrBit = true;
    bool filterRtrBit = true;
    bool isRtrFrame = false;

    bool EXPECT_OUTCOME = EXPECT_MESSAGE_REJECTED;

    test_filter(3, acceptance_code, acceptance_mask, frame_no__match, isExtendedFrame, maskRtrBit, filterRtrBit, isRtrFrame, EXPECT_OUTCOME);
}

void test_remote_frame_no_match_identifier_ignore_rtr_bit()
{

    bool matchRtrBit = false;
    bool filterRtrBit = false;
    bool isRtrFrame = true;

    bool EXPECT_OUTCOME = EXPECT_MESSAGE_REJECTED;

    test_filter(4, acceptance_code, acceptance_mask, frame_no__match, isExtendedFrame, matchRtrBit, filterRtrBit, isRtrFrame, EXPECT_OUTCOME);
}

void test_remote_frame_no_match_identifier_rtr_bit_is_remote_for_filter_and_frame()
{

    bool matchRtrBit = true;
    bool filterRtrBit = true;
    bool isRtrFrame = true;

    bool EXPECT_OUTCOME = EXPECT_MESSAGE_REJECTED;

    test_filter(5, acceptance_code, acceptance_mask, frame_no__match, isExtendedFrame, matchRtrBit, filterRtrBit, isRtrFrame, EXPECT_OUTCOME);
}

void test_remote_frame_no_match_identifier_rtr_bit_not_matching()
{

    bool maskRtrBit = true;
    bool filterRtrBit = false;
    bool isRtrFrame = true;

    bool EXPECT_OUTCOME = EXPECT_MESSAGE_REJECTED;

    test_filter(6, acceptance_code, acceptance_mask, frame_no__match, isExtendedFrame, maskRtrBit, filterRtrBit, isRtrFrame, EXPECT_OUTCOME);
}

void test_filter_dummy()
{
    // UNITY_TEST_ASSERT_EQUAL_INT16(1, 1, 1, "This is always true");
}
// }