#pragma once

#include "SimpleCAN.h"
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

uint8_t *random_data()
{
    uint32_t randomNumber = random();

    static uint8_t data[4];
    data[0] = (randomNumber >> 24) & 0xFF; // Extract the most significant byte
    data[1] = (randomNumber >> 16) & 0xFF; // Extract the next byte
    data[2] = (randomNumber >> 8) & 0xFF;  // Extract the second least significant byte
    data[3] = randomNumber & 0xFF;
    return data;
}

void test_loopback_write_then_read(BaseCAN *can, uint8_t testIndex, CanMsg txMsg, bool expectMessageAccepted)
{
    char test_name[80];

#ifndef ARDUINO_ARCH_ESP32
    snprintf(test_name, sizeof(test_name), "test_loopback_write_then_read | rx_queue_is_initially_empty | %d", testIndex);
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, can->available(), test_name);
    UNITY_TEST_ASSERT_EQUAL_INT16(0, can->available(), 1, test_name);
#endif

    can->write(txMsg);
    delay(1);

    if (expectMessageAccepted)
    {
#ifndef ARDUINO_ARCH_ESP32
        snprintf(test_name, sizeof(test_name), "test_loopback_write_then_read | accepted_has_message | %d", testIndex);
        TEST_ASSERT_EQUAL_INT_MESSAGE(1, can->available(), test_name);
#endif
        snprintf(test_name, sizeof(test_name), "test_loopback_write_then_read | test_read_status_is_ok | %d", testIndex);
        CanMsg rxMsg = can->read();
        // TEST_ASSERT_EQUAL_INT_MESSAGE(CAN_OK, can->read(&rxMsg), test_name);
        snprintf(test_name, sizeof(test_name), "test_loopback_write_then_read | test_identifier_is_same | %d", testIndex);
        TEST_ASSERT_EQUAL_INT_MESSAGE(
            txMsg.isExtendedId() ? txMsg.getExtendedId() : txMsg.getStandardId(),
            rxMsg.isExtendedId() ? rxMsg.getExtendedId() : rxMsg.getStandardId(),
            test_name);
        snprintf(test_name, sizeof(test_name), "test_loopback_write_then_read | test_is_extended | %d", testIndex);
        TEST_ASSERT_EQUAL_INT_MESSAGE(txMsg.isExtendedId(), rxMsg.isExtendedId(), test_name);
        snprintf(test_name, sizeof(test_name), "test_loopback_write_then_read | test_is_length=4 | %d", testIndex);
        TEST_ASSERT_EQUAL_INT_MESSAGE(txMsg.data_length, rxMsg.data_length, test_name);
        if (!txMsg.isRTR())
        {
            long rx_data = (rxMsg.data[0] << 24) + (rxMsg.data[1] << 16) + (rxMsg.data[2] << 8) + rxMsg.data[3];
            long tx_data = (txMsg.data[0] << 24) + (txMsg.data[1] << 16) + (txMsg.data[2] << 8) + txMsg.data[3];
            snprintf(test_name, sizeof(test_name), "test_loopback_write_then_read | test_data_is_same | %d", testIndex);
            TEST_ASSERT_EQUAL_INT_MESSAGE(tx_data, rx_data, test_name);
        }
        snprintf(test_name, sizeof(test_name), "test_loopback_write_then_read | test_rtr | %d", testIndex);
        TEST_ASSERT_EQUAL_INT_MESSAGE(txMsg.isRTR(), rxMsg.isRTR(), test_name);
        delay(1);
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

void test_filter(uint8_t testIndex, CanMsg txMsg, CanFilter filter, bool loopback, bool expectPass)
{
    if (loopback)
    {
        CAN.enableInternalLoopback();
    }
    else
    {
        CAN.disableInternalLoopback();
    }
    CAN.filter(filter);
    CAN.begin(250000);

    delay(1);
    test_loopback_write_then_read(&CAN, testIndex, txMsg, expectPass);
    delay(1);
    CAN.end();
    delay(1);
}

void data_frame_same_id_filter_any()
{

    FrameType frameType = FILTER_ANY_FRAME;
    bool isRtr = false;
    uint32_t txIdentifier = frame______same;
    int data_length = 4;
    bool loopbackFlag = true;
    // uint8_t data[4];
    uint8_t *data = random_data();

    // uint32_t randomData = 0;
    // data[0] = (randomData >> 24) & 0xFF; // Extract the 4th byte (Most significant byte)
    // data[1] = (randomData >> 16) & 0xFF; // Extract the 3rd byte
    // data[2] = (randomData >> 8) & 0xFF;  // Extract the 2nd byte
    // data[3] = randomData & 0xFF;
    FilterType type = isExtendedFrame ? MASK_EXTENDED : MASK_STANDARD;
    bool EXPECT_OUTCOME = EXPECT_MESSAGE_ACCEPTED;

    CanMsg msg = CanMsg(
        isExtendedFrame ? CanExtendedId(txIdentifier, isRtr) : CanStandardId(txIdentifier, isRtr),
        data_length,
        data);

    CanFilter filter = CanFilter(type, acceptance_code, acceptance_mask, FILTER_ANY_FRAME);

    test_filter(1, msg, filter, loopbackFlag, EXPECT_OUTCOME);
}

void data_frame_same_id_filter_data()
{
    FrameType frameType = FILTER_DATA_FRAME;
    bool isRtr = false;
    uint32_t txIdentifier = frame______same;
    int data_length = 4;
    bool loopbackFlag = true;
    uint8_t *data = random_data();
    FilterType type = isExtendedFrame ? MASK_EXTENDED : MASK_STANDARD;
    bool EXPECT_OUTCOME = EXPECT_MESSAGE_ACCEPTED;

    CanMsg msg = CanMsg(
        isExtendedFrame ? CanExtendedId(txIdentifier, isRtr) : CanStandardId(txIdentifier, isRtr),
        data_length,
        data);

    CanFilter filter = CanFilter(type, acceptance_code, acceptance_mask, frameType);

    test_filter(2, msg, filter, loopbackFlag, EXPECT_OUTCOME);
}

void data_frame_same_id_filter_remote()
{
    FrameType frameType = FILTER_REMOTE_FRAME;
    bool isRtr = false;
    uint32_t txIdentifier = frame______same;
    int data_length = 4;
    bool loopbackFlag = true;
    uint8_t *data = random_data();
    FilterType type = isExtendedFrame ? MASK_EXTENDED : MASK_STANDARD;
    bool EXPECT_OUTCOME = EXPECT_MESSAGE_REJECTED;

    CanMsg msg = CanMsg(
        isExtendedFrame ? CanExtendedId(txIdentifier, isRtr) : CanStandardId(txIdentifier, isRtr),
        data_length,
        data);

    CanFilter filter = CanFilter(type, acceptance_code, acceptance_mask, frameType);

    test_filter(3, msg, filter, loopbackFlag, EXPECT_OUTCOME);
}

void remote_frame_same_id_filter_any()
{
    FrameType frameType = FILTER_ANY_FRAME;
    bool isRtr = true;
    uint32_t txIdentifier = frame______same;
    int data_length = 0;
    bool loopbackFlag = true;
    uint8_t *data = random_data();
    FilterType type = isExtendedFrame ? MASK_EXTENDED : MASK_STANDARD;
    bool EXPECT_OUTCOME = EXPECT_MESSAGE_ACCEPTED;

    CanMsg msg = CanMsg(
        isExtendedFrame ? CanExtendedId(txIdentifier, isRtr) : CanStandardId(txIdentifier, isRtr),
        data_length,
        data);

    CanFilter filter = CanFilter(type, acceptance_code, acceptance_mask, frameType);

    test_filter(4, msg, filter, loopbackFlag, EXPECT_OUTCOME);
}

void remote_frame_same_id_filter_remote()
{
    FrameType frameType = FILTER_REMOTE_FRAME;
    bool isRtr = true;
    uint32_t txIdentifier = frame______same;
    int data_length = 0;
    bool loopbackFlag = true;
    uint8_t *data = nullptr;
    FilterType type = isExtendedFrame ? MASK_EXTENDED : MASK_STANDARD;

    bool EXPECT_OUTCOME = EXPECT_MESSAGE_ACCEPTED;

    CanMsg msg = CanMsg(
        isExtendedFrame ? CanExtendedId(txIdentifier, isRtr) : CanStandardId(txIdentifier, isRtr),
        data_length,
        data);

    CanFilter filter = CanFilter(type, acceptance_code, acceptance_mask, frameType);

    test_filter(5, msg, filter, loopbackFlag, EXPECT_OUTCOME);
}

void remote_frame_same_id_filter_data()
{
    FrameType frameType = FILTER_DATA_FRAME;
    bool isRtr = true;
    uint32_t txIdentifier = frame______same;
    int data_length = 0;
    bool loopbackFlag = true;
    uint8_t *data = nullptr;
    FilterType type = isExtendedFrame ? MASK_EXTENDED : MASK_STANDARD;
    bool EXPECT_OUTCOME = EXPECT_MESSAGE_REJECTED;

    CanMsg msg = CanMsg(
        isExtendedFrame ? CanExtendedId(txIdentifier, isRtr) : CanStandardId(txIdentifier, isRtr),
        data_length,
        data);

    CanFilter filter = CanFilter(type, acceptance_code, acceptance_mask, frameType);

    test_filter(6, msg, filter, loopbackFlag, EXPECT_OUTCOME);
}

/*
filter and frame identifiers are not identical
*/
void data_frame_matches_id_filter_both()
{
    FrameType frameType = FILTER_ANY_FRAME;
    bool isRtr = false;
    uint32_t txIdentifier = frame_____match;
    int data_length = 4;
    bool loopbackFlag = true;
    uint8_t *data = random_data();
    FilterType type = isExtendedFrame ? MASK_EXTENDED : MASK_STANDARD;
    bool EXPECT_OUTCOME = EXPECT_MESSAGE_ACCEPTED;

    CanMsg msg = CanMsg(
        isExtendedFrame ? CanExtendedId(txIdentifier, isRtr) : CanStandardId(txIdentifier, isRtr),
        data_length,
        data);

    CanFilter filter = CanFilter(type, acceptance_code, acceptance_mask, frameType);

    test_filter(7, msg, filter, loopbackFlag, EXPECT_OUTCOME);
}

void data_frame_matches_id_filter_data()
{
    FrameType frameType = FILTER_DATA_FRAME;
    bool isRtr = false;
    uint32_t txIdentifier = frame_____match;
    int data_length = 4;
    bool loopbackFlag = true;
    uint8_t *data = random_data();
    FilterType type = isExtendedFrame ? MASK_EXTENDED : MASK_STANDARD;
    bool EXPECT_OUTCOME = EXPECT_MESSAGE_ACCEPTED;

    CanMsg msg = CanMsg(
        isExtendedFrame ? CanExtendedId(txIdentifier, isRtr) : CanStandardId(txIdentifier, isRtr),
        data_length,
        data);

    CanFilter filter = CanFilter(type, acceptance_code, acceptance_mask, frameType);

    test_filter(8, msg, filter, loopbackFlag, EXPECT_OUTCOME);
}

void data_frame_matches_id_filter_remote()
{
    FrameType frameType = FILTER_REMOTE_FRAME;
    bool isRtr = false;
    uint32_t txIdentifier = frame_____match;
    int data_length = 4;
    bool loopbackFlag = true;
    uint8_t *data = random_data();
    FilterType type = isExtendedFrame ? MASK_EXTENDED : MASK_STANDARD;
    bool EXPECT_OUTCOME = EXPECT_MESSAGE_REJECTED;

    CanMsg msg = CanMsg(
        isExtendedFrame ? CanExtendedId(txIdentifier, isRtr) : CanStandardId(txIdentifier, isRtr),
        data_length,
        data);

    CanFilter filter = CanFilter(type, acceptance_code, acceptance_mask, frameType);

    test_filter(9, msg, filter, loopbackFlag, EXPECT_OUTCOME);
}

void remote_frame_matches_id_filter_both()
{
    FrameType frameType = FILTER_ANY_FRAME;
    bool isRtr = true;
    uint32_t txIdentifier = frame_____match;
    int data_length = 0;
    bool loopbackFlag = true;
    uint8_t *data = nullptr;
    FilterType type = isExtendedFrame ? MASK_EXTENDED : MASK_STANDARD;
    bool EXPECT_OUTCOME = EXPECT_MESSAGE_ACCEPTED;

    CanMsg msg = CanMsg(
        isExtendedFrame ? CanExtendedId(txIdentifier, isRtr) : CanStandardId(txIdentifier, isRtr),
        data_length,
        data);

    CanFilter filter = CanFilter(type, acceptance_code, acceptance_mask, frameType);

    test_filter(10, msg, filter, loopbackFlag, EXPECT_OUTCOME);
}

void remote_frame_matches_id_filter_remote()
{
    FrameType frameType = FILTER_REMOTE_FRAME;
    bool isRtr = true;
    uint32_t txIdentifier = frame_____match;
    int data_length = 0;
    bool loopbackFlag = true;
    uint8_t *data = nullptr;
    FilterType type = isExtendedFrame ? MASK_EXTENDED : MASK_STANDARD;
    bool EXPECT_OUTCOME = EXPECT_MESSAGE_ACCEPTED;

    CanMsg msg = CanMsg(
        isExtendedFrame ? CanExtendedId(txIdentifier, isRtr) : CanStandardId(txIdentifier, isRtr),
        data_length,
        data);

    CanFilter filter = CanFilter(type, acceptance_code, acceptance_mask, frameType);

    test_filter(11, msg, filter, loopbackFlag, EXPECT_OUTCOME);
}

void remote_frame_matches_id_filter_data()
{
    FrameType frameType = FILTER_DATA_FRAME;
    bool isRtr = true;
    uint32_t txIdentifier = frame_____match;
    int data_length = 0;
    bool loopbackFlag = true;
    uint8_t *data = nullptr;
    FilterType type = isExtendedFrame ? MASK_EXTENDED : MASK_STANDARD;
    bool EXPECT_OUTCOME = EXPECT_MESSAGE_REJECTED;

    CanMsg msg = CanMsg(
        isExtendedFrame ? CanExtendedId(txIdentifier, isRtr) : CanStandardId(txIdentifier, isRtr),
        data_length,
        data);

    CanFilter filter = CanFilter(type, acceptance_code, acceptance_mask, frameType);

    test_filter(12, msg, filter, loopbackFlag, EXPECT_OUTCOME);
}

/*
filter and frame identifiers do not match
*/
void data_frame_no_match_id_filter_both()
{
    FrameType frameType = FILTER_ANY_FRAME;
    bool isRtr = false;
    uint32_t txIdentifier = frame_no__match;
    int data_length = 4;
    bool loopbackFlag = true;
    uint8_t *data = random_data();
    FilterType type = isExtendedFrame ? MASK_EXTENDED : MASK_STANDARD;
    bool EXPECT_OUTCOME = EXPECT_MESSAGE_REJECTED;

    CanMsg msg = CanMsg(
        isExtendedFrame ? CanExtendedId(txIdentifier, isRtr) : CanStandardId(txIdentifier, isRtr),
        data_length,
        data);

    CanFilter filter = CanFilter(type, acceptance_code, acceptance_mask, frameType);

    test_filter(1, msg, filter, loopbackFlag, EXPECT_OUTCOME);
}

void data_frame_no_match_id_filter_data()
{
    FrameType frameType = FILTER_DATA_FRAME;
    bool isRtr = false;
    uint32_t txIdentifier = frame_no__match;
    int data_length = 4;
    bool loopbackFlag = true;
    uint8_t *data = random_data();
    FilterType type = isExtendedFrame ? MASK_EXTENDED : MASK_STANDARD;
    bool EXPECT_OUTCOME = EXPECT_MESSAGE_REJECTED;

    CanMsg msg = CanMsg(
        isExtendedFrame ? CanExtendedId(txIdentifier, isRtr) : CanStandardId(txIdentifier, isRtr),
        data_length,
        data);

    CanFilter filter = CanFilter(type, acceptance_code, acceptance_mask, frameType);

    test_filter(1, msg, filter, loopbackFlag, EXPECT_OUTCOME);
}

void data_frame_no_match_id_filter_remote()
{
    FrameType frameType = FILTER_REMOTE_FRAME;
    bool isRtr = false;
    uint32_t txIdentifier = frame_no__match;
    int data_length = 4;
    bool loopbackFlag = true;
    uint8_t *data = random_data();
    FilterType type = isExtendedFrame ? MASK_EXTENDED : MASK_STANDARD;
    bool EXPECT_OUTCOME = EXPECT_MESSAGE_REJECTED;

    CanMsg msg = CanMsg(
        isExtendedFrame ? CanExtendedId(txIdentifier, isRtr) : CanStandardId(txIdentifier, isRtr),
        data_length,
        data);

    CanFilter filter = CanFilter(type, acceptance_code, acceptance_mask, frameType);

    test_filter(1, msg, filter, loopbackFlag, EXPECT_OUTCOME);
}

void remote_frame_no_match_id_filter_both()
{
    FrameType frameType = FILTER_ANY_FRAME;
    bool isRtr = true;
    uint32_t txIdentifier = frame_no__match;
    int data_length = 0;
    bool loopbackFlag = true;
    uint8_t *data = nullptr;
    FilterType type = isExtendedFrame ? MASK_EXTENDED : MASK_STANDARD;
    bool EXPECT_OUTCOME = EXPECT_MESSAGE_REJECTED;

    CanMsg msg = CanMsg(
        isExtendedFrame ? CanExtendedId(txIdentifier, isRtr) : CanStandardId(txIdentifier, isRtr),
        data_length,
        data);

    CanFilter filter = CanFilter(type, acceptance_code, acceptance_mask, frameType);

    test_filter(1, msg, filter, loopbackFlag, EXPECT_OUTCOME);
}

void remote_frame_no_match_idfilter_remote()
{

    FrameType frameType = FILTER_REMOTE_FRAME;
    bool isRtr = true;
    uint32_t txIdentifier = frame_no__match;
    int data_length = 0;
    bool loopbackFlag = true;
    uint8_t *data = nullptr;
    FilterType type = isExtendedFrame ? MASK_EXTENDED : MASK_STANDARD;
    bool EXPECT_OUTCOME = EXPECT_MESSAGE_REJECTED;

    CanMsg msg = CanMsg(
        isExtendedFrame ? CanExtendedId(txIdentifier, isRtr) : CanStandardId(txIdentifier, isRtr),
        data_length,
        data);

    CanFilter filter = CanFilter(type, acceptance_code, acceptance_mask, frameType);

    test_filter(1, msg, filter, loopbackFlag, EXPECT_OUTCOME);
}

void remote_frame_no_match_id_filter_data()
{
    FrameType frameType = FILTER_REMOTE_FRAME;
    bool isRtr = true;
    uint32_t txIdentifier = frame_no__match;
    int data_length = 0;
    bool loopbackFlag = true;
    uint8_t *data = nullptr;
    FilterType type = isExtendedFrame ? MASK_EXTENDED : MASK_STANDARD;
    bool EXPECT_OUTCOME = EXPECT_MESSAGE_REJECTED;

    CanMsg msg = CanMsg(
        isExtendedFrame ? CanExtendedId(txIdentifier, isRtr) : CanStandardId(txIdentifier, isRtr),
        data_length,
        data);

    CanFilter filter = CanFilter(type, acceptance_code, acceptance_mask, frameType);

    test_filter(1, msg, filter, loopbackFlag, EXPECT_OUTCOME);
}

void data_frame_same_id_not_loopback()
{

    FrameType frameType = FILTER_ANY_FRAME;
    bool isRtr = false;
    uint32_t txIdentifier = frame______same;
    int data_length = 4;
    uint8_t *data = random_data();
    bool loopbackFlag = false;

    FilterType type = isExtendedFrame ? MASK_EXTENDED : MASK_STANDARD;
    bool EXPECT_OUTCOME = EXPECT_MESSAGE_REJECTED;

    CanMsg msg = CanMsg(
        isExtendedFrame ? CanExtendedId(txIdentifier, isRtr) : CanStandardId(txIdentifier, isRtr),
        data_length,
        data);

    CanFilter filter = CanFilter(type, acceptance_code, acceptance_mask, FILTER_ANY_FRAME);

    test_filter(1, msg, filter, loopbackFlag, EXPECT_OUTCOME);
}
