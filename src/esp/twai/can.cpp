#if defined(ARDUINO_ARCH_ESP32)

#include "can.h"
#include <Arduino.h>

void (*Can::receiveCallback)(CanFrame *rxMessage);

WEAK void SIMPLECAN_ESP_INIT(uint32_t bitrate, twai_timing_config_t *timing, CanMode mode);

Can::Can(gpio_num_t pinRX, gpio_num_t pinTX, gpio_num_t pinSHDN) : BaseCan(pinRX, pinTX, pinSHDN)
{
    pinMode(pinSHDN, OUTPUT);
    digitalWrite(pinSHDN, HIGH);
    _filter_config = {
        .acceptance_code = 0xFFFFFFFF, // impossible identifier (larger than 29bits)
        .acceptance_mask = 0x00000000, // all bits must match impossible mask (reject all)
        .single_filter = true};
}

CanStatus Can::init(uint32_t bitrate, CanMode mode)
{
    _general_config = {
        .mode = mode == CanMode::CAN_LOOPBACK ? TWAI_MODE_NO_ACK : TWAI_MODE_NORMAL,
        .tx_io = static_cast<gpio_num_t>(_pinTX),
        .rx_io = static_cast<gpio_num_t>(_pinRX),
        .clkout_io = TWAI_IO_UNUSED,
        .bus_off_io = TWAI_IO_UNUSED,
        .tx_queue_len = 5,
        .rx_queue_len = 5,
        .alerts_enabled = TWAI_ALERT_NONE,
        .clkout_divider = 0};

    _timing_config = TWAI_TIMING_CONFIG_500KBITS();

    _mode = mode;

    SIMPLECAN_ESP_INIT(bitrate, &_timing_config, mode);

    return twai_driver_install(&_general_config, &_timing_config, &_filter_config) == ESP_OK ? CAN_OK : CAN_ERROR;
}

CanStatus Can::deinit()
{
    return twai_driver_uninstall() == ESP_OK ? CAN_OK : CAN_ERROR;
}

CanStatus Can::start()
{
    if (_pinSHDN != NC)
    {
        digitalWrite(Can::_pinSHDN, LOW);
    }
    return twai_start() == ESP_OK ? CAN_OK : CAN_ERROR;
}
CanStatus Can::stop()
{
    if (_pinSHDN != NC)
    {
        digitalWrite(Can::_pinSHDN, HIGH);
    }
    return twai_stop() == ESP_OK ? CAN_OK : CAN_ERROR;
}

CanStatus Can::filter(FilterType filterType, uint32_t identifier, uint32_t mask, bool maskRtrBit, bool identifierRtrBit)
{

    if (filterType == FilterType::FILTER_ACCEPT_ALL)
    {
        _filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    }
    else if (filterType == FilterType::FILTER_DISABLE)
    {
        // TODO: check this is valid way of disabling or find better approach
        _filter_config = {.acceptance_code = 0xFFFFFFFF, // <- this is an impossible identifier
                          .acceptance_mask = 0x00000000, // <- this means all bits must match
                          .single_filter = true};
    }
    else if (filterType == FilterType::FILTER_MASK_STANDARD_ID)
    {
        // note: not attempting to support the data matching part of spec
        uint32_t maskExtraBits = maskRtrBit ? 0b111 : 0b000;
        uint32_t identifierExtraBits = identifierRtrBit ? 0b111 : 0b000;

        _filter_config = {.acceptance_code = ((identifier << 3) | identifierExtraBits) << 18,
                          .acceptance_mask = ~((mask << 3 | maskExtraBits) << 18), //~((mask << 3 | maskExtraBits)) << 18,
                          .single_filter = true};
    }
    else if (filterType == FilterType::FILTER_MASK_EXTENDED_ID)
    {
        uint32_t maskExtraBits = maskRtrBit ? 0b111 : 0b000;
        uint32_t identifierExtraBits = identifierRtrBit ? 0b111 : 0b000;

        _filter_config = {.acceptance_code = (identifier << 3) | identifierExtraBits,
                          .acceptance_mask = ~(mask << 3 | maskExtraBits),
                          .single_filter = true};
    }

#ifdef CAN_DEBUG
    // Serial.println("###### FILTER ######");
    // if (_filter_config.single_filter == true)
    // {
    Serial.print(" single filter (identifier: 0x");
    Serial.print(identifier, HEX);
    Serial.print(", mask: 0x");
    Serial.print(mask, HEX);
    Serial.println(")");
    // }
    // else
    // {
    //     Serial.print(" dual filters (identifier1: 0x");
    //     Serial.print(identifier & 0x0000FFFF, HEX);
    //     Serial.print(", mask1: 0x");
    //     Serial.print(mask & 0x0000FFFF, HEX);
    //     Serial.print(", identifier2: 0x");
    //     Serial.print((identifier & 0xFFFF0000) >> 16, HEX);
    //     Serial.print(", mask2: ");
    //     Serial.print((mask & 0xFFFF0000) >> 16, HEX);
    //     Serial.println(")");
    // }
#endif

    return twai_driver_install(&_general_config, &_timing_config, &_filter_config) == ESP_OK ? CAN_OK : CAN_ERROR;
}

CanStatus Can::writeFrame(CanFrame *txFrame)
{
    twai_message_t tx;
    tx.flags = TWAI_MSG_FLAG_NONE | txFrame->isRTR ? TWAI_MSG_FLAG_RTR : 0;
    tx.self = (_mode == CanMode::CAN_LOOPBACK) ? 1 : 0;
    tx.identifier = txFrame->identifier;
    tx.data_length_code = txFrame->dataLength;
    tx.extd = txFrame->isExtended ? 1 : 0;
    memcpy(tx.data, txFrame->data, txFrame->dataLength);

    if (twai_transmit(&tx, portMAX_DELAY) == ESP_OK)
    {
#ifdef CAN_DEBUG
        Serial.print("✅ tx: ");
        Serial.print(txFrame->identifier, HEX);
        Serial.print(" [");

        // uint8_t length = dlcToLength(dataLength);
        Serial.print(txFrame->dataLength);
        Serial.print("] ");
        for (uint32_t byte_index = 0; byte_index < txFrame->dataLength; byte_index++)
        {
            Serial.print(tx.data[byte_index], HEX);
            Serial.print(" ");
        }
        Serial.println();
#endif
        return CAN_OK;
    }
    else
    {
#ifdef CAN_DEBUG
        Serial.println("❌ tx");
#endif
        return CAN_ERROR;
    }
}

CanStatus Can::readFrame(CanFrame *rxMessage)
{

    twai_message_t message;
    Serial.println("waiting for message");

    esp_err_t status = twai_receive(&message, 0);
    if (status == ESP_OK)
    {
        Serial.print("received: ");
        Serial.println(message.data_length_code);
        // todo: check if this is correct
        rxMessage->dataLength = message.data_length_code;
        rxMessage->identifier = message.identifier;
        rxMessage->isRTR = message.rtr; // check this
        rxMessage->isExtended = message.extd;

        memcpy(rxMessage->data, message.data, rxMessage->dataLength);

#ifdef CAN_DEBUG
        Serial.print("✅ rx: ");
        Serial.print(rxMessage->identifier, HEX);
        Serial.print(" [");

        Serial.print(rxMessage->dataLength);
        Serial.print("] ");
        for (uint32_t byte_index = 0; byte_index < rxMessage->dataLength; byte_index++)
        {
            Serial.print(rxMessage->data[byte_index], HEX);
            Serial.print(" ");
        }
        Serial.print(message.extd);
        Serial.print(" ");
        Serial.print(message.rtr);
        Serial.println();
        return CAN_OK;
#endif
    }
    else if (status == ESP_ERR_TIMEOUT)
    {
#ifdef CAN_DEBUG
        Serial.println("🚫 rx no data");
#endif
        return CAN_NO_DATA;
    }
    else
    {
#ifdef CAN_DEBUG
        Serial.println("❌ rx");
#endif
        return CAN_ERROR;
    }
}

void SIMPLECAN_ESP_INIT(uint32_t bitrate, twai_timing_config_t *timing, CanMode mode)
{
    const uint32_t clockFreq = APB_CLK_FREQ;

    // Looking for a timeQuanta of between 8 and 25.
    // start at 16 and work outwards
    // this algo is inspired by: http://www.bittiming.can-wiki.info/

    uint32_t baseQuanta = 16;
    uint32_t timeQuanta = baseQuanta;

    uint32_t offset = 0;
    bool found = false;

    while (offset <= 9)
    {
        timeQuanta = baseQuanta - offset;
        if (clockFreq % (bitrate * timeQuanta * 2) == 0)
        {
            found = true;
            break;
        }
        timeQuanta = baseQuanta + offset;
        if (clockFreq % (bitrate * timeQuanta * 2) == 0)
        {
            found = true;
            break;
        }
        offset += 1;
    }
    if (!found)
    {
#ifdef CAN_DEBUG
        Serial.println("timeQuanta out of range");
#endif
        return;
    }

    uint32_t bitrate_prescaler = clockFreq / (bitrate * timeQuanta);

    uint32_t tseg_1 = uint32_t(0.875 * timeQuanta) - 1;

    float samplePoint = (1.0 + tseg_1) / timeQuanta;
    float samplePoint2 = (1.0 + tseg_1 + 1) / timeQuanta;

    if (abs(samplePoint2 - 0.875) < abs(samplePoint - 0.875))
    {
        tseg_1 += 1;
        samplePoint = samplePoint2;
    }

    uint32_t tseg_2 = timeQuanta - tseg_1 - 1;

    timing->brp = bitrate_prescaler;
    timing->tseg_1 = tseg_1;
    timing->tseg_2 = tseg_2;
    timing->sjw = 3;
    timing->triple_sampling = false;

#ifdef CAN_DEBUG

    uint32_t solvedBitrate = (APB_CLK_FREQ / bitrate_prescaler) / (1 + tseg_1 + tseg_2);

    Serial.println("###### TIMINGS ######");
    Serial.print("target bitrate:");
    Serial.print(bitrate);
    Serial.print(" (coreFreq:");
    Serial.print(ESP.getCpuFreqMHz());
    Serial.print(", APB_CLK_FREQ ");
    Serial.print(APB_CLK_FREQ);
    Serial.println(")");

    Serial.print("solution bitrate:");
    Serial.print(bitrate);
    Serial.print(" (bitrate_prescaler:");
    Serial.print(bitrate_prescaler);
    Serial.print(", timeQuanta:");
    Serial.print(timeQuanta);
    Serial.print(", tseg_1:");
    Serial.print(tseg_1);
    Serial.print(", tseg_2:");
    Serial.print(tseg_2);
    Serial.print(", samplePoint:");
    Serial.print(samplePoint);
    Serial.println(")");
#endif
}

#endif