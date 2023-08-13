#if defined(ARDUINO_ARCH_ESP32)

#include "can.h"
#include <Arduino.h>

void (*Can::receiveCallback)(CanFrame *rxMessage);
twai_message_t _rxEspFrame = {};
twai_message_t _txEspFrame = {};

Can::Can(uint32_t pinRX, uint32_t pinTX, uint32_t pinSHDN) : BaseCan(pinRX, pinTX, pinSHDN)
{
    pinMode(pinSHDN, OUTPUT);
    digitalWrite(pinSHDN, HIGH);
    _filter_config = {
        .acceptance_code = 0xFFFFFFFF, // impossible identifier (larger than 29bits)
        .acceptance_mask = 0x00000000, // all bits must match impossible mask (reject all)
        .single_filter = true};
}

CanStatus Can::init(CanMode mode, uint32_t bitrate)
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

    _timing_config = {};

    _mode = mode;

    const uint32_t clockFreq = APB_CLK_FREQ;

    CanTiming timing = solveCanTiming(clockFreq, bitrate, 2); // <-- multiplier of 2 will ensure prescaler is even (as per spec)

    _timing_config.brp = timing.prescaler;
    _timing_config.tseg_1 = timing.tseg1;
    _timing_config.tseg_2 = timing.tseg2;
    _timing_config.sjw = timing.sjw;
    _timing_config.triple_sampling = false;

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
    // it isn't possible to set filters after init, so we need to deinit and reinit
    deinit();

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
    else if (filterType == FilterType::FILTER_MASK_STANDARD)
    {
        // note: not attempting to support the data matching part of spec
        uint32_t maskExtraBits = maskRtrBit ? 0b111 : 0b000;
        uint32_t identifierExtraBits = identifierRtrBit ? 0b111 : 0b000;

        _filter_config = {.acceptance_code = ((identifier << 3) | identifierExtraBits) << 18,
                          .acceptance_mask = ~((mask << 3 | maskExtraBits) << 18), //~((mask << 3 | maskExtraBits)) << 18,
                          .single_filter = true};
    }
    else if (filterType == FilterType::FILTER_MASK_EXTENDED)
    {
        uint32_t maskExtraBits = maskRtrBit ? 0b111 : 0b000;
        uint32_t identifierExtraBits = identifierRtrBit ? 0b111 : 0b000;

        _filter_config = {.acceptance_code = (identifier << 3) | identifierExtraBits,
                          .acceptance_mask = ~(mask << 3 | maskExtraBits),
                          .single_filter = true};
    }

    return twai_driver_install(&_general_config, &_timing_config, &_filter_config) == ESP_OK ? CAN_OK : CAN_ERROR;
}

CanStatus Can::writeFrame(CanFrame *txFrame)
{

    _txEspFrame.flags = TWAI_MSG_FLAG_NONE | txFrame->isRTR ? TWAI_MSG_FLAG_RTR : 0;
    _txEspFrame.self = _mode == CanMode::CAN_LOOPBACK ? 1 : 0;
    _txEspFrame.identifier = txFrame->identifier;
    _txEspFrame.data_length_code = txFrame->dataLength;
    _txEspFrame.extd = txFrame->isExtended ? 1 : 0;
    memcpy(_txEspFrame.data, txFrame->data, txFrame->dataLength);

    if (twai_transmit(&_txEspFrame, portMAX_DELAY) == ESP_OK)
    {
#ifdef CAN_DEBUG
        Serial.print("tx: ");
        logFrame(txFrame);
#endif
        return CAN_OK;
    }
    else
    {
        return CAN_ERROR;
    }
}

CanStatus Can::readFrame(CanFrame *rxFrame)
{
    memset(&_rxEspFrame, 0, sizeof(_rxEspFrame)); // <-zero before reusing _rxHeader

    esp_err_t status = twai_receive(&_rxEspFrame, 0);
    if (status == ESP_OK)
    {
        rxFrame->dataLength = _rxEspFrame.data_length_code;
        rxFrame->identifier = _rxEspFrame.identifier;
        rxFrame->isRTR = _rxEspFrame.rtr;
        rxFrame->isExtended = _rxEspFrame.extd;

        memcpy(rxFrame->data, _rxEspFrame.data, rxFrame->dataLength);

#ifdef CAN_DEBUG
        Serial.print("rx: ");
        logFrame(rxFrame);
#endif
        return CAN_OK;
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
        return CAN_ERROR;
    }
}
#endif