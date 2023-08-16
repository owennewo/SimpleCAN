#if defined(ARDUINO_ARCH_ESP32)

#include "can.h"
#include <Arduino.h>

void (*Can::receiveCallback)(CanFrame *rxMessage);
twai_message_t _rxEspFrame = {};
twai_message_t _txEspFrame = {};
twai_status_info_t _statusInfo = {};

uint16_t Can::_pinRX;
uint16_t Can::_pinTX;
uint16_t Can::_pinSHDN;

Can::Can(uint32_t pinRX, uint32_t pinTX, uint32_t pinSHDN)
{
    pinMode(pinSHDN, OUTPUT);
    digitalWrite(pinSHDN, HIGH);
    Can::_pinRX = pinRX;
    Can::_pinTX = pinTX;
    Can::_pinSHDN = pinSHDN;
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

    return logStatus('i',
                     twai_driver_install(&_general_config, &_timing_config, &_filter_config));
}

CanStatus Can::deinit()
{
    return logStatus('u',
                     twai_driver_uninstall());
}

CanStatus Can::start()
{
    if (_pinSHDN != NC)
    {
        digitalWrite(Can::_pinSHDN, LOW);
    }
    return logStatus('s',
                     twai_start());
}
CanStatus Can::stop()
{
    if (_pinSHDN != NC)
    {
        digitalWrite(Can::_pinSHDN, HIGH);
    }
    return logStatus('x',
                     twai_stop());
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

    return logStatus('f',
                     twai_driver_install(&_general_config, &_timing_config, &_filter_config));
}

CanStatus Can::writeFrame(CanFrame *txFrame)
{

    _txEspFrame.flags = TWAI_MSG_FLAG_NONE | txFrame->isRTR ? TWAI_MSG_FLAG_RTR : 0;
    _txEspFrame.self = _mode == CanMode::CAN_LOOPBACK ? 1 : 0;
    _txEspFrame.identifier = txFrame->identifier;
    _txEspFrame.data_length_code = txFrame->dataLength;
    _txEspFrame.extd = txFrame->isExtended ? 1 : 0;
    memcpy(_txEspFrame.data, txFrame->data, txFrame->dataLength);

    if (logStatus('w',
                  twai_transmit(&_txEspFrame, portMAX_DELAY)) == CAN_OK)
    {
#ifdef CAN_DEBUG
        _Serial->print("tx: ");
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

    if (logStatus('r', twai_receive(&_rxEspFrame, 0)) == CAN_OK)
    {
        rxFrame->dataLength = _rxEspFrame.data_length_code;
        rxFrame->identifier = _rxEspFrame.identifier;
        rxFrame->isRTR = _rxEspFrame.rtr;
        rxFrame->isExtended = _rxEspFrame.extd;

        memcpy(rxFrame->data, _rxEspFrame.data, rxFrame->dataLength);

#ifdef CAN_DEBUG
        _Serial->print("rx: ");
        logFrame(rxFrame);
#endif
        return CAN_OK;
    }
    else
    {
        return CAN_ERROR;
    }
}

uint32_t Can::available()
{
    twai_status_info_t esp_status = {};
    twai_get_status_info(&esp_status);
    return esp_status.msgs_to_rx;
}

CanStatus Can::logStatus(char op, esp_err_t status)
{
#ifdef CAN_DEBUG
    if (status != ESP_OK)
    {
        twai_get_status_info(&_statusInfo);

        _Serial->print("ERROR (");
        _Serial->print(op);
        _Serial->print(") ");
        _Serial->print(status, HEX);
        _Serial->print(", can_state: ");
        _Serial->print(_statusInfo.state, HEX); // google HAL_CAN_StateTypeDef e.g. 5 = HAL_CAN_STATE_ERROR
        if (_statusInfo.tx_error_counter > 0)
        {
            _Serial->print(", tx_error_counter: ");
            _Serial->print(_statusInfo.tx_error_counter, HEX);
        }
        if (_statusInfo.rx_error_counter > 0)
        {
            _Serial->print(", rx_error_counter: ");
            _Serial->print(_statusInfo.rx_error_counter, HEX);
        }
        if (_statusInfo.tx_failed_count > 0)
        {
            _Serial->print(", tx_failed_count: ");
            _Serial->print(_statusInfo.tx_failed_count, HEX);
        }
        if (_statusInfo.rx_missed_count > 0)
        {
            _Serial->print(", rx_missed_count: ");
            _Serial->print(_statusInfo.rx_missed_count, HEX);
        }
        if (_statusInfo.rx_overrun_count > 0)
        {
            _Serial->print(", rx_overrun_count: ");
            _Serial->print(_statusInfo.rx_overrun_count, HEX);
        }
        if (_statusInfo.arb_lost_count > 0)
        {
            _Serial->print(", arb_lost_count: ");
            _Serial->print(_statusInfo.arb_lost_count, HEX);
        }
        if (_statusInfo.bus_error_count > 0)
        {
            _Serial->print(", bus_error_count: ");
            _Serial->print(_statusInfo.bus_error_count, HEX);
        }
        _Serial->println();
    }
#endif
    return status == ESP_OK ? CAN_OK : CAN_ERROR;
}

#endif