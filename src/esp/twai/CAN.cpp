#if defined(ARDUINO_ARCH_ESP32)

#include "CAN.h"
#include <Arduino.h>

void (*ESP_TWAI_CAN::receiveCallback)(CanMsg *rxMessage);
twai_message_t _rxEspFrame = {};
twai_message_t _txEspFrame = {};
twai_status_info_t _statusInfo = {};

uint16_t ESP_TWAI_CAN::_pinRX;
uint16_t ESP_TWAI_CAN::_pinTX;
uint16_t ESP_TWAI_CAN::_pinSHDN;

ESP_TWAI_CAN::ESP_TWAI_CAN(uint32_t pinRX, uint32_t pinTX, uint32_t pinSHDN)
{
    pinMode(pinSHDN, OUTPUT);
    digitalWrite(pinSHDN, HIGH);
    ESP_TWAI_CAN::_pinRX = pinRX;
    ESP_TWAI_CAN::_pinTX = pinTX;
    ESP_TWAI_CAN::_pinSHDN = pinSHDN;
    _filter_config = {
        .acceptance_code = 0xFFFFFFFF, // impossible identifier (larger than 29bits)
        .acceptance_mask = 0x00000000, // all bits must match impossible mask (reject all)
        .single_filter = true};

    mode = CAN_STANDARD;
}

bool ESP_TWAI_CAN::begin(int can_bitrate)
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

    const uint32_t clockFreq = APB_CLK_FREQ;

    CanTiming timing = solveCanTiming(clockFreq, (uint32_t)can_bitrate, 2); // <-- multiplier of 2 will ensure prescaler is even (as per spec)

    _timing_config = {
        .brp = timing.prescaler,
        .tseg_1 = (uint8_t)timing.tseg1,
        .tseg_2 = (uint8_t)timing.tseg2,
        .sjw = (uint8_t)timing.sjw,
        .triple_sampling = false};

    logStatus('i',
              twai_driver_install(&_general_config, &_timing_config, &_filter_config));

    if (_pinSHDN != NC)
    {
        digitalWrite(ESP_TWAI_CAN::_pinSHDN, LOW);
    }
    return logStatus('s',
                     twai_start());
}

void ESP_TWAI_CAN::end()
{
    if (_pinSHDN != NC)
    {
        digitalWrite(ESP_TWAI_CAN::_pinSHDN, HIGH);
    }
    logStatus('x',
              twai_stop());

    logStatus('u',
              twai_driver_uninstall());
}

void ESP_TWAI_CAN::filter(CanFilter filter)
{

    uint32_t maskExtraBits = 0b000;
    uint32_t identifierExtraBits = 0b000;

    if (filter.getFrameType() == FILTER_DATA_FRAME)
    {
        maskExtraBits = 0b111;
        identifierExtraBits = 0b000;
    }
    else if (filter.getFrameType() == FILTER_REMOTE_FRAME)
    {
        maskExtraBits = 0b111;
        identifierExtraBits = 0b111;
    }

    if (filter.getType() == FilterType::ACCEPT_ALL)
    {
        _filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    }
    else if (filter.getType() == FilterType::REJECT_ALL)
    {
        // TODO: check this is valid way of disabling or find better approach
        _filter_config = {.acceptance_code = 0xFFFFFFFF, // <- this is an impossible identifier
                          .acceptance_mask = 0x00000000, // <- this means all bits must match
                          .single_filter = true};
    }
    else if (filter.getType() == FilterType::MASK_STANDARD)
    {

        _filter_config = {.acceptance_code = ((filter.getIdentifier() << 3) | identifierExtraBits) << 18,
                          .acceptance_mask = ~((filter.getMask() << 3 | maskExtraBits) << 18), //~((mask << 3 | maskExtraBits)) << 18,
                          .single_filter = true};
    }
    else if (filter.getType() == FilterType::MASK_EXTENDED)
    {

        _filter_config = {.acceptance_code = (filter.getIdentifier() << 3) | identifierExtraBits,
                          .acceptance_mask = ~(filter.getMask() << 3 | maskExtraBits),
                          .single_filter = true};
    }

    // expectation is that begin() will be called after filter()
}

int ESP_TWAI_CAN::write(CanMsg const &txMsg)
{

    _txEspFrame.flags = TWAI_MSG_FLAG_NONE | txMsg.isRTR() ? TWAI_MSG_FLAG_RTR : 0;
    _txEspFrame.self = mode == CanMode::CAN_LOOPBACK ? 1 : 0;
    _txEspFrame.identifier = txMsg.isExtendedId() ? txMsg.getExtendedId() : txMsg.getStandardId();
    _txEspFrame.data_length_code = txMsg.data_length;
    _txEspFrame.extd = txMsg.isExtendedId() ? 1 : 0;
    if (txMsg.data_length > 0)
    {
        memcpy(_txEspFrame.data, txMsg.data, txMsg.data_length);
    }

    if (logStatus('w',
                  twai_transmit(&_txEspFrame, portMAX_DELAY)) == CAN_OK)
    {
#ifdef CAN_DEBUG
        _Serial->print("tx: ");
        txMsg.printTo(*_Serial);
        _Serial->println();
#endif
        return CAN_OK;
    }
    else
    {
        return CAN_ERROR;
    }
}

CanMsg ESP_TWAI_CAN::read()
{
    twai_message_t _rxEspFrame;

    memset(&_rxEspFrame, 0, sizeof(_rxEspFrame)); // <-zero before reusing _rxHeader

    if (logStatus('r', twai_receive(&_rxEspFrame, 0)) == CAN_OK)
    {
        CanMsg const rxMsg(
            (_rxEspFrame.extd) ? CanExtendedId(_rxEspFrame.identifier, _rxEspFrame.rtr)
                               : CanStandardId(_rxEspFrame.identifier, _rxEspFrame.rtr),
            _rxEspFrame.data_length_code,
            _rxEspFrame.data);

#ifdef CAN_DEBUG
        _Serial->print("rx: ");
        rxMsg.printTo(*_Serial);
        _Serial->println();
#endif
        return rxMsg;
    }
    else
    {
        return CanMsg(); // return empty message
    }
}

size_t ESP_TWAI_CAN::available()
{
    twai_status_info_t esp_status = {};
    twai_get_status_info(&esp_status);
    return esp_status.msgs_to_rx;
}

CanStatus ESP_TWAI_CAN::logStatus(char op, esp_err_t status)
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

#if CAN_HOWMANY > 0
ESP_TWAI_CAN CAN(PIN_CAN0_RX, PIN_CAN0_TX, PIN_CAN0_SHDN);
#endif

// #if CAN_HOWMANY > 1
// ESP_TWAI_CAN CAN1(PIN_CAN1_RX, PIN_CAN1_TX, PIN_CAN1_SHDN);
// #endif

#endif