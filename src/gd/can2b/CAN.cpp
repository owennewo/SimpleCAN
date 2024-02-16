#if ARDUINO_ARCH_GD32

#include "./CAN.h"

extern "C" void CAN0_RX0_IRQHandler(void);
extern "C" void CAN1_RX0_IRQHandler(void);

void (*GD_CAN::callbackFunction_)() = nullptr;
can_receive_message_struct rxHeader_ = {};
can_trasnmit_message_struct txHeader_ = {};
uint32_t GD_CAN::hcan_ = 0;

uint16_t GD_CAN::pinRX_;
uint16_t GD_CAN::pinTX_;
uint16_t GD_CAN::pinSHDN_;

uint16_t can_prescaler;
int can_bitrate;
uint8_t can_tseg1;
uint8_t can_tseg2;
uint8_t can_sjw;

GD_CAN::GD_CAN(uint16_t pinRX, uint16_t pinTX, uint16_t pinSHDN) : filter_(CanFilter(FilterType::ACCEPT_ALL)), started_(false)
{
    hcan_ = CAN0;
    pinRX_ = pinRX;
    pinTX_ = pinTX;
    pinSHDN_ = pinSHDN;

    if (hcan_ == CAN0)
    {
        // rcu_apb1_clock_config(RCU_APB1_CKAHB_DIV2);

        // rcu_periph_clock_enable(RCU_CAN0);
        // rcu_periph_clock_enable(RCU_GPIOB);

        // gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
        // gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

        // pin_function((PinName)GD_CAN::pinRX_, pinmap_function((PinName)GD_CAN::pinRX_, PinMap_CAN_RD));
        // pin_function((PinName)GD_CAN::pinTX_, pinmap_function((PinName)GD_CAN::pinTX_, PinMap_CAN_TD));

        // // Clear bits 13 and 14 first
        // AFIO_PCF0 &= ~(AFIO_PCF0_CAN0_REMAP_BIT_13 | AFIO_PCF0_CAN0_REMAP_BIT_14);

        // // Set bit 14 only (for '10' binary configuration)
        // AFIO->PCF0 |= AFIO_PCF0_CAN0_REMAP_BIT_14;

        // NVIC_SetPriority(USBD_LP_CAN0_RX0_IRQn, 0);
        // nvic_irq_enable(USBD_LP_CAN0_RX0_IRQn, 3, 0);
    }
    else
    {
#ifdef GD32F30X_CL
        rcu_periph_clock_enable(RCU_CAN1);
        nvic_irq_enable(CAN1_RX0_IRQn, 0, 0);
#endif
    }

    if (pinSHDN != NC)
    {
        pinMode(pinSHDN, OUTPUT);
    }

    mode = CAN_NORMAL;
}

bool GD_CAN::begin(int bitrate)
{
    if (bitrate > 1000000)
    {
        failAndBlink(CAN_ERROR_BITRATE_TOO_HIGH);
    }

    can_parameter_struct can_parameter;

    can_deinit(hcan_);

    rcu_apb1_clock_config(RCU_APB1_CKAHB_DIV2);

    rcu_periph_clock_enable(RCU_CAN0);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);

    gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
    gpio_pin_remap_config(GPIO_CAN_PARTIAL_REMAP, ENABLE);

    uint32_t clockFreq = rcu_clock_freq_get(CK_APB1);

    CanTiming timing = solveCanTiming(clockFreq, bitrate);

    // nvic_irq_enable(USBD_LP_CAN0_RX0_IRQn, 3, 0);
    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);

    // can_parameter.resync_jump_width=CAN_BT_SJW_1TQ;
    can_parameter.time_segment_1 = CAN_BT_BS1_14TQ;
    can_parameter.time_segment_2 = CAN_BT_BS2_1TQ;
    can_parameter.prescaler = 15;

    can_parameter.resync_jump_width = (uint8_t)timing.sjw - 1;
    can_parameter.time_segment_1 = (uint8_t)(timing.tseg1 - 1);
    can_parameter.time_segment_2 = (uint8_t)(timing.tseg2 - 1);
    can_parameter.prescaler = (uint16_t)timing.prescaler;
    // working_mode seems badly named, but it's the only way to set loopback mode
    can_parameter.working_mode = mode == CAN_LOOPBACK ? CAN_LOOPBACK_MODE : CAN_NORMAL_MODE;

    can_bitrate = bitrate;

    logStatus('i',
              can_init(hcan_, &can_parameter));

    if (pinSHDN_ != NC)
    {
        digitalWrite(pinSHDN_, LOW);
    }

    started_ = true;
    applyFilter();
    // todo: sort out return type
    return true;
}

void GD_CAN::end()
{
    if (pinSHDN_ != NC)
    {
        digitalWrite(pinSHDN_, HIGH);
    }
    can_deinit(hcan_);
    started_ = false;
}

void GD_CAN::filter(CanFilter filter)
{
    filter_ = filter;
}

void GD_CAN::applyFilter()
{

    // Unimplemented: the ability to filter on IDE or RTR bits (personally not that useful?!)

    static uint32_t filterIdHigh = 0xffff;
    static uint32_t filterIdLow = 0xffff; // <-- all digits must match
    static uint32_t filterMaskHigh = 0xffff;
    static uint32_t filterMaskLow = 0xffff;
    uint8_t filterIndex = 0;

    if (filter_.getType() == FilterType::MASK_STANDARD)
    {
        filterIdHigh = filter_.getIdentifier() << 5; // make room for IDE, RTR bits (+ 3 unused)
        filterMaskHigh = filter_.getMask() << 5;
        filterIdLow = 0x0000;
        filterMaskLow = 0x0000;
    }
    else if (filter_.getType() == FilterType::MASK_EXTENDED)
    {
        filterIdLow = (filter_.getIdentifier() & 0x0000ffff) << 3; // make room for IDE, RTR bit (+ 1 unused bit)
        filterIdHigh = filter_.getIdentifier() >> 16;
        filterMaskLow = (filter_.getMask() & 0x0000ffff) << 3;
        filterMaskHigh = filter_.getMask() >> 16;
    }
    else if (filter_.getType() == FilterType::ACCEPT_ALL)
    {
        filterIdLow = 0x0000;
        filterIdHigh = 0x0000; //<- no digits have to match
        filterMaskLow = 0x0000;
        filterMaskHigh = 0x0000;
    }

#ifdef CAN_DEBUG
    _Serial->println("###### FILTER ######");
    _Serial->print("filterType: ");
    _Serial->print(filter_.getType());
    _Serial->print(" (identifier: ");
    _Serial->print(filter_.getIdentifier(), HEX);
    _Serial->print(",  mask: ");
    _Serial->print(filter_.getMask(), HEX);
    _Serial->println(")");
    _Serial->print("registers (filterIdLow: ");
    _Serial->print(filterIdLow, HEX);
    _Serial->print(", filterIdHigh: ");
    _Serial->print(filterIdHigh, HEX);
    _Serial->print(", filterMaskLow: ");
    _Serial->print(filterMaskLow, HEX);
    _Serial->print(", filterMaskHigh: ");
    _Serial->print(filterMaskHigh, HEX);
    _Serial->println(")");
#endif

    can_filter_parameter_struct can_filter;

    can_struct_para_init(CAN_FILTER_STRUCT, &can_filter);

    can_filter.filter_number = (hcan_ == CAN0) ? 0 : 15;
    can_filter.filter_mode = CAN_FILTERMODE_MASK;
    can_filter.filter_bits = CAN_FILTERBITS_32BIT;
    can_filter.filter_list_high = filterIdHigh;
    can_filter.filter_list_low = filterIdLow;
    can_filter.filter_mask_high = filterMaskHigh;
    can_filter.filter_mask_low = filterMaskLow;
    can_filter.filter_fifo_number = filterIndex;
    can_filter.filter_enable = ENABLE;
    // logStatus('f',
    can_filter_init(&can_filter);

    // todo: support interrupt approach (alternative to polling)
    // can_interrupt_enable(hcan_, CAN_INT_RFNE0 | CAN_INT_TME);
}

int GD_CAN::write(CanMsg const &txMsg)
{
#ifdef CAN_DEBUG
    _Serial->print("tx: ");
    txMsg.printTo(*_Serial);
    _Serial->println();

#endif

    txHeader_ = {
        .tx_sfid = txMsg.isExtendedId() ? 0 : txMsg.getStandardId(),
        .tx_efid = txMsg.isExtendedId() ? txMsg.getExtendedId() : 0,
        .tx_ff = txMsg.isExtendedId() ? (uint8_t)CAN_FF_EXTENDED : (uint8_t)CAN_FF_STANDARD,
        .tx_ft = txMsg.isRTR() ? (uint8_t)CAN_FT_REMOTE : (uint8_t)CAN_FT_DATA,
        .tx_dlen = txMsg.data_length,
    };

    if (txMsg.data_length > 0)
    {
        memcpy(txHeader_.tx_data, txMsg.data, txMsg.data_length);
    }

    return logStatus('t',
                     can_message_transmit(hcan_, &txHeader_));
}

CanMsg GD_CAN::read()
{

    memset(&rxHeader_, 0, sizeof(rxHeader_)); // <-zero before reusing rxHeader_

    can_message_receive(hcan_, CAN_FIFO0, &rxHeader_);
    can_error_enum err = can_error_get(hcan_);
    CanMsg const rxMsg(
        (rxHeader_.rx_ff == CAN_FF_EXTENDED) ? CanExtendedId(rxHeader_.rx_efid, rxHeader_.rx_ft == CAN_FT_REMOTE)
                                             : CanStandardId(rxHeader_.rx_sfid, rxHeader_.rx_ft == CAN_FT_REMOTE),
        rxHeader_.rx_dlen,
        rxHeader_.rx_data);

#ifdef CAN_DEBUG
    _Serial->print("rx: ");
    rxMsg.printTo(*_Serial);
    _Serial->println();
#endif
    return rxMsg;
}

size_t GD_CAN::available()
{
    return can_receive_message_length_get(hcan_, CAN_FIFO0);
}

void CAN0_RX0_IRQHandler(void)
{
    can_receive_message_struct receive_message;
    memset(&receive_message, 0, sizeof(receive_message));
    can_message_receive(CAN0, CAN_FIFO0, &receive_message);
}

CanStatus GD_CAN::logStatus(char op, uint32_t status)
{
#ifdef CAN_DEBUG
    if (status != 0x0)
    {
        _Serial->print("ERROR (");
        _Serial->print(op);
        _Serial->print(") ");
        _Serial->print(status);
    }
#endif
    return status == 0x0 ? CAN_OK : CAN_ERROR;
}

#if CAN_HOWMANY > 0
GD_CAN CAN(PIN_CAN0_RX, PIN_CAN0_TX, PIN_CAN0_SHDN);
#endif

// #if CAN_HOWMANY > 1
// GD_CAN CAN1(PIN_CAN1_RX, PIN_CAN1_TX, PIN_CAN1_SHDN);
// #endif

#endif