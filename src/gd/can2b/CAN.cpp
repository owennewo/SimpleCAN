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

GD_CAN::GD_CAN(uint16_t pinRX, uint16_t pinTX, uint16_t pinSHDN) : filter_(CanFilter(FilterType::ACCEPT_ALL)), started_(false)
{

    hcan_ = CAN0;
    pinRX_ = pinRX;
    pinTX_ = pinTX;
    pinSHDN_ = pinSHDN;

    if (hcan_ == CAN0)
    {
        rcu_periph_clock_enable(RCU_CAN0);
        nvic_irq_enable(USBD_LP_CAN0_RX0_IRQn, 0, 0);
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

    uint32_t clockFreq = SystemCoreClock; // HAL_RCC_GetPCLK1Freq();
    // uint32_t clockFreq = HAL_RCC_GetSysClockFreq();

    CanTiming timing = solveCanTiming(clockFreq, bitrate);
    // CAN_InitTypeDef *init = &(hcan_.Init);

    can_parameter_struct can_parameter;

    can_deinit(hcan_);

    /* initialize CAN */
    can_parameter.time_triggered = DISABLE;
    can_parameter.auto_bus_off_recovery = DISABLE;
    can_parameter.auto_wake_up = DISABLE;
    can_parameter.auto_retrans = ENABLE;
    can_parameter.rec_fifo_overwrite = DISABLE;
    can_parameter.trans_fifo_order = DISABLE;
    can_parameter.working_mode = mode == CanMode::CAN_LOOPBACK ? CAN_LOOPBACK_MODE : CAN_NORMAL_MODE;
    /* configure baudrate to 125kbps */
    can_parameter.resync_jump_width = timing.sjw - 1;
    can_parameter.time_segment_1 = timing.tseg1 - 1;
    can_parameter.time_segment_2 = timing.tseg1 - 2;
    can_parameter.prescaler = timing.prescaler;
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
    // return logStatus('s',
    //                  HAL_CAN_Start(&hcan_));
}

void GD_CAN::end()
{
    if (pinSHDN_ != NC)
    {
        digitalWrite(pinSHDN_, HIGH);
    }
    // logStatus('x',
    //           HAL_CAN_Stop(&hcan_));

    // logStatus('d',
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

    // CAN_FilterTypeDef filterDef = {
    //     .FilterIdHigh = filterIdHigh,
    //     .FilterIdLow = filterIdLow,
    //     .FilterMaskIdHigh = filterMaskHigh,
    //     .FilterMaskIdLow = filterMaskLow,
    //     .FilterFIFOAssignment = CAN_FILTER_FIFO0,
    //     .FilterBank = filterIndex,
    //     .FilterMode = CAN_FILTERMODE_IDMASK,
    //     .FilterScale = filter_.getType() == FilterType::MASK_EXTENDED ? CAN_FILTERSCALE_32BIT : CAN_FILTERSCALE_16BIT,
    //     .FilterActivation = filter_.getType() == FilterType::REJECT_ALL ? DISABLE : ENABLE,
    // };
    // logStatus('f',
    //           HAL_CAN_ConfigFilter(&hcan_, &filterDef));
}

// CanStatus GD_CAN::subscribe(void (*_messageReceiveCallback)())
// {
//     // TODO: subscriptions
//     // GD_CAN::callbackFunction_ = _messageReceiveCallback;
//     // return logStatus('a',
//     //                  HAL_CAN_ActivateNotification(&hcan_, CAN_IT_RX_FIFO0_MSG_PENDING));
//     return CAN_OK;
// }

// CanStatus GD_CAN::unsubscribe()
// {
//     // TODO: subscriptions
//     // callbackFunction_ = nullptr;
//     // return logStatus('u',
//     //                  HAL_CAN_DeactivateNotification(&hcan_, CAN_IT_RX_FIFO0_MSG_PENDING));
//     return CAN_OK;
// }

int GD_CAN::write(CanMsg const &txMsg)
{
#ifdef CAN_DEBUG
    _Serial->print("tx: ");
    txMsg.printTo(*_Serial);
    _Serial->println();

#endif

    // /* initialize transmit message */
    // transmit_message.tx_sfid = 0;
    // transmit_message.tx_efid = 0x000A; // 0x1234;
    // transmit_message.tx_ff = CAN_FF_EXTENDED;
    // transmit_message.tx_ft = CAN_FT_DATA;
    // transmit_message.tx_dlen = 2;
    // transmit_message.tx_data[0] = 0xDE;
    // transmit_message.tx_data[1] = 0xCA;
    // /* transmit a message */

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

    // uint32_t usedMailbox;

    // return logStatus('t',
    //                  HAL_CAN_AddTxMessage(&hcan_, &txHeader_, (uint8_t *)txMsg.data, &usedMailbox));
}

CanMsg GD_CAN::read()
{

    memset(&rxHeader_, 0, sizeof(rxHeader_)); // <-zero before reusing rxHeader_

    uint8_t data[MAX_DATA_LENGTH];

    // if (logStatus('r',
    can_message_receive(hcan_, CAN_FIFO0, &rxHeader_);
    // {
    //     return CanMsg();
    // }
    // else
    // {
    CanMsg const rxMsg(
        (rxHeader_.rx_ff == CAN_FF_EXTENDED) ? CanExtendedId(rxHeader_.rx_efid, rxHeader_.rx_ft == CAN_FT_REMOTE)
                                             : CanStandardId(rxHeader_.rx_sfid, rxHeader_.rx_ft == CAN_FT_REMOTE),
        rxHeader_.rx_dlen,
        data);

#ifdef CAN_DEBUG
    _Serial->print("rx: ");
    rxMsg.printTo(*_Serial);
    _Serial->println();
#endif
    return rxMsg;
    // }
}

size_t GD_CAN::available()
{
    return can_receive_message_length_get(hcan_, CAN_FIFO0);
}

void CAN0_RX0_IRQHandler(void)
{
    // TODO: Fix interrupts approach
    //   can_message_receive(CAN0, CAN_FIFO0, &receive_message);
    //   if ((0x1234 == receive_message.rx_efid) && (CAN_FF_EXTENDED == receive_message.rx_ff) && (2 == receive_message.rx_dlen))
    //   {
    //     test_flag_interrupt = SUCCESS;
    //   }
}
void CAN1_RX0_IRQHandler(void)
{
    // TODO: Fix interrupts approach
    //   can_message_receive(CAN1, CAN_FIFO0, &receive_message);
    //   if ((0x1234 == receive_message.rx_efid) && (CAN_FF_EXTENDED == receive_message.rx_ff) && (2 == receive_message.rx_dlen))
    //   {
    //     test_flag_interrupt = SUCCESS;
    //   }
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
        // _Serial->print(", can_state: ");
        // _Serial->print(HAL_CAN_GetState(&hcan_), HEX); // google HAL_CAN_StateTypeDef e.g. 5 = HAL_CAN_STATE_ERROR
        // _Serial->print(", can_error: ");
        // _Serial->println(HAL_CAN_GetError(&hcan_), HEX); // google CAN_HandleTypeDef::ErrorCode  e.g. 0x00020000U = HAL_CAN_ERROR_TIMEOUT
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