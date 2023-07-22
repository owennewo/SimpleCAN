#pragma once

#include "Arduino.h"
#include "stm32f4xx_hal_can.h"

// Value needed for prescaler. depends 
// on CLK configuration
enum CanSpeed
{
	BAUD_1M = 0,
	BAUD_500K,
	BAUD_250K,
	BAUD_125K,
	BAUD_100K,
	BAUD_50K,
	BAUD_20K,
};

struct can_timing {
	uint32_t Prescaler;
	uint32_t TimeSeg1;
	uint32_t TimeSeg2;
}

// these timings assume 42MHz PCKL1 which is common for 168Mhz HCKL
const can_timings[] {
	{.Prescaler=3,.TimeSeg1=11,.TimeSeg2=2}, // BAUD_1M
	{.Prescaler=6,.TimeSeg1=11,.TimeSeg2=2}, // BAUD_500K
	{.Prescaler=12,.TimeSeg1=11,.TimeSeg2=2}, // BAUD_250K
	{.Prescaler=21,.TimeSeg1=13,.TimeSeg2=2}, // BAUD_125K
	{.Prescaler=28,.TimeSeg1=12,.TimeSeg2=2}, // BAUD_100K
	{.Prescaler=56,.TimeSeg1=12,.TimeSeg2=2}, // BAUD_50K
	{.Prescaler=140,.TimeSeg1=12,.TimeSeg2=2}, // BAUD_20K
};

enum CanMode
{
	NormalCAN = CAN_MODE_NORMAL,
	LoopBackCan = CAN_MODE_LOOPBACK,
	// SilentCAN = CAN_MODE_SILENT,
	// SilentLoopBackCAN = CAN_MODE_SILENT_LOOPBACK
};

typedef enum
{
  CAN_OK       = 0x00U,
  CAN_ERROR    = 0x01U,
  CAN_BUSY     = 0x02U,
  CAN_TIMEOUT  = 0x03U
} CAN_Status;


typedef struct{
  uint8_t dlc;
  uint32_t id;
  bool isRTR;
  bool isStandard;
  uint8_t data[8];
} can_message_t;

typedef std::function<void(can_message_t*)> can_callback_function_t;

static CAN_FilterTypeDef FILTER_ACCEPT_ALL = {
    .FilterMaskIdHigh = 0,
    .FilterMaskIdLow = 0,
    .FilterFIFOAssignment = CAN_FILTER_FIFO0,
    .FilterMode = CAN_FILTERMODE_IDMASK,
    .FilterScale= CAN_FILTERSCALE_32BIT,
    .FilterActivation = ENABLE,
};

class SimpleCAN
{
public:

	SimpleCAN();
	CAN_Status init(int rx_pin, int tx_pin, CanSpeed speed = BAUD_500K, CanMode mode = NormalCAN);
	CAN_Status filter(CAN_FilterTypeDef *filter);
	CAN_Status filterAcceptAll();
	CAN_Status subscribe(can_callback_function_t function);
	CAN_Status subscribe2(void (*_receive) (can_message_t * message) = nullptr);
	CAN_Status unsubscribe();
	CAN_Status begin();
	CAN_Status stop();
	CAN_Status transmit(can_message_t* txMessage);
	CAN_Status receive(can_message_t* rxMessage);
	static void _receive(can_message_t* rxMessage);
	//static void(*receiveCallback)(can_message_t* rxMessage);
	static can_callback_function_t receiveFunction;
	
	static CAN_HandleTypeDef* _hcan;
private: 
	void createCanHandle(CanSpeed speed, CanMode mode);
};

