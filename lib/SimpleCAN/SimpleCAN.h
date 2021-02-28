#ifndef __CAN__H
#define __CAN__H

#include "Arduino.h"
#include "stm32f4xx_hal_can.h"


// Value needed for prescaler. depends 
// on CLK configuration
enum CanSpeed
{
	Kbit500 = 5,
	Kbit250 = 10,
	Kbit125 = 20,
	Kbit100 = 25,
  Kbit50 =  50,
  Kbit20  = 125

};

enum CanMode
{
	NormalCAN = CAN_MODE_NORMAL,
	SilentCAN = CAN_MODE_SILENT,
	LoopBackCan = CAN_MODE_LOOPBACK,
	SilentLoopBackCAN = CAN_MODE_SILENT_LOOPBACK
};


typedef struct{
  uint8_t dlc;
  uint32_t msgID;
  bool isRTR;
  bool isStandard;
  uint8_t data[8];
}CanMessage;


CanMessage createMessage(void);
/**
 CAN wrapper for STM32F405RGT6 board.
*/
class SimpleCan
{
public:
	class RxHandler
	{
	public:
		RxHandler(uint16_t dataLength, void(*callback)(CAN_RxHeaderTypeDef rxHeader, uint8_t *rxData));
		~RxHandler();
		void notify(CAN_HandleTypeDef *hcan1);
	
  private:
		CAN_RxHeaderTypeDef _rxHeader;
		uint8_t *_rxData;
		void(*_callback)(CAN_RxHeaderTypeDef, uint8_t *);
	};

	SimpleCan();
	HAL_StatusTypeDef init(CanSpeed speed, CanMode mode);
	HAL_StatusTypeDef configFilter(CAN_FilterTypeDef *filterDef);
  HAL_StatusTypeDef configSnifferFilter();
	HAL_StatusTypeDef activateNotification(RxHandler *rxHandler);
	HAL_StatusTypeDef deactivateNotification();
	HAL_StatusTypeDef begin();
	HAL_StatusTypeDef stop();
	HAL_StatusTypeDef send(CanMessage message);
  //private: interrupt handler needs access to it
  CAN_HandleTypeDef hcan;
  static RxHandler *_rxHandler;

};
#endif