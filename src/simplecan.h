#if defined(ARDUINO_ARCH_ESP32)
#include "esp/twai/can.h"
#elif defined(HAL_CAN_MODULE_ENABLED)
#include "stm/can/can.h"
#elif defined(HAL_FDCAN_MODULE_ENABLED)
#include "stm/fdcan/can.h"
#else
#error "No CAN module is enabled, expecting a define for ARDUINO_ARCH_ESP32 | HAL_CAN_MODULE_ENABLED | HAL_FDCAN_MODULE_ENABLED"
#endif