#ifndef CAR_SIGNALS_H
#define CAR_SIGNALS_H

#include "Car_Types.h"
#include "CanBus.h"

/**
 * @brief Parse a CAN/FDCAN frame using the current vehicle profile.
 * @return 1 when at least one signal was decoded, otherwise 0
 */
uint8_t Car_ParseFrame(const CAN_RxFrame_t *frame);
uint8_t Car_ProcessRxFrame(const CAN_RxFrame_t *frame);
void Car_ParseCAN(uint32_t id, const uint8_t *data);
void Car_ResetDiagState(void);
const Car_DiagState_t *Car_GetDiagState(void);
void Car_RebuildSignalMonitor(void);
const Car_SignalMonitorTable_t *Car_GetSignalMonitor(void);
const char *Car_GetSignalNameToken(CAN_SignalName_t signal_name);

#endif // CAR_SIGNALS_H
