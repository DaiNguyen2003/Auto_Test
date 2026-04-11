#ifndef CAR_SIGNALS_H
#define CAR_SIGNALS_H

#include "Car_Types.h"
#include "CanBus.h"

/**
 * @brief Parse a CAN/FDCAN frame using the current vehicle profile.
 * @return 1 when at least one signal was decoded, otherwise 0
 */
uint8_t Car_ParseFrame(const CAN_RxFrame_t *frame);
void Car_ParseCAN(uint32_t id, const uint8_t *data);

#endif // CAR_SIGNALS_H
