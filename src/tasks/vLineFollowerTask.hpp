#ifndef V_LINE_FOLLOWER_TASK_H
#define V_LINE_FOLLOWER_TASK_H

#include "MotionControllerPinning.h"
#include "MotionControllerConfig.h"
#include "LineFollowerTaskConfig.h"
#include "digitalInput.hpp"

#include "queues.hpp"

#include "Tmc5240.hpp"
#include "LineSensor.hpp"

void followLine(Tmc5240 *Driver0, Tmc5240 *Driver1, LineSensor *lineSensor, uint32_t *flags);

int32_t ControllerC(int8_t e);

void stopDrives(Tmc5240 *Driver0, Tmc5240 *Driver1, uint32_t *flags);

void vLineFollowerTask(void *pvParameters);

#endif // V_LINE_FOLLOWER_TASK_H