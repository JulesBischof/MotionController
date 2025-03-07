#ifndef V_LINE_FOLLOWER_TASK_H
#define V_LINE_FOLLOWER_TASK_H

#include "MotionControllerPinning.h"
#include "MotionControllerConfig.h"

#include "queues.hpp"

#include "Tmc5240.hpp"
#include "LineSensor.hpp"

void vLineFollowerTask(void *pvParameters);

#endif // V_LINE_FOLLOWER_TASK_H