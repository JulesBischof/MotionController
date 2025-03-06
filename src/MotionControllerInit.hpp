#ifndef MOTIONCONTROLLERINIT_H
#define MOTIONCONTROLLERINIT_H

#include "pico/stdlib.h"

#include "hardware/i2c.h"
#include "hardware/spi.h"

#include "MotionControllerPinning.h"
#include "MotionControllerConfig.h"

void MotionControllerInit(void);

void initUArt(void);
void initSPI(void);
void initI2C(void);

#endif // MOTIONCONTROLLERINIT_H