#include "Tmc5240Test.h"

#include "Tmc5240.hpp"
#include "MotionControllerConfig.h"
#include "MotionControllerPinning.h"

void Tmc5240Test(void)
{
    Tmc5240 Driver_0 = Tmc5240(TMC5240_SPI_INSTANCE, SPI_CS_DRIVER_0);

//    Tmc5240 Driver_1 = Tmc5240(TMC5240_SPI_INSTANCE, SPI_CS_DRIVER_1);
}