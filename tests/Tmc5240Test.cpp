#include "Tmc5240Test.hpp"

#include "TestConfig.hpp"

#include "Tmc5240.hpp"
#include "MotionControllerConfig.hpp"
#include "MotionControllerPinning.hpp"

void Tmc5240Test(void)
{
    spiDevices::Tmc5240 Driver_0 = spiDevices::Tmc5240(TMC5240_SPI_INSTANCE, SPI_CS_DRIVER_0, false);

    spiDevices::Tmc5240 Driver_1 = spiDevices::Tmc5240(TMC5240_SPI_INSTANCE, SPI_CS_DRIVER_1, true);

#if TEST_TMC5260_VELOCITY_MODE == 1
    Driver_0.moveVelocityMode(0, 200000, 5000);
    Driver_1.moveVelocityMode(0, 200000, 5000);
    vTaskDelay(pdMS_TO_TICKS(1000));


    Driver_0.moveVelocityMode(0, 0, 5000);
    Driver_1.moveVelocityMode(0, 0, 5000);
    vTaskDelay(pdMS_TO_TICKS(1000));
    #endif

    #if TEST_TMC5260_POSITION_MODE == 1
    Driver_0.moveRelativePositionMode(51200, 200000, 5000);
    Driver_1.moveRelativePositionMode(51200, 200000, 5000);

    vTaskDelay(pdMS_TO_TICKS(1000));

    Driver_0.moveRelativePositionMode(-51200, 200000, 5000);
    Driver_1.moveRelativePositionMode(-51200, 200000, 5000);
    #endif
}