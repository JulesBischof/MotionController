#ifndef ICM42670_H
#define ICM42670_H

#include "I2cBase.hpp"

namespace i2cDevices
{

    class Icm42670p : public I2cBase
    {
    private:
    protected:
        void _initDevice() override;
        void _checkDevice() override;

    public:
        Icm42670p();
        Icm42670p(i2c_inst_t *i2cInstance, uint8_t i2cAddress);
        ~Icm42670p();

        int16_t getRawDataZaxis();

        float getTemperature();

        float ConvertLsbToDps(int16_t rawVal);

        static float getAngle(int32_t xActualDriver0, int32_t xActualDriver1);
    };

}
#endif //ICM42670_H