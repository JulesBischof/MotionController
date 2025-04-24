#include "MovementTracker.hpp"

#include "StepperService.hpp"

#include <cmath>

MtnCtrl::MovementTracker::MovementTracker(spiDevices::Tmc5240 *driver0, spiDevices::Tmc5240 *driver1)
: _driver0(driver0), _driver1(driver1)
{
    init();
}

MtnCtrl::MovementTracker::~MovementTracker()
{
}

void MtnCtrl::MovementTracker::init()
{
    int32_t xActualDriver0 = _driver0->getXActual();
    int32_t xActualDriver1 = _driver1->getXActual();

    _rotationXActualStartDriver0 = xActualDriver0;
    _rotationXActualStartDriver1 = xActualDriver1;

    _distanceXActualStartDriver0 = xActualDriver0;
    _distanceXActualStartDriver1 = xActualDriver1;
}

void MtnCtrl::MovementTracker::resetRotation()
{
    _rotationXActualStartDriver0 = _driver0->getXActual();
    _rotationXActualStartDriver1 = _driver1->getXActual();
}

void MtnCtrl::MovementTracker::resetDistance()
{
    _distanceXActualStartDriver0 = _driver0->getXActual();
    _distanceXActualStartDriver1 = _driver1->getXActual();
}

int32_t MtnCtrl::MovementTracker::getRotation()
{
    int32_t drivenDistance0 = _rotationXActualStartDriver0 - _driver0->getXActual();
    int32_t drivenDistance1 = _rotationXActualStartDriver1 - _driver1->getXActual();

    float phi = services::StepperService::convertDeltaDrivenDistanceToDegree(drivenDistance0 - drivenDistance1);

    // 10 due to prain uart expects rotation in form of 10 * degree
    return static_cast<int32_t>(round(phi * 10));
}

int32_t MtnCtrl::MovementTracker::getDistance()
{
    int32_t xActualDriver0 = _driver0->getXActual();
    int32_t xActualDriver1 = _driver1->getXActual();

    int32_t average = ( (xActualDriver0 - _rotationXActualStartDriver0) + (xActualDriver1 - _rotationXActualStartDriver1) / 2);

    return services::StepperService::convertMicrostepsToMillimeter(average);
}