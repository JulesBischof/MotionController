#ifndef TESTS_H
#define TESTS_H

#include "TestConfig.hpp"
#include "MotionControllerConfig.hpp"

#include <stdio.h>
#include "pico/stdlib.h"

#include "hardware/i2c.h"

#include "Icm42670pTest.hpp"
#include "Tmc5240Test.hpp"
#include "Tla2528Test.hpp"
#include "LineSensorTest.hpp"
#include "HcSr04Test.hpp"
#include "LineFollowerTaskTest.hpp"
#include "UartConfigTest.hpp"

void testApp(void);

void i2cBroadcast(void);

#endif // TESTS_H