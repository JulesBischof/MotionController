#ifndef TESTS_H
#define TESTS_H

#include "TestConfig.h"
#include "MotionControllerConfig.h"

#include <stdio.h>
#include "pico/stdlib.h"

#include "hardware/i2c.h"

#include "Icm42670pTest.h"
#include "Tmc5240Test.h"
#include "Tla2528Test.h"
#include "LineSensorTest.h"
#include "HcSr04Test.h"
#include "LineFollowerTaskTest.hpp"
#include "UartConfigTest.h"

void testApp(void);

void i2cBroadcast(void);

#endif // TESTS_H