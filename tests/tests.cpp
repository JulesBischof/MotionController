#include "tests.h"

void testTask(void *pvParameters)
{
        while (1)
        {
                printf("TestMode is active \n");
                printf("Running tests...\n");

#if BROADCAST_I2C_DEVICES == 1
                printf("Broadcasting I2C devices...\n");
                i2cBroadcast();
#endif
#if TEST_TMC5240 == 1
                printf("Running TMC5240 Test...\n");
                Tmc5240Test();
#endif
#if TEST_ICM42670 == 1
                printf("Running ICM42670 Test...\n");
                Icm42670Test();
#endif
#if TEST_TLA2528 == 1
                printf("Running TLA2528 Test...\n");
                Tla2528Test();
#endif
#if TEST_LINESENSOR == 1
                printf("Running LineSensor Test...\n");
                LineSensorTest();
#endif
#if TEST_HCSR04 == 1
                printf("Running HcSr04 Test...\n");
                HcSr04Test();
#endif
#if TEST_ARDUINO_ADC == 1
                printf("Running ArduinoNanoAdcUartSlave Test...\n");
                ArduinoAdcSlaveTest();
#endif
#if TEST_VLINEFOLLOWERTASK == 1
                printf("Running vLineFollowerTask Test...\n");
                LineFollowerTaskTest();
#endif
#if TEST_UART_IRQ_CONFIGURATION == 1
                printf("running UART-IRQ Config Test");
                UartConfigTest();
#endif
                printf("Tests done - LOOP \n");
                vTaskDelay(pdMS_TO_TICKS(1000));
        }
}

// I2C reserves some addresses for special purposes. We exclude these from the scan.
// These are any addresses of the form 000 0xxx or 111 1xxx
bool reserved_addr(uint8_t addr)
{
        return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

/// @brief performs an i2c Broadcast to find all devices on the i2c bus
/// @param  void
void i2cBroadcast(void)
{
        // Sweep through all 7-bit I2C addresses, to see if any slaves are present on
        // the I2C bus. Print out a table that looks like this:
        //
        // I2C Bus Scan
        //    0 1 2 3 4 5 6 7 8 9 A B C D E F
        // 00 . . . . . . . . . . . . . . . .
        // 10 . . @ . . . . . . . . . . . . .
        // 20 . . . . . . . . . . . . . . . .
        // 30 . . . . @ . . . . . . . . . . .
        // 40 . . . . . . . . . . . . . . . .
        // 50 . . . . . . . . . . . . . . . .
        // 60 . . . . . . . . . . . . . . . .
        // 70 . . . . . . . . . . . . . . . .
        // E.g. if addresses 0x12 and 0x34 were acknowledged.

        printf("\nI2C Bus Scan\n");
        printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

        for (int addr = 0; addr < (1 << 7); ++addr)
        {
                if (addr % 16 == 0)
                {
                        printf("%02x ", addr);
                }

                // Perform a 1-byte dummy read from the probe address. If a slave
                // acknowledges this address, the function returns the number of bytes
                // transferred. If the address byte is ignored, the function returns
                // -1.

                // Skip over any reserved addresses.
                int ret;
                uint8_t rxdata;
                if (reserved_addr(addr))
                        ret = PICO_ERROR_GENERIC;
                else
                        ret = i2c_read_blocking(I2C_INSTANCE_DEVICES, addr, &rxdata, 1, false);

                printf(ret < 0 ? "." : "@");
                printf(addr % 16 == 15 ? "\n" : "  ");
        }
        printf("Done.\n");
}

// entry point for tests
void testApp(void)
{
        // create test-task - some Devices depend on running inside a task
        xTaskCreate(testTask, "TestTask", 2048, NULL, 1, NULL);
        // start scheduler
        vTaskStartScheduler();
        while (1)
                ;
        return;
}