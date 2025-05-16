#include "TestShell.hpp"

#include "stdio.h"
#include "string.h"

#include "LoggerService.hpp"

#include "pico/stdlib.h"

using namespace MtnCtrl;

namespace services
{
    void TestShell::_handleCmd(TaskCommand cmd)
    {
        switch (cmd)
        {
        case (TaskCommand::CraneGrip):
        {
            DispatcherMessage msg(DispatcherTaskId::RaspberryHatComTask,
                                  DispatcherTaskId::GripControllerComTask,
                                  TaskCommand::CraneGrip,
                                  0);
            xQueueSendToFront(_messageDispatcherQueue, &msg, portMAX_DELAY);
        }
        break;
        case (TaskCommand::CraneRelease):
        {
            DispatcherMessage msg(DispatcherTaskId::RaspberryHatComTask,
                                  DispatcherTaskId::GripControllerComTask,
                                  TaskCommand::CraneRelease,
                                  0);
            xQueueSendToFront(_messageDispatcherQueue, &msg, portMAX_DELAY);
        }
        break;
        case (TaskCommand::NoCommand):
            printMenu();
            break;
        default:
            printf("INVALID CMD!");
            break;
        }
    }

    MtnCtrl::TaskCommand TestShell::_parseCommand(const char *msg)
    {
        if (strcmp(msg, (const char *)"help"))
        {
            return TaskCommand::NoCommand;
        }
        if (strcmp(msg, (const char *)"grip"))
        {
            return TaskCommand::CraneGrip;
        }
        if (strcmp(msg, (const char *)"release"))
        {
            return TaskCommand::CraneRelease;
        }
        else
            return TaskCommand::NoCommand;
    }

#define BUFFERSIZE 100
    void TestShell::_shellTask()
    {
        char buffer[BUFFERSIZE];
        for (;;)
        {
            printMenu();
            printf("> \n");

            int ch = 0;
            int i = 0;

            while (true)
            {
                ch = getchar_timeout_us(0); // Sofortiger Check ohne Block

                if (ch != PICO_ERROR_TIMEOUT)
                {
                    // \r oder \n = Eingabeende
                    if (ch == '\n' || ch == '\r')
                    {
                        break;
                    }
                    if (i < sizeof(buffer) - 1)
                    {
                        buffer[i++] = (char)ch;
                    }
                }
                else
                {
                    // Kein Zeichen -> gib CPU ab
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            }

            // Erstes Zeichen erhalten
            buffer[i++] = (char)ch;

            // Weitere Zeichen lesen
            while (i < sizeof(buffer) - 1)
            {
                ch = getchar_timeout_us(0);
                if (ch == PICO_ERROR_TIMEOUT)
                {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    continue;
                }
                if (ch == '\n' || ch == '\r')
                    break;

                buffer[i++] = (char)ch;
            }

            buffer[i] = '\0'; // Nullterminator

            printf("recieved: %s\n", buffer);

            TaskCommand cmd = _parseCommand((const char *)buffer);
            _handleCmd(cmd);
        }
    }

    void TestShell::_shellTaskWrapper(void *pv)
    {
        TestShell *inst = NULL;
        if (pv != NULL)
        {
            inst = (TestShell *)pv;
            inst->_shellTask();
        }
    }

    void TestShell::_startShellTask()
    {
        BaseType_t ret = xTaskCreate(
            _shellTaskWrapper,
            "ShellTask",
            1000 / sizeof(StackType_t),
            this,
            tskIDLE_PRIORITY + 1,
            NULL);

        if (ret == pdFALSE)
        {
            services::LoggerService::fatal("_startShellTask()", "TASK CREATION FAILED");
        }
    }

    TestShell::TestShell(QueueHandle_t messagedispatcherQueue) : _messageDispatcherQueue(messagedispatcherQueue)
    {
        _startShellTask();
    }

    TestShell::~TestShell()
    {
    }

    void TestShell::printMenu()
    {
        printf("MotionControllerShell ---- helpscreen \n");

        printf("possible commands... \n");
        printf("help         -------- print this menu");
        printf("grip/release -------- crane");
    }
}