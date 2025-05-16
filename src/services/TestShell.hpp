#pragma once

#include "FreeRTOS.h"
#include "semphr.h"

#include "DispatcherMessage.hpp"

namespace services
{
    class TestShell
    {
        private:
            void _handleCmd(MtnCtrl::TaskCommand cmd);

            MtnCtrl::TaskCommand _parseCommand(const char* msg);

            void _shellTask();
            static void _shellTaskWrapper(void *pv);
            void _startShellTask();
            QueueHandle_t _messageDispatcherQueue;

        public:
            TestShell(QueueHandle_t messagedispatcherQueue);
            TestShell(){};
            ~TestShell();

            void printMenu();
    };
}