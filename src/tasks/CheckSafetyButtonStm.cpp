#include "CheckSafetyButtonStm.hpp"

#include "MotionController.hpp"


#include "LineFollowerTaskConfig.h"
#include "LineFollowerTaskStatusFlags.hpp"


namespace MotionController
{
    CheckSafetyButtonStm::CheckSafetyButtonStm(uint32_t *_statusFlags, DigitalInput *safetyButton, QueueHandle_t lineFollowerTaskQueue) : 
        StmBase(_statusFlags),
        _safetyButton(safetyButton),
        _lineFollowerTaskQueue(lineFollowerTaskQueue)
    {
    }

    CheckSafetyButtonStm::~CheckSafetyButtonStm()
    {
    }

    void CheckSafetyButtonStm::init()
    {
        _state = State::WAIT_FOR_BUTTON;
    }
    
    bool CheckSafetyButtonStm::run()
    {
        bool retVal = false;
        switch (_state)
        {
        case State::WAIT_FOR_BUTTON:
            if (_safetyButton->getValue())
            {
                _state = State::BUTTON_PRESSED;
                retVal = true;
            }
            break;

        case State::BUTTON_PRESSED:
            // stop drives
            DispatcherMessage msg(
                DispatcherTaskId::LineFollowerTask,
                DispatcherTaskId::LineFollowerTask,
                TaskCommand::Stop,
                LINEFOLLOWERCONFIG_DISTANCE_LINESENSOR_TO_AXIS_mm / 10); // convert to cm
            if (xQueueSend(_lineFollowerTaskQueue, &msg, pdMS_TO_TICKS(10)) != pdPASS)
            { /* ERROR!!?? */
            }
            *_statusFlags |= SAFETY_BUTTON_PRESSED;
            break;
        }
        retVal = false;
    }

    void CheckSafetyButtonStm::reset()
    {
        _state = State::WAIT_FOR_BUTTON;
    }

    void CheckSafetyButtonStm::update(uint32_t msgData)
    {   
        // nothing to do in here
    }
}