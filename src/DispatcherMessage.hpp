#pragma once

namespace nMotionController
{
    enum class DispatcherTaskId : uint8_t
    {
        NoTask,
        DispatcherTask,
        LineFollowerTask,
        RaspberryHatComTask,
        GripControllerComTask
    };

    enum class TaskCommand : uint8_t
    {
        NoCommand,
        Move,
        Reverse,
        Turn,
        Stop,
        Info,
        Ping,
        Pong,
        Error,
        PollDistance,
        PollLineSensor,
        PollDegree,
        PollStatusFlags,
        HandThroughMessage,
        DecodeMessage
    };

    enum class TaskInfo : uint8_t
    {
        NoInfo,
        LineFollowerLostLine,
        LineFollowerCrosspointDetected,
        GripControllerReady,
        GripControllerNotReady
    };

    /// @brief struct containing Messages for intertask communication
    /// data is split into 2 32bit values due to memory allignment
    /// difficulties in the beginnning. Defining one uint64_t for data
    /// leads to hardfaults
    struct DispatcherMessage
    {
        DispatcherTaskId senderTaskId;
        DispatcherTaskId receiverTaskId;
        TaskCommand command;
        alignas(8) uint32_t data[2];

        DispatcherMessage()
            : senderTaskId(DispatcherTaskId::NoTask),
              receiverTaskId(DispatcherTaskId::NoTask),
              command(TaskCommand::NoCommand),
              data{0, 0}
        {
        }
        DispatcherMessage(DispatcherTaskId sender, DispatcherTaskId receiver, TaskCommand cmd, uint64_t d)
            : senderTaskId(sender), receiverTaskId(receiver), command(cmd) { setData(d); }

        uint64_t getData() const { return ((uint64_t)data[1] << 32) | data[0]; }

        void setData(uint64_t d)
        {
            data[0] = (uint32_t)(d & 0xFFFFFFFF);
            data[1] = (uint32_t)((d >> 32) & 0xFFFFFFFF);
        }
    };
}