#pragma once

namespace MtnCtrl
{
    enum class DispatcherTaskId : uint8_t
    {
        NoTask,
        Broadcast,
        DispatcherTask,
        LineFollowerTask,
        RaspberryHatComTask,
        GripControllerComTask,
        BarrierHandlerTask,
    };

    enum class TaskCommand : uint8_t
    {
        NoCommand,
        Move,
        SlowDown,
        PositionReached,
        Turn,
        Stop,
        Info,
        BarrierDetectedInfo,
        NodeDetectedInfo,
        LostLineInfo,
        SafetyButtonInfo,
        Ping,
        Pong,
        Error,
        PollDistance,
        PollLineSensor,
        PollDegree,
        PollStatusFlags,
        PollUltrasonic,
        HandThroughMessage,
        DecodeMessage,
        CraneGrip,
        CraneRelease,
        GcAck
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

        /// @brief constructor creates a Dispatcher Message MessageBlock with default values
        DispatcherMessage()
            : senderTaskId(DispatcherTaskId::NoTask),
              receiverTaskId(DispatcherTaskId::NoTask),
              command(TaskCommand::NoCommand),
              data{0, 0}
        {
        }

        /// @brief creates a Dispatcher Messager 
        /// @param sender sender Task ID
        /// @param receiver reciever Task ID
        /// @param cmd Command to Send
        /// @param d Data to send (64bit)
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