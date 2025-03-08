#include "queues.hpp"

/* -------------------------------------------------- */
/*              definition global queues              */
/* -------------------------------------------------- */
QueueHandle_t xDispatcherQueue = NULL;
QueueHandle_t xLineFollowerTaskQueue = NULL;
QueueHandle_t xCom0TaskQueue = NULL;

/// @brief initializes global queues
/// @param  void
void vInitQueues(void)
{
    xDispatcherQueue = xQueueCreate(10, sizeof(dispatcherMessage_t));
    xLineFollowerTaskQueue = xQueueCreate(10, sizeof(dispatcherMessage_t));
    xCom0TaskQueue = xQueueCreate(10, sizeof(dispatcherMessage_t));
}

/* -------------------------------------------------- */
/*                 getters global queues              */
/* -------------------------------------------------- */

/// @brief returns the dispatcher queue
/// @param  void
/// @return Handle to dispatcher queue
QueueHandle_t getDispatcherTaskQueue(void)
{
    return xDispatcherQueue;
}

/// @brief returns the line follower queue
/// @param  void
/// @return Handle to dispatcher queue
QueueHandle_t getLineFollowerTaskQueue(void)
{
    return xLineFollowerTaskQueue;
}

/// @brief returns the com0 queue
/// @param  void
/// @return Handle to Com0TaskQueue
QueueHandle_t getCom0TaskQueue(void)
{
    return xCom0TaskQueue;
}
