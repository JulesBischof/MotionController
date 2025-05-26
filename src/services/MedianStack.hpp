#pragma once

#include "pico/stdlib.h"

namespace services
{
    /// @brief MedianStack stores values in a buffer and provides median calculation.
    ///
    /// The buffer has a fixed size and is automatically cleared after each median calculation.
    /// Memory is dynamically allocated (FreeRTOS-compatible).
    ///
    /// @tparam T Data type of the stored values (e.g., int, float).
    template <typename T>
    class MedianStack
    {
    private:
        uint8_t _writePos;
        uint8_t _size;

        T *_buffer; ///< Pointer to the dynamically allocated buffer.

        /// @brief Checks if the buffer is empty.
        /// @return true if no values are stored, false otherwise.
        bool _isEmpty();

    public:
        /// @brief Default constructor. Creates an empty MedianStack.
        MedianStack() {};

        /// @brief Constructor with defined buffer size.
        /// @param size Number of values that can be stored in the buffer.
        MedianStack(uint16_t size);

        /// @brief Destructor. Frees the allocated buffer memory.
        ~MedianStack();

        /// @brief Clears the buffer and resets all values to zero.
        void clearBuffer();

        /// @brief Checks if the buffer is full.
        /// @return true if no more values can be written, false otherwise.
        bool isFull();

        /// @brief Calculates and returns the median of stored values.
        ///
        /// The buffer is sorted internally. After calculation, the buffer is cleared.
        /// @return The median value of stored entries. Returns 0 if the buffer is empty.
        T getMedian();

        /// @brief Adds a value to the buffer.
        ///
        /// If the buffer is full, the value is discarded.
        /// @param val The value to add.
        void push(T val);

        /// @brief Copies the buffer content to an external array.
        ///
        /// @param pDest Pointer to the destination array.
        /// @return true if copy is successful, false if destination or buffer is null.
        bool cpyBufferToArray(T *pDest);
    };
}

#include "MedianStack.tpp"