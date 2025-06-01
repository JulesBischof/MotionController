#pragma once

namespace services
{
    template <typename T>
    class RingBuffer
    {
    private:
        uint32_t _writePos; ///< Current write position in the buffer.
        uint32_t _readPos;  ///< Current read position in the buffer.
        uint32_t _size;     ///< Size of the buffer.
        bool _enableOverflow; ///< If true, allows overwriting old values when the buffer is full.
        T *_buffer;  ///< Pointer to the dynamically allocated buffer.
        bool _isEmpty();  ///< Checks if the buffer is empty.
    public:
        /// @brief Default constructor. Creates an empty RingBuffer.
        RingBuffer() {};

        /// @brief Constructor with defined buffer size.
        /// @param size Number of values that can be stored in the buffer.
        /// @param enableOerflow If true, allows overwriting old values when the buffer is full.
        /// @note If enableOerflow is false, the buffer will not accept new values when full.
        RingBuffer(uint32_t size, bool enableOerflow = true);

        /// @brief Destructor. Frees the allocated buffer memory.
        ~RingBuffer();

        /// @brief Clears the buffer and resets all values to zero.
        void clearBuffer();

        /// @brief Adds a value to the buffer.
        /// @param val The value to add.
        void push(T val);

        /// @brief checks if the buffer is full
        /// @return true if no more values can be written, false otherwise
        bool isFull();

        /// @brief Reads the oldest value from the buffer
        T pop();

        /// @brief sums up all values in the buffer
        T getSum();
    };
}

#include "RingBuffer.tpp"