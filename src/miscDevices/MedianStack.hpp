#pragma once

#include "pico/stdlib.h"

namespace miscDevices
{
    template <typename T>
    class MedianStack
    {
    private:
        uint8_t _writePos, _size;

        T *_buffer;

        void _clearBuffer();
        bool _isEmpty();

    public:
        MedianStack(){};
        MedianStack(uint16_t size);
        ~MedianStack();

        bool isFull();

        T getMedian();
        void push(T val);
    };
}