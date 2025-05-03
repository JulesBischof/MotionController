#pragma once

#include "pico/stdlib.h"

namespace miscDevices
{
    class MedianStack
    {
    private:
        uint8_t _writePos, _size;

        float *_buffer;

        void _clearBuffer();
        bool _isEmpty();

    public:
        MedianStack(uint8_t size);
        ~MedianStack();

        bool isFull();

        float getMedian();
        void push(float val);
    };
}