#include "MedianStack.hpp"

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>

#include "LoggerService.hpp"

namespace miscDevices
{
    MedianStack::MedianStack(uint8_t size) : _size(size)
    {
        _writePos = 0;
        void *ptr = pvPortMalloc(sizeof(float) * size);
        if (ptr == nullptr)
        {
            services::LoggerService::fatal("MedianBuffer ctor", "Malloc Failed");
            while (1)
            {
            }
        }
        _buffer = static_cast<float *>(ptr);

        _clearBuffer();
    }

    MedianStack::~MedianStack()
    {
        /* not implemented */
    }

    void MedianStack::_clearBuffer()
    {
        // init buffer with zeros
        for (uint8_t i = 0; i < _size; i++)
        {
            _buffer[i] = 0;
        }

        _writePos = 0;
    }

    bool MedianStack::_isEmpty()
    {
        return (_writePos == 0);
    }

    bool MedianStack::isFull()
    {
        // writePos represents "net Value to Write" - thats why +1
        return (_writePos == (_size + 1));
    }

    int comp(const void *a, const void *b)
    {
        float fa = *(const float *)a;
        float fb = *(const float *)b;
        return (fa > fb) - (fa < fb);
    }

    float MedianStack::getMedian()
    {
        if (_isEmpty())
        {
            return 0.0f;
        }

        // sort buffer
        qsort(_buffer, _size, sizeof(_buffer[0]), comp);

        float median = (_size % 2) ? (_buffer[_size / 2]) : ((_buffer[_size / 2 - 1] + _buffer[_size / 2]) / 2.0f);

        _clearBuffer();
        return median;
    }

    void MedianStack::push(float val)
    {
        // buffer full? well, do nothing
        if (isFull())
        {
            return;
        }

        _buffer[_writePos] = val;
        _writePos++;
    }
}