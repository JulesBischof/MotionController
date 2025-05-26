#include "MedianStack.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <cstring>
#include "LoggerService.hpp"

namespace services
{

    template<typename T>
    MedianStack<T>::MedianStack(uint16_t size) : _size(size)
    {
        _writePos = 0;
        void *ptr = pvPortMalloc(sizeof(T) * size);
        if (ptr == nullptr)
        {
            services::LoggerService::fatal("MedianStack ctor", "Malloc Failed");
            while (1)
            {
            }
        }
        _buffer = static_cast<T *>(ptr);

        clearBuffer();
    }

    template <typename T>
    MedianStack<T>::~MedianStack()
    {
        services::LoggerService::debug("MedianStack dctor", "free buffer Memory");
        if (_buffer != nullptr)
        {
            vPortFree(_buffer);
            _buffer = nullptr;
        }
        else
        {
            services::LoggerService::error("MedianStack dctor", "Buffer already freed");
        }
    }

    template <typename T>
    void MedianStack<T>::clearBuffer()
    {
        // init buffer with zeros
        for (uint8_t i = 0; i < _size; i++)
        {
            _buffer[i] = 0;
        }

        _writePos = 0;
    }

    template <typename T>
    bool MedianStack<T>::_isEmpty()
    {
        return (_writePos == 0);
    }

    template <typename T>
    bool MedianStack<T>::isFull()
    {
        // writePos represents "net Value to Write" - thats why +1
        return (_writePos >= _size);
    }

    template <typename T>
    int comp(const void *a, const void *b)
    {
        T fa = *(const T *)a;
        T fb = *(const T *)b;
        return (fa > fb) - (fa < fb);
    }

    template<typename T>
    T MedianStack<T>::getMedian()
    {
        if (_isEmpty())
        {
            return 0.0f;
        }

        // sort buffer
        qsort(_buffer, _size, sizeof(_buffer[0]), comp<T>);

        float median = (_size % 2) ? (_buffer[_size / 2]) : ((_buffer[_size / 2 - 1] + _buffer[_size / 2]) / 2.0f);

        clearBuffer();
        return median;
    }

    template<typename T>
    void MedianStack<T>::push(T val)
    {
        // buffer full? well, do nothing
        if (isFull())
        {
            return;
        }

        _buffer[_writePos++] = val;
    }

    template <typename T>
    bool MedianStack<T>::cpyBufferToArray(T *pDest)
    {
        if (pDest == nullptr || _buffer == nullptr)
        {
            return false;
        }
        memcpy(pDest, _buffer, sizeof(_buffer));
        return true;
    }
}