#include "RingBuffer.hpp" 
#include "LoggerService.hpp"

namespace services
{
    template <typename T>
    RingBuffer<T>::RingBuffer(uint32_t size, bool enableOverflow) : _size(size), _enableOverflow(enableOverflow)
    {
        _writePos = 0;
        _readPos = 0;

        void *ptr = pvPortMalloc(sizeof(T) * size);
        if (ptr == nullptr)
        {
            services::LoggerService::fatal("RingBuffer ctor", "Malloc Failed");
            while (1)
            {
            }
        }
        _buffer = static_cast<T *>(ptr);
        clearBuffer();
    }

    template <typename T>
    RingBuffer<T>::~RingBuffer()
    {
        if (_buffer != nullptr)
        {
            vPortFree(_buffer);
            _buffer = nullptr;
        }
    }

    template <typename T>
    void RingBuffer<T>::clearBuffer()
    {
        _writePos = 0;
        _readPos = 0;

        // init buffer with zeros
        for (uint32_t i = 0; i < _size; i++)
        {
            _buffer[i] = 0;
        }
        services::LoggerService::debug("RingBuffer::clearBuffer", "Buffer cleared");
    }

    template <typename T>
    void RingBuffer<T>::push(T val)
    {
        if (_enableOverflow || !isFull())
        {
            _buffer[_writePos] = val;
            _writePos = (_writePos + 1) % _size;
            if (_writePos == _readPos) // If we wrapped around, move read position forward
            {
                _readPos = (_readPos + 1) % _size;
            }
            services::LoggerService::debug("RingBuffer::push", "Value added to buffer");
        }
        else
        {
            services::LoggerService::warn("RingBuffer::push", "Buffer is full, value not added");
        }
    }

    template <typename T>
    bool RingBuffer<T>::isFull()
    {
        return ((_writePos + 1) % _size == _readPos);
    }

    template <typename T>
    T RingBuffer<T>::pop()
    {
        if (_isEmpty())
        {
            services::LoggerService::warn("RingBuffer::pop", "Buffer is empty, returning 0");
            return 0;
        }

        T val = _buffer[_readPos];
        _readPos = (_readPos + 1) % _size;
        return val;
    }

    template <typename T>
    T RingBuffer<T>::getSum()
    {
        T sum = 0;
        for (uint32_t i = 0; i < _size; i++)
        {
            sum += _buffer[i];
        }
        return sum;
    }

    template <typename T>
    bool RingBuffer<T>::_isEmpty()
    {
        return (_writePos == _readPos);
    }
}