#pragma once

#include "pico/stdlib.h"

namespace MotionController
{
    template <typename StateType>
    class StmBase
    {
    protected:
        uint32_t *_statusFlags;
        StateType _state;

    public:
        StmBase(uint32_t *_statusFlags);
        virtual ~StmBase() = default;

        virtual void init() = 0;
        virtual bool run() = 0;
        virtual void reset() = 0;
        virtual void update(uint32_t msgData) = 0;
        virtual StateType getState() = 0;
        
    };

    template <typename StateType>
    inline StmBase<StateType>::StmBase(uint32_t *_statusFlags)
    {
    }
}