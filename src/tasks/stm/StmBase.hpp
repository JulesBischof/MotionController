#pragma once

#include "LineFollowerTaskStatusFlags.hpp"

#include "pico/stdlib.h"

namespace MtnCtrl
{
    namespace stm
    {
        template <typename StateType>
        class StmBase
        {
        protected:
            uint32_t *_statusFlags;
            StateType _state;

        public:
            StmBase() : _statusFlags(nullptr), _state() {}
            StmBase(uint32_t *_statusFlags) : _statusFlags(_statusFlags), _state() {}
            virtual ~StmBase() = default;

            virtual void init() = 0;
            virtual bool run() = 0;
            virtual void reset() = 0;
            virtual void update(uint32_t msgData) = 0;
            virtual StateType getState() = 0;
        };
    }
}