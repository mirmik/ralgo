#ifndef RALGO_ROBO_QUADGEN_H
#define RALGO_ROBO_QUADGEN_H

#include <igris/util/graycode.h>
#include <ralgo/robo/stepper.h>

#pragma GCC optimize("O3")

namespace robo
{
    class quadgen : public robo::stepper
    {
        uint8_t _curcode = 0;

    public:
        void inc() override
        {
            ++counter;
            switch (_curcode)
            {
            case 0b00:
                _curcode = 0b01;
                break;
            case 0b01:
                _curcode = 0b11;
                break;
            case 0b10:
                _curcode = 0b00;
                break;
            case 0b11:
                _curcode = 0b10;
                break;
            }
            apply_state(_curcode);
        }

        void dec() override
        {
            --counter;
            switch (_curcode)
            {
            case 0b00:
                _curcode = 0b10;
                break;
            case 0b01:
                _curcode = 0b00;
                break;
            case 0b10:
                _curcode = 0b11;
                break;
            case 0b11:
                _curcode = 0b01;
                break;
            }
            apply_state(_curcode);
        }

        void set_state(uint8_t state)
        {
            _curcode = igris::graycode(state);
            apply_state(_curcode);
        }

        uint8_t current_encoder_code()
        {
            return _curcode;
        }

        void no_action() override
        {
            apply_state(_curcode);
        }

    protected:
        void apply_state(uint8_t current_code)
        {
            bool a = (current_code & 0b01);
            bool b = (current_code & 0b10);
            apply_state(a, b);
        }

        virtual void apply_state(bool a, bool b) = 0;
    };
}

#pragma GCC reset_options

#endif
