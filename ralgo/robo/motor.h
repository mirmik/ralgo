#ifndef RALGO_ROBO_MOTOR_H
#define RALGO_ROBO_MOTOR_H

namespace robo
{
    /** Базовый класс драйвера мощности.
        Управляет двигателями или подобными им устройствами.
    */
    struct motor
    {
        /// Подать мощность в интервале [-1:+1]
        virtual void power(float pwr) = 0;

        /// Отключить подачу мощности.
        virtual void stop() { power(0); }
    };
}

#endif
