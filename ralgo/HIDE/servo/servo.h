#ifndef RALGO_SERVO_SERVO_H
#define RALGO_SERVO_SERVO_H

#include "impulse_writer.h"
#include "position_driver.h"

#include <emergency_stop.h>

namespace ralgo
{
    template <class Lock = igris::syslock, class P = int64_t, class V = float,
              class A = float, class T = time_t>
    class servo // : public position_driver<P,V,A>
    {
    public:
        Lock *lock;

        ralgo::impulse_writer<Lock, P, V> *writer = nullptr;
        ralgo::trajectory<P, V, A, T> *curtraj = nullptr;

        ralgo::accdcc_by_time_trajectory<P, V, A, T> movetraj;
        ralgo::keep_trajectory<P, V, A, T> keeptraj;

        ralgo::phase<P, V, A> phs;

        V move_mode_speed = 0;
        A move_mode_acctime = 0;

        servo() : curtraj(&keeptraj), phs(0, 0, 0) {}

        int relative_move(P dist)
        {
            T lintime = (ralgo::abs(dist) / move_mode_speed);

            if (lintime > 2 * move_mode_acctime)
            {
                lintime -= 2 * move_mode_acctime;
            }
            else
            {
                lintime = 0;
            }

            movetraj.init(phs.d0, phs.d0 + dist, move_mode_acctime, lintime,
                          move_mode_acctime);
            movetraj.set_start_time(millis());

            lock->lock();
            curtraj = &movetraj;
            lock->unlock();
        }

        void serve(T time)
        {
            int sts;

            assert(writer);
            assert(curtraj);

            lock->lock();
            sts = curtraj->inabstime(time, &phs);
            lock->unlock();
            writer->write(phs.d0, phs.d1);
            // dprln(phs.d0, phs.d1);

            if (sts == -1)
            {
                dprln("finish operation");
                // Вышли за текущее время траектории.
                // Операция завершена, устанавливаем траекторию сохранения в
                // качестве активной.
                keeptraj.x = phs.d0;
                curtraj = &keeptraj;

                emergency_stop();
                abort();
            }
        }
    };
} // namespace ralgo

#endif
