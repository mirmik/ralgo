#ifndef RALGO_CNC_PLANBLOCK_H
#define RALGO_CNC_PLANBLOCK_H

#include <assert.h>
#include <math.h>
#include <nos/io/ostream.h>
#include <nos/print.h>
#include <ralgo/cnc/defs.h>
#include <ralgo/linalg/vecops.h>
#include <ralgo/linalg/vector_view.h>
#include <stdint.h>
#include <string.h>

/*         B         C
           __________
          /          \
         /            \
        /              \
       A                D
*/

namespace cnc
{
    class planner_block
    {
    public:
        std::array<double, NMAX_AXES> axdist = {};
        double nominal_velocity = 0;
        double start_velocity = 0;
        double final_velocity = 0;
        double acceleration = 0;
        double fullpath = 0;

    private:
        std::array<double, NMAX_AXES> _direction = {};

    public:
        // отметки времени хранят инкрементное время до планирования и
        // абсолютное время после активации блока.
        int64_t start_ic = 0;
        int64_t acceleration_before_ic =
            0; // < момент времени до которого идёт разгон
        int64_t deceleration_after_ic =
            0; // < момент времени до которого идёт плоский учисток
        int64_t block_finish_ic =
            0; // < момент времени, когда блок будет завершён
        int64_t active_finish_ic = 0; // < когда блок перестанет быть активным

        int blockno = 0;
        uint8_t exact_stop = 0;

    public:
        void print_to_stream(nos::ostream &os)
        {
            nos::println_to(os, "blockno: ", blockno);
            nos::println_to(os, "start_ic: ", start_ic);
            nos::println_to(
                os, "acceleration_before_ic: ", acceleration_before_ic);
            nos::println_to(
                os, "deceleration_after_ic: ", deceleration_after_ic);
            nos::println_to(os, "block_finish_ic: ", block_finish_ic);
            nos::println_to(os, "active_finish_ic: ", active_finish_ic);
            nos::println_to(os, "nominal_velocity: ", nominal_velocity);
            nos::println_to(os, "start_velocity: ", start_velocity);
            nos::println_to(os, "final_velocity: ", final_velocity);
            nos::println_to(os, "acceleration: ", acceleration);
            nos::println_to(os, "fullpath: ", fullpath);
            nos::println_to(os, "exact_stop: ", exact_stop);
            nos::println_to(os, "direction: ", _direction);
        }

        const std::array<double, NMAX_AXES> &direction()
        {
            return _direction;
        }

        void set_direction(const std::initializer_list<double> &dir)
        {
            std::copy(dir.begin(), dir.end(), _direction.begin());
        }

        /*void immedeate_smooth_stop(int64_t iteration_counter)
        {
            int64_t shift = deceleration_after_ic - iteration_counter;
            deceleration_after_ic = deceleration_after_ic - shift;
            block_finish_ic = block_finish_ic - shift;
            active_finish_ic = active_finish_ic - shift;
        }*/

        planner_block() {}
        planner_block(const planner_block &) = default;
        planner_block &operator=(const planner_block &) = default;

        int64_t acceleration_time() const
        {
            return acceleration_before_ic;
        }

        int64_t deceleration_time() const
        {
            return block_finish_ic - deceleration_after_ic;
        }

        int64_t flat_time() const
        {
            return deceleration_after_ic - acceleration_before_ic;
        }

        double A_velocity()
        {
            return start_velocity;
        }

        double B_velocity()
        {
            return nominal_velocity;
        }

        double C_velocity()
        {
            return nominal_velocity;
        }

        double D_velocity()
        {
            return nominal_velocity - acceleration * deceleration_time();
        }

        double AB_distance()
        {
            return 0.5 * (start_velocity + nominal_velocity) *
                   acceleration_time();
        }

        double CD_distance()
        {
            return 0.5 * (nominal_velocity + final_velocity) *
                   deceleration_time();
        }

        double BC_distance()
        {
            return B_velocity() * flat_time();
        }

        bool validation()
        {
            if (fabs(start_velocity + acceleration_time() * acceleration -
                     nominal_velocity) > 1e-3)
                return false;

            if (fabs(AB_distance() + BC_distance() + CD_distance() - fullpath) >
                1e-5)
                return false;

            if (ralgo::vecops::norm(_direction) - 1 >= 1e-5)
                return false;

            return true;
        }

        bool is_triangle()
        {
            // Треугольный патерн суть вырожденная трапеция.
            return acceleration_before_ic == deceleration_after_ic;
        }

        // Получает на вход время начала исполнения блока. Сдвигает все
        // временные метки, переводя времена из относительной системы в
        // абсолютную.
        void shift_timestampes(int64_t iteration_counter)
        {
            start_ic += iteration_counter;
            acceleration_before_ic += iteration_counter;
            deceleration_after_ic += iteration_counter;
            block_finish_ic += iteration_counter;
            active_finish_ic += iteration_counter;
        }

        bool is_active(int64_t interrupt_counter)
        {
            if (exact_stop)
                return interrupt_counter < block_finish_ic;
            else
                return interrupt_counter < active_finish_ic;
        }

        bool is_active_or_postactive(int64_t interrupt_counter)
        {
            return interrupt_counter < block_finish_ic;
        }

        bool is_accel(int64_t interrupt_counter)
        {
            return interrupt_counter < acceleration_before_ic;
        }

        double current_acceleration(int64_t interrupt_counter)
        {
            // Вычисление ускорения для трапециидального паттерна.
            if (interrupt_counter < acceleration_before_ic)
                return acceleration;
            if (interrupt_counter < deceleration_after_ic)
                return 0;
            if (interrupt_counter < block_finish_ic)
                return -acceleration;
            return 0;
        }

        void assign_accelerations(double *accs, int len, int64_t itercounter)
        {
            double acceleration = current_acceleration(itercounter);

            if (acceleration == 0)
            {
                for (int i = 0; i < len; ++i)
                    accs[i] = 0;
                return;
            }

            for (int i = 0; i < len; ++i)
            {
                accs[i] = acceleration * _direction[i];
            }
        }

        void append_accelerations(double *accs, int len, int64_t itercounter)
        {
            double acceleration = current_acceleration(itercounter);

            if (acceleration == 0)
                return;

            for (int i = 0; i < len; ++i)
            {
                accs[i] += acceleration * _direction[i];
            }
        }

        void set_state(const ralgo::vector_view<double> &axdist,
                       int axes,
                       double velocity,
                       double acceleration)
        {
            start_velocity = 0;
            final_velocity = 0;

            auto direction =
                ralgo::vecops::normalize<ralgo::vector<double>>(axdist);

            for (int i = 0; i < axes; ++i)
            {
                this->_direction[i] = direction[i];
                this->axdist[i] = axdist[i];
            }

            double pathsqr = 0;
            for (int i = 0; i < axes; ++i)
                pathsqr += axdist[i] * axdist[i];
            double path = sqrt(pathsqr); // area
            double time = path / velocity;

            int itime = ceil(time);
            int preftime = ceil(velocity / acceleration);

            // Для трапециидального паттерна.
            this->active_finish_ic = itime;
            this->fullpath = path;
            this->start_ic = 0;

            if (itime > preftime)
            {
                // trapecidal pattern
                this->acceleration_before_ic = preftime;
                this->deceleration_after_ic = itime;
                this->block_finish_ic = itime + preftime;
                this->nominal_velocity = path / itime;
                this->acceleration = this->nominal_velocity / preftime;
            }

            else
            {
                // triangle pattern
                double maxspeed = sqrt(path * acceleration);
                double halftime = path / maxspeed;
                int itime2 = ceil(halftime);

                this->acceleration_before_ic = itime2;
                this->deceleration_after_ic = itime2;
                this->block_finish_ic = itime2 * 2;
                this->nominal_velocity = path / itime2;
                this->acceleration = this->nominal_velocity / itime2;
            }

            assert(validation());
        }

        bool set_stop_pattern(int axes,
                              double velocity,
                              double acceleration,
                              ralgo::vector_view<double> _direction)
        {
            start_velocity = velocity;
            final_velocity = 0;
            double path = velocity * velocity / (2 * acceleration);

            for (int i = 0; i < axes; ++i)
            {
                this->_direction[i] = _direction[i];
                this->axdist[i] = path * _direction[i];
            }
            int preftime = ceil(velocity / acceleration);
            this->acceleration_before_ic = 0;
            this->deceleration_after_ic = 0;
            this->block_finish_ic = preftime;
            this->nominal_velocity = path / preftime * 2;
            this->acceleration = this->nominal_velocity / preftime;
            this->fullpath = path;
            this->exact_stop = true;

            bool valid = validation();
            return valid;
        }
    };
}

#endif
