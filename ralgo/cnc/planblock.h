#ifndef RALGO_CNC_PLANBLOCK_H
#define RALGO_CNC_PLANBLOCK_H

#include <ralgo/cnc/defs.h>
#include <ralgo/linalg/vector_view.h>
#include <nos/io/ostream.h>
#include <nos/print.h>

#include <stdint.h>
#include <string.h>

#include <assert.h>
#include <math.h>

namespace cnc
{
    class planner_block
    {
    public:
        std::array<double, NMAX_AXES> axdist;
        double nominal_velocity = 0;
        double acceleration = 0;
        double fullpath = 0;
        std::array<double, NMAX_AXES> multipliers;

        // отметки времени хранят инкрементное время до планирования и
        // абсолютное время после активации блока.
        int64_t start_ic = 0;
        int64_t acceleration_before_ic = 0;
        int64_t deceleration_after_ic = 0;

        int64_t block_finish_ic = 0;
        int64_t active_finish_ic = 0;

        int blockno = 0;
        uint8_t exact_stop = 0;

    public:
        void immedeate_smooth_stop(int64_t iteration_counter) 
        {
           int64_t shift = deceleration_after_ic - iteration_counter;
           deceleration_after_ic = deceleration_after_ic - shift;
           block_finish_ic = block_finish_ic - shift;
           active_finish_ic = active_finish_ic - shift;
        }

        planner_block() {}
        planner_block(const planner_block&) = default;
        planner_block& operator=(const planner_block&) = default;

        size_t print_to(nos::ostream& os) const
        {
            PRINTTO(os, axdist);
            PRINTTO(os, nominal_velocity);
            PRINTTO(os, acceleration);
            PRINTTO(os, fullpath);
            PRINTTO(os, multipliers);
            
            PRINTTO(os, start_ic);
            PRINTTO(os, acceleration_before_ic);
            PRINTTO(os, deceleration_after_ic);
            PRINTTO(os, block_finish_ic);
            PRINTTO(os, active_finish_ic);
            PRINTTO(os, blockno);
            PRINTTO(os, exact_stop);
            return 0;
        } 

        bool validation()
        {
            if (fabs(acceleration_before_ic * acceleration - nominal_velocity) >
                1e-5)
                return false;

            if (fabs(nominal_velocity * deceleration_after_ic - fullpath) >
                1e-5)
                return false;

            return true;
        }

        bool is_triangle()
        {
            return acceleration_before_ic == deceleration_after_ic;
        }

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
                accs[i] = acceleration * multipliers[i];
            }
        }

        void append_accelerations(double *accs, int len, int64_t itercounter)
        {
            double acceleration = current_acceleration(itercounter);

            if (acceleration == 0)
                return;

            for (int i = 0; i < len; ++i)
            {
                accs[i] += acceleration * multipliers[i];
            }
        }

        void set_state(const ralgo::vector_view<double>& axdist, int axes, double velocity,
                       double acceleration, const ralgo::vector_view<double>& multipliers)
        {
            for (int i = 0; i < axes; ++i)
            {
                this->multipliers[i] = multipliers[i];
                this->axdist[i] = axdist[i];
            }

            //assert(velocity < 1);
            //assert(acceleration < 1);

            double pathsqr = 0;
            for (int i = 0; i < axes; ++i)
                pathsqr += axdist[i] * axdist[i];
            double path = sqrt(pathsqr); // area
            double time = path / velocity;

            int itime = ceil(time);
            int preftime = ceil(velocity / acceleration);

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

        void set_stop_state(ralgo::vector_view<double> axdist, int axes, double velocity,
           double acceleration, ralgo::vector_view<double> multipliers)
        {
            for (int i = 0; i < axes; ++i)
            {
                this->multipliers[i] = multipliers[i];
                this->axdist[i] = axdist[i];
            }
            double pathsqr = 0;
            for (int i = 0; i < axes; ++i)
                pathsqr += axdist[i] * axdist[i];
            double path = sqrt(pathsqr); // area

            int preftime = ceil(velocity / acceleration);
            this->fullpath = 
            this->acceleration_before_ic = 0;
            this->deceleration_after_ic = 0;
            this->block_finish_ic = preftime;
            this->nominal_velocity = path / preftime * 2;
            this->acceleration = this->nominal_velocity / preftime;
        }
    };
}

#endif
