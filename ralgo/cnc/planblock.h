#ifndef RALGO_CNC_PLANBLOCK_H
#define RALGO_CNC_PLANBLOCK_H

#include <assert.h>
#include <math.h>
#include <nos/io/ostream.h>
#include <nos/log.h>
#include <nos/print.h>
#include <ralgo/cnc/defs.h>
#include <ralgo/cnc/util.h>
#include <ralgo/linalg/vecops.h>
#include <ralgo/linalg/vector_view.h>
#include <stdint.h>
#include <string.h>

/**
 * Trapezoidal velocity profile for a motion block.
 *
 * Velocity profile shape:
 *         B         C
 *           __________
 *          /          \
 *         /            \
 *        /              \
 *       A                D
 *
 * Where:
 *   A = start_velocity (entry)
 *   B = nominal_velocity (cruise start)
 *   C = nominal_velocity (cruise end)
 *   D = final_velocity (exit)
 *
 * All units are in steps and ticks (timer interrupts).
 * Unit conversion (mm -> steps, sec -> tick) happens in Interpreter.
 */

namespace cnc
{
    class planner_block
    {
    public:
        // === Geometry (in steps) ===
        std::array<cnc_float_type, NMAX_AXES> axdist = {};  ///< Distance per axis [steps]
        cnc_float_type fullpath = 0;  ///< Euclidean path length [steps]

        // === Velocity profile (in steps/tick) ===
        cnc_float_type nominal_velocity = 0;  ///< Cruise velocity [steps/tick]
        cnc_float_type start_velocity = 0;    ///< Entry velocity [steps/tick]
        cnc_float_type final_velocity = 0;    ///< Exit velocity [steps/tick]
        cnc_float_type acceleration = 0;      ///< Acceleration magnitude [steps/tick²]

    private:
        std::array<cnc_float_type, NMAX_AXES> _direction = {};  ///< Normalized direction vector [dimensionless, |d|=1]

    public:
        // === Timing (in ticks) ===
        // Timestamps are relative before activation, absolute after.
        int64_t start_ic = 0;                 ///< Block start time [tick]
        int64_t acceleration_before_ic = 0;   ///< End of acceleration phase [tick]
        int64_t deceleration_after_ic = 0;    ///< Start of deceleration phase [tick]
        int64_t block_finish_ic = 0;          ///< Block end time [tick]

        // === Metadata ===
        int blockno = 0;        ///< Block sequence number
        uint8_t exact_stop = 0; ///< If true, velocity must reach zero at block end

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
            nos::println_to(os, "nominal_velocity: ", nominal_velocity);
            nos::println_to(os, "start_velocity: ", start_velocity);
            nos::println_to(os, "final_velocity: ", final_velocity);
            nos::println_to(os, "acceleration: ", acceleration);
            nos::println_to(os, "fullpath: ", fullpath);
            nos::println_to(os, "exact_stop: ", exact_stop);
            nos::println_to(os, "direction: ", _direction);
        }

        const std::array<cnc_float_type, NMAX_AXES> &direction() const
        {
            return _direction;
        }

        void set_direction(const std::initializer_list<cnc_float_type> &dir)
        {
            std::copy(dir.begin(), dir.end(), _direction.begin());
        }

        /// Рассчитать максимальную скорость на стыке двух блоков.
        /// Использует алгоритм Junction Deviation (как в Marlin/Grbl).
        ///
        /// @param dir1 - направление первого блока (нормализованное)
        /// @param dir2 - направление второго блока (нормализованное)
        /// @param nominal_velocity - номинальная скорость (steps/tick)
        /// @param acceleration - ускорение (steps/tick²)
        /// @param junction_deviation - допустимое отклонение (steps)
        /// @param axes - количество осей
        /// @return максимальная скорость на стыке (steps/tick)
        static cnc_float_type calculate_junction_velocity(
            const std::array<cnc_float_type, NMAX_AXES> &dir1,
            const std::array<cnc_float_type, NMAX_AXES> &dir2,
            cnc_float_type nominal_velocity,
            cnc_float_type acceleration,
            cnc_float_type junction_deviation,
            int axes)
        {
            // Вычисляем косинус угла между направлениями
            cnc_float_type cos_theta = 0;
            for (int i = 0; i < axes; ++i)
            {
                cos_theta += dir1[i] * dir2[i];
            }

            // Ограничиваем cos_theta в [-1, 1] для защиты от численных ошибок
            if (cos_theta > 1.0)
                cos_theta = 1.0;
            if (cos_theta < -1.0)
                cos_theta = -1.0;

            // Если направления почти совпадают - возвращаем номинальную скорость
            if (cos_theta > 0.9999)
            {
                return nominal_velocity;
            }

            // Если разворот на 180° - возвращаем 0
            if (cos_theta < -0.9999)
            {
                return 0;
            }

            // sin(theta/2) = sqrt((1 - cos_theta) / 2)
            cnc_float_type sin_half_theta = sqrt(0.5 * (1.0 - cos_theta));

            // Junction velocity из условия ограничения центростремительного ускорения:
            // v² / R = a, где R = junction_deviation / sin(theta/2)
            // => v = sqrt(a * R) = sqrt(a * junction_deviation / sin(theta/2))
            cnc_float_type v_junction =
                sqrt(acceleration * junction_deviation / sin_half_theta);

            // Ограничиваем номинальной скоростью
            if (v_junction > nominal_velocity)
                v_junction = nominal_velocity;

            return v_junction;
        }

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

        cnc_float_type A_velocity()
        {
            return start_velocity;
        }

        cnc_float_type B_velocity()
        {
            return nominal_velocity;
        }

        cnc_float_type C_velocity()
        {
            return nominal_velocity;
        }

        cnc_float_type D_velocity()
        {
            return nominal_velocity - acceleration * deceleration_time();
        }

        cnc_float_type AB_distance()
        {
            return 0.5 * (start_velocity + nominal_velocity) *
                   acceleration_time();
        }

        cnc_float_type CD_distance()
        {
            return 0.5 * (nominal_velocity + final_velocity) *
                   deceleration_time();
        }

        cnc_float_type BC_distance()
        {
            return B_velocity() * flat_time();
        }

        bool validation()
        {
            if (fabs(AB_distance() + BC_distance() + CD_distance() - fullpath) >
                1e-5)
            {
                nos::log::error("Block is not valid by reason N2");
                return false;
            }

            if (ralgo::vecops::norm(_direction) - 1 >= 1e-5)
            {
                nos::log::error("Block is not valid by reason N3");
                return false;
            }

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
        }

        bool is_active(int64_t interrupt_counter)
        {
            if (exact_stop)
                return interrupt_counter < block_finish_ic &&
                       interrupt_counter >= acceleration_before_ic;
            else
                return interrupt_counter < deceleration_after_ic &&
                       interrupt_counter >= acceleration_before_ic;
        }

        bool is_active_or_postactive(int64_t interrupt_counter)
        {
            return interrupt_counter < block_finish_ic &&
                   interrupt_counter >= acceleration_before_ic;
        }

        bool is_accel(int64_t interrupt_counter)
        {
            return interrupt_counter < acceleration_before_ic &&
                   interrupt_counter >= start_ic;
        }

        bool is_flat(int64_t interrupt_counter)
        {
            return interrupt_counter <= deceleration_after_ic &&
                   interrupt_counter >= acceleration_before_ic;
        }

        bool is_decel(int64_t interrupt_counter)
        {
            return interrupt_counter <= block_finish_ic &&
                   interrupt_counter > deceleration_after_ic;
        }

        cnc_float_type current_acceleration(int64_t interrupt_counter)
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

        void
        assign_accelerations(cnc_float_type *accs, int len, int64_t itercounter)
        {
            cnc_float_type acceleration = current_acceleration(itercounter);

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

        void
        append_accelerations(cnc_float_type *accs, int len, int64_t itercounter)
        {
            cnc_float_type acceleration = current_acceleration(itercounter);

            if (acceleration == 0)
                return;

            for (int i = 0; i < len; ++i)
            {
                accs[i] += acceleration * _direction[i];
            }
        }

        /// Установить состояние блока с заданными граничными скоростями.
        ///
        /// Трапецеидальный профиль скорости:
        ///
        ///        B______________C
        ///       /                \
        ///      /                  \
        ///     /                    \
        ///    A                      D
        ///    ^                      ^
        /// start_vel              final_vel
        ///
        /// @param axdist - расстояние по каждой оси [steps]
        /// @param axes - количество осей
        /// @param velocity - номинальная (крейсерская) скорость [steps/tick]
        /// @param acceleration - ускорение [steps/tick²]
        /// @param start_vel - начальная скорость [steps/tick], по умолчанию 0
        /// @param final_vel - конечная скорость [steps/tick], по умолчанию 0
        void set_state(const ralgo::vector_view<cnc_float_type> &axdist,
                       int axes,
                       cnc_float_type velocity,
                       cnc_float_type acceleration,
                       cnc_float_type start_vel = 0,
                       cnc_float_type final_vel = 0)
        {
            auto direction =
                ralgo::vecops::normalize<ralgo::vector<cnc_float_type>>(axdist);

            for (int i = 0; i < axes; ++i)
            {
                this->_direction[i] = direction[i];
                this->axdist[i] = axdist[i];
            }

            cnc_float_type pathsqr = 0;
            for (int i = 0; i < axes; ++i)
                pathsqr += axdist[i] * axdist[i];
            cnc_float_type path = sqrt(pathsqr);

            this->fullpath = path;
            this->acceleration = acceleration;
            this->start_ic = 0;

            // Ограничиваем граничные скорости номинальной
            if (start_vel > velocity)
                start_vel = velocity;
            if (final_vel > velocity)
                final_vel = velocity;

            this->start_velocity = start_vel;
            this->final_velocity = final_vel;
            this->nominal_velocity = velocity;

            // Вычисляем тайминги
            recalculate_timing();
        }

        /// Пересчитать тайминги блока с текущими start_velocity и final_velocity.
        /// Вызывается после изменения граничных скоростей (например, после
        /// backward/forward pass в look-ahead алгоритме).
        ///
        /// Формулы для трапецеидального профиля:
        /// - Расстояние разгона: d_acc = (V² - Vs²) / (2*a)
        /// - Расстояние торможения: d_dec = (V² - Vf²) / (2*a)
        /// - Расстояние круиза: d_cruise = fullpath - d_acc - d_dec
        ///
        /// Если d_cruise < 0, это треугольный профиль - nominal_velocity снижается.
        void recalculate_timing()
        {
            cnc_float_type path = this->fullpath;
            cnc_float_type acc = this->acceleration;
            cnc_float_type Vs = this->start_velocity;
            cnc_float_type Vf = this->final_velocity;
            cnc_float_type Vn = this->nominal_velocity;

            if (acc <= 0 || path <= 0)
            {
                // Некорректные параметры - нулевой блок
                this->acceleration_before_ic = 0;
                this->deceleration_after_ic = 0;
                this->block_finish_ic = 0;
                return;
            }

            // Расстояние для разгона от Vs до Vn
            cnc_float_type d_acc = (Vn * Vn - Vs * Vs) / (2 * acc);
            // Расстояние для торможения от Vn до Vf
            cnc_float_type d_dec = (Vn * Vn - Vf * Vf) / (2 * acc);
            // Расстояние круиза
            cnc_float_type d_cruise = path - d_acc - d_dec;

            if (d_cruise >= 0)
            {
                // Трапецеидальный профиль
                // Время разгона: t_acc = (Vn - Vs) / acc
                cnc_float_type t_acc = (Vn - Vs) / acc;
                // Время торможения: t_dec = (Vn - Vf) / acc
                cnc_float_type t_dec = (Vn - Vf) / acc;
                // Время круиза: t_cruise = d_cruise / Vn
                cnc_float_type t_cruise = d_cruise / Vn;

                int64_t i_acc = static_cast<int64_t>(ceil(t_acc));
                int64_t i_cruise = static_cast<int64_t>(ceil(t_cruise));
                int64_t i_dec = static_cast<int64_t>(ceil(t_dec));

                this->acceleration_before_ic = i_acc;
                this->deceleration_after_ic = i_acc + i_cruise;
                this->block_finish_ic = i_acc + i_cruise + i_dec;
            }
            else
            {
                // Треугольный профиль - не достигаем nominal_velocity
                // Находим пиковую скорость Vp: d_acc + d_dec = path
                // (Vp² - Vs²)/(2a) + (Vp² - Vf²)/(2a) = path
                // 2*Vp² - Vs² - Vf² = 2*a*path
                // Vp = sqrt((2*a*path + Vs² + Vf²) / 2)
                cnc_float_type Vp_sq = (2 * acc * path + Vs * Vs + Vf * Vf) / 2;

                if (Vp_sq < Vs * Vs || Vp_sq < Vf * Vf)
                {
                    // Невозможно выполнить движение с заданными параметрами
                    // (слишком короткий путь для торможения)
                    // Используем минимальный профиль
                    Vp_sq = fmax(Vs * Vs, Vf * Vf);
                }

                cnc_float_type Vp = sqrt(Vp_sq);

                // Обновляем nominal_velocity на реально достижимую
                this->nominal_velocity = Vp;

                // Время разгона и торможения
                cnc_float_type t_acc = (Vp - Vs) / acc;
                cnc_float_type t_dec = (Vp - Vf) / acc;

                int64_t i_acc = static_cast<int64_t>(ceil(t_acc));
                int64_t i_dec = static_cast<int64_t>(ceil(t_dec));

                this->acceleration_before_ic = i_acc;
                this->deceleration_after_ic = i_acc;  // Без круиза
                this->block_finish_ic = i_acc + i_dec;
            }

            // Минимальная длительность блока - 1 тик
            if (this->block_finish_ic < 1)
                this->block_finish_ic = 1;
        }

        /// Установить новые граничные скорости и пересчитать тайминги.
        /// Используется в backward/forward pass look-ahead алгоритма.
        void set_junction_velocities(cnc_float_type start_vel,
                                     cnc_float_type final_vel)
        {
            this->start_velocity = start_vel;
            this->final_velocity = final_vel;
            recalculate_timing();
        }

        bool
        set_stop_pattern(int axes,
                         cnc_float_type velocity,
                         cnc_float_type acceleration,
                         const ralgo::vector_view<cnc_float_type> &_direction)
        {
            start_velocity = velocity;
            final_velocity = 0;
            cnc_float_type path = velocity * velocity / (2 * acceleration);

            for (int i = 0; i < axes; ++i)
            {
                this->_direction[i] = _direction[i];
                this->axdist[i] = path * _direction[i];
            }

            // Поскольку время дискретно, движение должно быть завершено
            // в момент времени, соответствующий целому числу.
            // Для этого выполняется округление расчётного времени
            // и небольшая модификация скорости и ускорения.
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
