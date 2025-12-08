#ifndef RALGO_CNC_INTERPRETER_H
#define RALGO_CNC_INTERPRETER_H

#include <cstdlib>
#include <igris/container/array_view.h>
#include <igris/container/flat_map.h>
#include <igris/container/ring.h>
#include <igris/container/static_vector.h>
#include <igris/datastruct/argvc.h>
#include <igris/shell/callable_collection.h>
#include <igris/sync/syslock.h>
#include <igris/util/numconvert.h>
#include <nos/ilist.h>
#include <nos/io/string_writer.h>
#include <nos/log/logger.h>
#include <nos/print/stdtype.h>
#include <nos/shell/executor.h>
#include <nos/util/string.h>
#include <ralgo/cnc/control_task.h>
#include <ralgo/cnc/error_handler.h>
#include <ralgo/cnc/feedback_guard.h>
#include <ralgo/cnc/planblock.h>
#include <ralgo/cnc/planner.h>
#include <ralgo/cnc/revolver.h>
#include <ralgo/cnc/util.h>
#include <ralgo/global_protection.h>
#include <ralgo/linalg/vecops.h>
#include <ralgo/log.h>
#include <stdlib.h>
#include <string.h>
#include <string>

namespace cnc
{
    class interpreter
    {
    public:
        cnc::planner *planner = nullptr;
        cnc::revolver *revolver = nullptr;
        cnc::feedback_guard *feedback_guard = nullptr;
        cnc::error_handler *errors = nullptr;

    private:
        igris::ring<planner_block> *blocks = nullptr;
        int total_axes = 0;
        cnc_float_type revolver_frequency = 0;

        // Steps per unit for each axis (conversion factor units -> steps)
        // For linear axes: steps/mm, for rotary axes: steps/rad
        std::array<cnc_float_type, NMAX_AXES> _steps_per_unit = {};

        // Мультипликатор на входе интерпретатора.
        // Применяется к позициям, скоростям и ускорениям во входном потоке
        // комманд. Есть подозрение, что механика gains переусложнена и его надо
        // заменить на единый множитель для всех осей.
        igris::static_vector<cnc_float_type, NMAX_AXES> gains = {};

        igris::static_vector<cnc_float_type, NMAX_AXES> _final_position = {};
        igris::static_vector<cnc_float_type, NMAX_AXES> max_axes_velocities =
            {};
        igris::static_vector<cnc_float_type, NMAX_AXES> max_axes_accelerations =
            {};
        int blockno = 0;
        cnc_float_type saved_acc = 0;
        cnc_float_type saved_feed = 0;
        cnc_float_type _smooth_stop_acceleration = 0;
        igris::delegate<void> _external_final_shift_handle = {};
        planner_block lastblock = {};
        bool with_cleanup = true;
        volatile bool stop_procedure_started = false;

    public:
        interpreter(igris::ring<planner_block> *blocks,
                    cnc::planner *planner,
                    cnc::revolver *revolver,
                    cnc::feedback_guard *feedback_guard,
                    cnc::error_handler *errors = nullptr)
            : planner(planner), revolver(revolver),
              feedback_guard(feedback_guard), errors(errors), blocks(blocks)
        {
        }

        interpreter(const interpreter &) = delete;
        interpreter(interpreter &&) = delete;
        interpreter &operator=(const interpreter &) = delete;
        interpreter &operator=(interpreter &&) = delete;

    private:
        /// Report error if error_handler is configured
        void report_error(error_code code, int8_t axis = -1, int32_t context = 0)
        {
            if (errors)
                errors->report(code, axis, context);
        }

    public:
        void cleanup()
        {
            blockno = 0;
            lastblock = {};
        }

        cnc_float_type smooth_stop_acceleration(
            const ralgo::vector<cnc_float_type> &direction) const
        {
            cnc_float_type maximum_acceleration =
                _smooth_stop_acceleration != 0 ? _smooth_stop_acceleration
                                               : saved_acc;
            return evaluate_external_accfeed_2(
                direction, maximum_acceleration, max_axes_accelerations);
        }

        const igris::static_vector<cnc_float_type, NMAX_AXES> &final_position()
        {
            return _final_position;
        }

        planner_block last_block()
        {
            return lastblock;
        }

        void set_final_shift_handler(const igris::delegate<void> &dlg)
        {
            _external_final_shift_handle = dlg;
        }

        void set_start_shift_handler(const igris::delegate<void> &dlg)
        {
            planner->set_start_operation_handle(dlg);
        }

        void init_axes(int total_axes)
        {
            this->total_axes = total_axes;
            gains.resize(total_axes);
            _final_position.resize(total_axes);
            max_axes_velocities.resize(total_axes);
            max_axes_accelerations.resize(total_axes);
            planner->set_axes_count(total_axes);
            ralgo::vecops::fill(gains, 1);
            ralgo::vecops::fill(_final_position, 0);
            ralgo::vecops::fill(max_axes_accelerations, 0);
            ralgo::vecops::fill(max_axes_velocities, 0);
            // Default steps_per_unit = 1 (1:1 mapping)
            for (int i = 0; i < total_axes; ++i)
                _steps_per_unit[i] = 1.0;

            planner->final_shift_pushed =
                igris::make_delegate(&interpreter::final_shift_handle, this);
        }

        void restore_finishes()
        {
            auto steps = revolver->current_steps();
            for (int i = 0; i < total_axes; ++i)
            {
                // Convert steps to units: units = steps / steps_per_unit
                if (_steps_per_unit[i] != 0)
                    _final_position[i] =
                        static_cast<cnc_float_type>(steps[i]) / _steps_per_unit[i];
            }
        }

        void final_shift_handle()
        {
            if (blocks->avail() == 0)
            {
                restore_finishes();
                stop_procedure_started = false;

                if (with_cleanup)
                {
                    planner->cleanup();
                    revolver->cleanup();
                    cleanup();
                }

                if (_external_final_shift_handle)
                    _external_final_shift_handle();
            }
            if (feedback_guard)
                feedback_guard->finish_operations();
        }

        int check_correctness(nos::ostream &os)
        {
            if (total_axes != (int)planner->get_total_axes())
            {
                nos::println_to(os, "total_axes mismatch with planner");
                return 1;
            }
            if (total_axes != revolver->steppers_total)
            {
                nos::println_to(os, "total_axes mismatch with revolver");
                return 1;
            }

            if (ralgo::global_protection)
            {
                nos::println_to(os, "need_to_disable_global_protection");
                return 1;
            }
            if (revolver_frequency == 0)
            {
                nos::println_to(os, "revolver_frequency is null");
                return 1;
            }
            for (auto &i : gains)
            {
                if (i != 1)
                {
                    nos::println_to(os,
                                    "gains must be 1, becourse another variant "
                                    "is not tested");
                    return 1;
                }
            }
            return 0;
        }

        control_task
        create_task_for_relative_move(const std::vector<idxpos> poses,
                                      cnc_float_type feed,
                                      cnc_float_type acc)
        {
            control_task task(total_axes);
            task.feed = feed == 0 ? saved_feed : feed;
            task.acc = acc == 0 ? saved_acc : acc;
            for (auto &i : poses)
            {
                task.poses()[i.idx] = i.pos;
            }
            task.isok = true;
            task.set_active_axes(poses);
            return task;
        }

        control_task
        create_task_for_absolute_move(const std::vector<idxpos> poses,
                                      cnc_float_type feed,
                                      cnc_float_type acc)
        {
            control_task task(total_axes);
            task.feed = feed == 0 ? saved_feed : feed;
            task.acc = acc == 0 ? saved_acc : acc;
            auto finalpos = _final_position;
            task.set_poses({finalpos.data(), finalpos.size()});
            for (auto &i : poses)
            {
                task.poses()[i.idx] = i.pos;
            }
            for (int i = 0; i < total_axes; ++i)
            {
                task.poses()[i] -= finalpos[i];
            }
            task.isok = true;
            task.set_active_axes(poses);
            return task;
        }

        /// Расщитывает ускорение или скорость для блока на основании
        /// запрошенных скоростей или ускорений для отдельных осей и
        /// скоростей или ускорений для точки в евклидовом пространстве.
        static cnc_float_type evaluate_external_accfeed(
            const ralgo::vector<cnc_float_type> &direction,
            cnc_float_type absolute_maximum,
            const igris::static_vector<cnc_float_type, NMAX_AXES>
                &element_maximums)
        {
            int total_axes = (int)direction.size();
            cnc_float_type minmul = std::numeric_limits<cnc_float_type>::max();

            if (absolute_maximum == 0 &&
                ralgo::vecops::norm(element_maximums) == 0)
                return 0;

            if (absolute_maximum != 0)
                minmul = absolute_maximum;

            for (int i = 0; i < total_axes; i++)
                if (element_maximums[i] != 0)
                {
                    cnc_float_type lmul =
                        element_maximums[i] / fabs(direction[i]);
                    if (minmul > lmul)
                        minmul = lmul;
                }

            if (minmul == std::numeric_limits<cnc_float_type>::max())
                return 1000;
            return minmul;
        }

        static cnc_float_type evaluate_external_accfeed_2(
            const ralgo::vector<cnc_float_type> &direction,
            cnc_float_type absolute_maximum,
            const igris::static_vector<cnc_float_type, NMAX_AXES>
                &element_maximums)
        {
            if (ralgo::vecops::norm(element_maximums) == 0)
            {
                return absolute_maximum;
            }

            if (absolute_maximum == 0)
            {
                auto bounded =
                    ralgo::vecops::ray_to_box<ralgo::vector<cnc_float_type>>(
                        direction, element_maximums);
                return ralgo::vecops::norm(bounded);
            }
            else
            {
                auto vec = ralgo::vecops::mul_vs(direction, absolute_maximum);
                auto bounded =
                    ralgo::vecops::bound_to_box<ralgo::vector<cnc_float_type>>(
                        vec, element_maximums);
                return ralgo::vecops::norm(bounded);
            }
        }

        bool evaluate_interpreter_task(const control_task &task,
                                       planner_block &block,
                                       nos::ostream &)
        {
            saved_acc = task.acc;
            saved_feed = task.feed;

            // === Input validation ===

            // Check frequency is configured
            if (revolver_frequency <= 0)
            {
                report_error(error_code::invalid_frequency, -1,
                             static_cast<int32_t>(revolver_frequency));
                return true;
            }

            // Check task has non-zero movement
            cnc_float_type tasknorm = ralgo::vecops::norm(task.poses());
            if (tasknorm < 1e-10)
            {
                // Empty move - not an error, just skip
                return true;
            }

            // Check steps_per_unit for active axes
            for (auto idx : task.active_axes)
            {
                if (idx >= 0 && idx < total_axes && _steps_per_unit[idx] <= 0)
                {
                    report_error(error_code::invalid_steps_per_unit, idx);
                    return true;
                }
            }

            // Get steps_per_unit for unit conversion (from interpreter's own storage)
            ralgo::vector_view<cnc_float_type> gears(_steps_per_unit.data(),
                                                     total_axes);

            // Apply gains (user-space scaling)
            auto intdists_mm =
                ralgo::vecops::mul_vv<ralgo::vector<cnc_float_type>>(
                    task.poses(), gains);

            // Convert mm to steps
            auto intdists_steps =
                ralgo::vecops::mul_vv<ralgo::vector<cnc_float_type>>(
                    intdists_mm, gears);

            auto direction =
                ralgo::vecops::normalize<ralgo::vector<cnc_float_type>>(
                    task.poses());

            // Check direction is valid (not zero after normalization)
            cnc_float_type dirnorm = ralgo::vecops::norm(direction);
            if (dirnorm < 0.99 || dirnorm > 1.01)
            {
                report_error(error_code::invalid_direction);
                return true;
            }

            auto dirgain =
                ralgo::vecops::norm(ralgo::vecops::mul_vv(direction, gains));

            // Calculate steps_per_unit scaling factor for velocity/acceleration
            // Uses direction-weighted average for multi-axis moves
            auto dirsteps =
                ralgo::vecops::norm(ralgo::vecops::mul_vv(direction, gears));

            auto evalfeed = evaluate_external_accfeed_2(
                direction, task.feed, max_axes_velocities);
            auto evalacc = evaluate_external_accfeed_2(
                direction, task.acc, max_axes_accelerations);

            if (evalacc <= 0)
            {
                report_error(error_code::invalid_acceleration, -1,
                             static_cast<int32_t>(evalacc * 1000));
                return true;
            }

            if (evalfeed <= 0)
            {
                report_error(error_code::invalid_velocity, -1,
                             static_cast<int32_t>(evalfeed * 1000));
                return true;
            }

            // Convert mm/sec to steps/sec
            cnc_float_type feed_mm = evalfeed * dirgain;
            cnc_float_type acc_mm = evalacc * dirgain;
            cnc_float_type feed_steps = feed_mm * dirsteps;
            cnc_float_type acc_steps = acc_mm * dirsteps;

            // Convert steps/sec to steps/tick (division is safe - frequency checked above)
            cnc_float_type reduced_feed = feed_steps / revolver_frequency;
            cnc_float_type reduced_acc =
                acc_steps / (revolver_frequency * revolver_frequency);

            if (feedback_guard)
                for (auto idx : task.active_axes)
                {
                    feedback_guard->enable_tandem_protection_for_axis(idx);
                }

            // Output: distances in steps, velocity/acc in steps/tick
            block.set_state(
                intdists_steps, total_axes, reduced_feed, reduced_acc);
            return false;
        }

        std::vector<cnc_float_type> final_gained_position()
        {
            std::vector<cnc_float_type> ret(total_axes);
            for (int i = 0; i < total_axes; ++i)
                ret[i] = _final_position[i] / gains[i];
            return ret;
        }

        void set_scale(const ralgo::vector_view<cnc_float_type> &vec)
        {
            if (vec.size() != static_cast<size_t>(total_axes))
            {
                ralgo::warn("set_scale fail");
            }
            std::copy(vec.begin(), vec.end(), gains.begin());
        }

        void set_saved_acc(cnc_float_type acc)
        {
            saved_acc = acc;
        }

        void set_saved_feed(cnc_float_type feed)
        {
            saved_feed = feed;
        }

        void evaluate_task(const control_task &task, nos::ostream &os)
        {
            bool fastfinish = evaluate_interpreter_task(task, lastblock, os);
            if (fastfinish)
            {
                _external_final_shift_handle();
                ralgo::info("fastfinish");
                //    print_interpreter_state(os);
                return;
            }

            // axdist is in steps, convert back to mm for _final_position
            for (int i = 0; i < total_axes; ++i)
            {
                if (_steps_per_unit[i] != 0)
                    _final_position[i] += lastblock.axdist[i] / _steps_per_unit[i];
            }

            auto &placeblock = blocks->head_place();
            lastblock.blockno = blockno++;
            placeblock = lastblock;

            system_lock();
            blocks->move_head_one();
            system_unlock();
        }

        void command_incremental_move(const nos::argv &argv, nos::ostream &os)
        {
            if (stop_procedure_started)
                return;

            ralgo::info("command_incremental_move");

            if (check_correctness(os))
                return;
            auto poses = get_task_poses_from_argv(argv);
            cnc_float_type feed = get_task_feed_from_argv(argv);
            cnc_float_type acc = get_task_acc_from_argv(argv);
            auto task = create_task_for_relative_move(poses, feed, acc);
            if (!task.isok)
                return;
            evaluate_task(task, os);
        }

        void command_absolute_move(const nos::argv &argv, nos::ostream &os)
        {
            if (stop_procedure_started)
                return;

            ralgo::info("command_absolute_move");

            if (check_correctness(os))
                return;
            auto poses = get_task_poses_from_argv(argv);
            cnc_float_type feed = get_task_feed_from_argv(argv);
            cnc_float_type acc = get_task_acc_from_argv(argv);
            auto task = create_task_for_absolute_move(poses, feed, acc);

            if (!task.isok)
            {
                ralgo::info("command_absolute_move: task is not ok");
                return;
            }
            evaluate_task(task, os);
        }

        void command_absolute_pulses(const nos::argv &argv, nos::ostream &os)
        {
            if (stop_procedure_started)
                return;

            ralgo::info("command_absolute_move");

            if (check_correctness(os))
                return;

            auto poses = get_task_poses_from_argv(argv);
            cnc_float_type feed = get_task_feed_from_argv(argv);
            cnc_float_type acc = get_task_acc_from_argv(argv);
            (void)poses;
            (void)feed;
            (void)acc;

            // TODO: implement

            return;
        }

        void inspect_args(const nos::argv &argv, nos::ostream &os)
        {
            PRINTTO(os, argv.size());
            for (auto &arg : argv)
            {
                PRINTTO(os, arg);
            }
        }

        void command_M204(const nos::argv &argv, nos::ostream &os)
        {
            if (argv.size() == 0)
            {
                nos::print_to(os, "%f", saved_acc);
                return;
            }

            inspect_args(argv, os);

            saved_acc = igris_atof64(argv[0].data(), nullptr);
            PRINTTO(os, saved_acc);
        }

        // Включить режим остановки.
        void command_M112(const nos::argv &, nos::ostream &)
        {
            system_lock();
            system_unlock();
        }

        void smooth_stop()
        {
            ralgo::info("SMOOTH_STOP");

            system_lock();
            if (stop_procedure_started)
            {
                ralgo::info("prevented because stop procedure is started");
                system_unlock();
                return;
            }

            auto curvels = revolver->current_velocities();
            auto cursteps = revolver->current_steps_no_lock();

            if (ralgo::vecops::norm(curvels) < 1e-5)
            {
                ralgo::info("prevented because not moving");
                system_unlock();
                return;
            }

            // Create direction vector with only active axes (total_axes)
            ralgo::vector<cnc_float_type> direction_vec(total_axes);
            for (int i = 0; i < total_axes; ++i)
                direction_vec[i] = curvels[i];
            auto direction =
                ralgo::vecops::normalize<ralgo::vector<cnc_float_type>>(
                    direction_vec);

            // Convert acceleration from units/s² to steps/tick²
            // smooth_stop_acceleration() returns units/s² (same as saved_acc)
            // We need to multiply by steps_per_unit factor to get steps/s²,
            // then divide by freq² to get steps/tick²
            ralgo::vector_view<cnc_float_type> gears(_steps_per_unit.data(),
                                                      total_axes);
            auto dirsteps =
                ralgo::vecops::norm(ralgo::vecops::mul_vv(direction, gears));

            auto external_acceleration = smooth_stop_acceleration(direction) *
                                         dirsteps / revolver_frequency /
                                         revolver_frequency;

            auto velocity = ralgo::vecops::norm(direction_vec);
            if (velocity == 0)
            {
                ralgo::info("prevented because velocity is not valid");
                system_unlock();
                return;
            }

            bool is_valid = lastblock.set_stop_pattern(
                total_axes, velocity, external_acceleration, direction);

            if (!is_valid)
            {
                nos::log::info("prevented because stop pattern is not valid");
                system_unlock();
                return;
            }

            restore_finishes();
            // axdist is in steps, convert back to mm for _final_position
            for (int i = 0; i < total_axes; ++i)
            {
                if (_steps_per_unit[i] != 0)
                    _final_position[i] += lastblock.axdist[i] / _steps_per_unit[i];
            }

            planner->clear_for_stop();
            planner->force_skip_all_blocks();
            auto &placeblock = blocks->head_place();
            lastblock.blockno = 0;
            placeblock = lastblock;
            blocks->move_head_one();
            stop_procedure_started = true;
            system_unlock();
        }

        void g_command(const nos::argv &argv, nos::ostream &os)
        {
            int cmd = atoi(&argv[0].data()[1]);
            switch (cmd)
            {
            case 1:
                command_incremental_move(argv.without(1), os);
                break;
            default:
                nos::println_to(os, "Unresolved G command");
            }
        }

        void m_command(const nos::argv &argv, nos::ostream &os)
        {
            int cmd = atoi(&argv[0].data()[1]);
            switch (cmd)
            {
            case 112:
                command_M204(argv.without(1), os);
                break;
            case 204:
                command_M204(argv.without(1), os);
                break;
            default:
                nos::println_to(os, "Unresolved M command");
            }
        }

        int gcode(const nos::argv &argv, nos::ostream &os)
        {
            if (argv.size() == 0)
                return 0;

            char cmdsymb = argv[0][0];
            switch (cmdsymb)
            {
            case 'G':
            {
                g_command(argv, os);
                break;
            }

            case 'M':
            {
                m_command(argv, os);
                break;
            }

            default:
                nos::fprintln_to(os, "Unresolved command: {}", argv[0]);
            }

            return 0;
        }

        std::vector<size_t> args_to_index_vector(const nos::argv &args)
        {
            std::vector<size_t> ret;
            for (unsigned int i = 0; i < args.size(); ++i)
            {
                auto index = std::stoi(args[i]);
                ret.push_back(index);
            }
            return ret;
        }

        std::vector<size_t> args_symbols_to_index_vector(const nos::argv &args)
        {
            std::vector<size_t> ret;
            for (unsigned int i = 0; i < args.size(); ++i)
            {
                auto index = symbol_to_index(args[i][0]);
                ret.push_back(index);
            }
            return ret;
        }

        int command(const nos::argv &argv, nos::ostream &os)
        {
            auto *fptr = clicommands.find(argv[0].data());
            if (fptr)
            {
                return (*fptr)(argv, os);
            }

            nos::fprintln_to(
                os, "Unresolved command: {}", std::string(argv[0]));
            return 0;
        }

        int command_help(nos::ostream &os)
        {
            for (auto &rec : clicommands)
            {
                nos::fprintln_to(os, "{} - {}", rec.key, rec.help);
            }
            return 0;
        }

        int gcode_help(nos::ostream &os)
        {
            nos::println_to(os, "G1 <axis><pos>... F<feed> M<accel> - linear move (relative)");
            nos::println_to(os, "  Example: G1 X10 Y-5 F100 M500");
            nos::println_to(os, "M112 - emergency stop");
            nos::println_to(os, "M204 [<accel>] - get/set default acceleration");
            return 0;
        }

        igris::static_callable_collection<int(const nos::argv &,
                                              nos::ostream &),
                                          50>
            clicommands{
                {"setprotect",
                 "Disable global protection (no args)",
                 [](const nos::argv &, nos::ostream &) {
                     ralgo::global_protection = false;
                     ralgo::info("Protection disabled");
                     return 0;
                 }},

                {"stop",
                 "Smooth stop all axes with deceleration (no args)",
                 [this](const nos::argv &, nos::ostream &) {
                     smooth_stop();
                     return 0;
                 }},

                {"lastblock",
                 "Print info about last/current motion block (no args)",
                 [this](const nos::argv &, nos::ostream &os) {
                     last_block().print_to_stream(os);
                     return 0;
                 }},

                {"relmove",
                 "Relative move. Args: <axis><dist>... F<feed> M<accel>. "
                 "Example: relmove X10 Y-5 F100 M500",
                 [this](const nos::argv &argv, nos::ostream &os) {
                     if (argv.size() < 2)
                     {
                         nos::println_to(os, "Usage: relmove <axis><dist>... F<feed_mm/s> M<accel_mm/s2>");
                         nos::println_to(os, "  Axes: X,Y,Z,A,B,C,I,J,K (0-8)");
                         nos::println_to(os, "  Example: relmove X10 Y-5.5 Z2 F100 M500");
                         return 0;
                     }
                     command_incremental_move(argv.without(1), os);
                     return 0;
                 }},

                {"absmove",
                 "Absolute move. Args: <axis><pos>... F<feed> M<accel>. "
                 "Example: absmove X100 Y50 F100 M500",
                 [this](const nos::argv &argv, nos::ostream &os) {
                     if (argv.size() < 2)
                     {
                         nos::println_to(os, "Usage: absmove <axis><pos>... F<feed_mm/s> M<accel_mm/s2>");
                         nos::println_to(os, "  Axes: X,Y,Z,A,B,C,I,J,K (0-8)");
                         nos::println_to(os, "  Example: absmove X100 Y50 Z0 F100 M500");
                         return 0;
                     }
                     command_absolute_move(argv.without(1), os);
                     return 0;
                 }},

                {"abspulses",
                 "Absolute move in pulses (steps). Args: <axis><steps>... F<feed> M<accel>",
                 [this](const nos::argv &argv, nos::ostream &os) {
                     if (argv.size() < 2)
                     {
                         nos::println_to(os, "Usage: abspulses <axis><steps>... F<feed> M<accel>");
                         nos::println_to(os, "  Example: abspulses X1000 Y500 F100 M500");
                         return 0;
                     }
                     command_absolute_pulses(argv.without(1), os);
                     return 0;
                 }},

                {"steps",
                 "Print current position in steps/pulses for all axes (no args)",
                 [this](const nos::argv &, nos::ostream &os) {
                     nos::println_to(os, current_steps());
                     return 0;
                 }},

                {"finishes",
                 "Print current/final position in mm for all axes (no args)",
                 [this](const nos::argv &, nos::ostream &os) {
                     nos::print_list_to(os, _final_position);
                     nos::println_to(os);
                     return 0;
                 }},

                {"gains",
                 "Print input scaling factors for all axes (no args)",
                 [this](const nos::argv &, nos::ostream &os) {
                     nos::print_list_to(os, gains);
                     nos::println_to(os);
                     return 0;
                 }},

                {"gears",
                 "Print steps_per_unit (gears) for all axes (no args)",
                 [this](const nos::argv &, nos::ostream &os) {
                     nos::print_list_to(os, _steps_per_unit);
                     nos::println_to(os);
                     return 0;
                 }},

                {"setgear",
                 "Set steps_per_unit for axis. Args: <axis> <value>. "
                 "Example: setgear X 100.5",
                 [this](const nos::argv &argv, nos::ostream &os) {
                     if (argv.size() < 3)
                     {
                         nos::println_to(os, "Usage: setgear <axis> <steps_per_unit>");
                         nos::println_to(os, "  Example: setgear X 100.5");
                         return 0;
                     }
                     auto axno = symbol_to_index(argv[1][0]);
                     cnc_float_type val = igris_atof64(argv[2].data(), NULL);
                     set_steps_per_unit(axno, val);
                     feedback_guard->set_control_to_drive_multiplier(axno, val);
                     return 0;
                 }},

                {"set_control_gear",
                 "Set control gear (steps_per_unit). Args: <axis> <value>. "
                 "Same as setgear",
                 [this](const nos::argv &argv, nos::ostream &os) {
                     if (argv.size() < 3)
                     {
                         nos::println_to(os, "Usage: set_control_gear <axis> <value>");
                         return 0;
                     }
                     auto axno = symbol_to_index(argv[1][0]);
                     cnc_float_type val = igris_atof64(argv[2].data(), NULL);
                     set_steps_per_unit(axno, val);
                     feedback_guard->set_control_to_drive_multiplier(axno, val);
                     return 0;
                 }},

                {"set_feedback_gear",
                 "Set feedback gear (encoder multiplier). Args: <axis> <value>",
                 [this](const nos::argv &argv, nos::ostream &os) {
                     if (argv.size() < 3)
                     {
                         nos::println_to(os, "Usage: set_feedback_gear <axis> <value>");
                         return 0;
                     }
                     auto axno = symbol_to_index(argv[1][0]);
                     cnc_float_type val = igris_atof64(argv[2].data(), NULL);
                     feedback_guard->set_feedback_to_drive_multiplier(axno,
                                                                      val);
                     return 0;
                 }},

                {"setpos",
                 "Set current position without moving. Args: <axis> <pos_mm>. "
                 "Example: setpos X 0",
                 [this](const nos::argv &argv, nos::ostream &os) {
                     if (argv.size() < 3)
                     {
                         nos::println_to(os, "Usage: setpos <axis> <position_mm>");
                         nos::println_to(os, "  Sets current position without moving (for homing)");
                         nos::println_to(os, "  Example: setpos X 0");
                         return 0;
                     }
                     auto axno = symbol_to_index(argv[1][0]);
                     cnc_float_type val_mm = igris_atof64(argv[2].data(), NULL);
                     system_lock();
                     _final_position[axno] = val_mm;
                     steps_t steps = static_cast<steps_t>(val_mm * _steps_per_unit[axno]);
                     revolver->get_steppers()[axno]->set_counter_value(steps);
                     feedback_guard->set_feedback_position(axno, val_mm);
                     system_unlock();
                     return 0;
                 }},

                {"disable_tandem_protection",
                 "Disable tandem protection by index. Args: <index>",
                 [this](const nos::argv &argv, nos::ostream &os) {
                     if (argv.size() < 2)
                     {
                         nos::println_to(os, "Usage: disable_tandem_protection <index>");
                         return 0;
                     }
                     auto idx = std::stoi(argv[1]);
                     feedback_guard->remove_tandem(idx);
                     return 0;
                 }},

                {"enable_tandem_protection",
                 "Enable tandem protection. Args: <master_axis>,<slave_axis>:<max_error>",
                 [this](const nos::argv &argv, nos::ostream &os) {
                     if (argv.size() < 2)
                     {
                         nos::println_to(os, "Usage: enable_tandem_protection <master>,<slave>:<max_error>");
                         nos::println_to(os, "  Example: enable_tandem_protection 0,1:10");
                         return 0;
                     }
                     feedback_guard->add_tandem_command(argv.without(1), os);
                     return 0;
                 }},

                {"tandem_info",
                 "Print info about configured tandem axes (no args)",
                 [this](const nos::argv &, nos::ostream &os) {
                     const auto &tandems = feedback_guard->tandems();
                     if (tandems.size() == 0)
                     {
                         nos::println_to(os, "Tandem list is empty");
                     }
                     for (auto &tandem : tandems)
                     {
                         nos::println_to(os, tandem.info());
                     }
                     return 0;
                 }},

                {"drop_pulses_allowed",
                 "Get/set max following error. Args: <axis> [<pulses>]. "
                 "Without value - prints current",
                 [this](const nos::argv &argv, nos::ostream &os) {
                     if (argv.size() < 2)
                     {
                         nos::println_to(os, "Usage: drop_pulses_allowed <axis> [<max_pulses>]");
                         nos::println_to(os, "  Without value - prints current setting");
                         return 0;
                     }
                     size_t no = std::stoi(argv[1]);

                     if (argv.size() > 2)
                     {
                         int64_t pulses = std::stoi(argv[2]);
                         feedback_guard->set_drop_pulses_allowed(no, pulses);
                     }
                     else
                     {
                         nos::println_to(
                             os, feedback_guard->drop_pulses_allowed(no));
                     }
                     return 0;
                 }},

                {"velmaxs",
                 "Set max velocities (mm/s). Args: <idx>:<vel>... "
                 "Example: velmaxs 0:100 1:100 2:50",
                 [this](const nos::argv &argv, nos::ostream &os) {
                     if (argv.size() < 2)
                     {
                         nos::println_to(os, "Usage: velmaxs <axis>:<velocity>...");
                         nos::println_to(os, "  Example: velmaxs 0:100 1:100 2:50");
                         return 0;
                     }
                     auto fmap = args_to_index_value_map(argv.without(1));
                     for (auto &[key, val] : fmap)
                     {
                         max_axes_velocities[key] = val;
                     }
                     return 0;
                 }},

                {"accmaxs",
                 "Set max accelerations (mm/s^2). Args: <idx>:<acc>... "
                 "Example: accmaxs 0:1000 1:1000 2:500",
                 [this](const nos::argv &argv, nos::ostream &os) {
                     if (argv.size() < 2)
                     {
                         nos::println_to(os, "Usage: accmaxs <axis>:<acceleration>...");
                         nos::println_to(os, "  Example: accmaxs 0:1000 1:1000 2:500");
                         return 0;
                     }
                     auto fmap = args_to_index_value_map(argv.without(1));
                     for (auto &[key, val] : fmap)
                         max_axes_accelerations[key] = val;
                     return 0;
                 }},

                {"help",
                 "Print all commands (text + G-code)",
                 [this](const nos::argv &, nos::ostream &os) {
                     nos::println_to(os, "=== Text commands ===");
                     command_help(os);
                     nos::println_to(os, "\n=== G-code commands ===");
                     gcode_help(os);
                     return 0;
                 }},

                {"help-cmd",
                 "Print text commands help",
                 [this](const nos::argv &, nos::ostream &os) {
                     command_help(os);
                     return 0;
                 }},

                {"help-cnc",
                 "Print G-code commands help",
                 [this](const nos::argv &, nos::ostream &os) {
                     gcode_help(os);
                     return 0;
                 }},

                {"state",
                 "Print interpreter state: freq, vel, acc, gears, etc (no args)",
                 [this](const nos::argv &, nos::ostream &os) {
                     return print_interpreter_state(os);
                 }},

                {"guard_info",
                 "Print feedback guard state and errors (no args)",
                 [this](const nos::argv &, nos::ostream &os) {
                     return feedback_guard->guard_info(os);
                 }},

                {"planner_pause",
                 "Pause/resume planner. Args: <0|1>. 1=pause, 0=resume",
                 [this](const nos::argv &argv, nos::ostream &os) {
                     if (argv.size() < 2)
                     {
                         nos::println_to(os, "Usage: planner_pause <0|1>");
                         nos::println_to(os, "  1 = pause, 0 = resume");
                         return 0;
                     }
                     auto en = std::stoi(argv[1]);
                     planner->set_pause_mode(en);
                     return 0;
                 }}};

        int gcode_drop_first(const nos::argv &argv, nos::ostream &os)
        {
            return gcode(argv.without(1), os);
        }

        int command_drop_first(const nos::argv &argv, nos::ostream &os)
        {
            if (argv.size() == 1)
            {
                nos::println_to(os, "No command");
                return command_help(os);
            }

            return command(argv.without(1), os);
        }

        std::vector<int64_t> current_steps()
        {
            std::vector<int64_t> vect(total_axes);
            revolver->current_steps(vect.data());
            return vect;
        }

        int print_interpreter_state(nos::ostream &os)
        {
            PRINTTO(os, revolver_frequency);
            PRINTTO(os, saved_acc);
            PRINTTO(os, saved_feed);
            nos::print_to(os, "cur_vel: ");
            nos::print_list_to(os, revolver->current_velocities());
            nos::println_to(os);
            nos::print_to(os, "cur_acc:");
            nos::print_list_to(os, planner->accelerations);
            nos::println_to(os);
            nos::print_to(os, "velmaxs:");
            nos::print_list_to(os, max_axes_velocities);
            nos::println_to(os);
            nos::print_to(os, "accmaxs:");
            nos::print_list_to(os, max_axes_accelerations);
            nos::println_to(os);
            nos::print_to(os, "gains:");
            nos::print_list_to(os, gains);
            nos::println_to(os);
            nos::print_to(os, "gears:");
            nos::print_list_to(os, _steps_per_unit);
            nos::println_to(os);
            nos::print_to(os, "active_block: ");
            nos::println_to(os, planner->active_block != nullptr);
            nos::print_to(os, "active: ");
            nos::println_to(os, planner->active);
            nos::print_to(os, "head: ");
            nos::println_to(os, planner->blocks->head_index());
            return 0;
        }

        // Check if command is G-code (starts with G or M + digit)
        static bool is_gcode_command(const std::string_view &cmd)
        {
            if (cmd.size() < 2)
                return false;
            char c = cmd[0];
            return (c == 'G' || c == 'M') && std::isdigit(cmd[1]);
        }

        // Legacy executor for backward compatibility with cmd/cnc prefixes
        nos::executor executor = nos::executor({
            {"cmd",
             "text commands (legacy prefix)",
             nos::make_delegate(&interpreter::command_drop_first, this)},
            {"cnc",
             "gcode commands (legacy prefix)",
             nos::make_delegate(&interpreter::gcode_drop_first, this)},
        });

        std::string newline(const char *line, size_t)
        {
            nos::string_buffer output;
            auto tokens = nos::tokens(line);

            if (tokens.empty())
                return output.str();

            const auto &first = tokens[0];

            // Check for legacy prefixes (backward compatibility)
            if (first == "cmd" || first == "cnc")
            {
                executor.execute(tokens, output);
                return output.str();
            }

            // Auto-detect command type
            if (is_gcode_command(first))
            {
                // G-code command
                gcode(nos::argv(tokens), output);
            }
            else
            {
                // Text command
                command(nos::argv(tokens), output);
            }

            return output.str();
        }

        std::string newline(const std::string &line)
        {
            return newline(line.c_str(), line.size());
        }

        void set_revolver_frequency(cnc_float_type freq)
        {
            revolver_frequency = freq;
        }

        void set_axes_count(int total)
        {
            total_axes = total;
        }

        size_t get_axes_count()
        {
            return total_axes;
        }

        // Steps per mm management (gears)
        void set_steps_per_unit(int axis, cnc_float_type value)
        {
            if (axis >= 0 && axis < (int)NMAX_AXES)
                _steps_per_unit[axis] = value;
        }

        cnc_float_type get_steps_per_unit(int axis) const
        {
            if (axis >= 0 && axis < (int)NMAX_AXES)
                return _steps_per_unit[axis];
            return 0;
        }

        const std::array<cnc_float_type, NMAX_AXES> &steps_per_unit() const
        {
            return _steps_per_unit;
        }

        void set_gears(const igris::array_view<cnc_float_type> &arr)
        {
            for (size_t i = 0; i < arr.size() && i < NMAX_AXES; ++i)
                _steps_per_unit[i] = arr[i];
        }
    };
}

#endif
