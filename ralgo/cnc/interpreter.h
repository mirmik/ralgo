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
#include <ralgo/cnc/buffer_controller.h>
#include <ralgo/cnc/cnc_math.h>
#include <ralgo/cnc/control_task.h>
#include <ralgo/cnc/error_handler.h>
#include <ralgo/cnc/feedback_guard.h>
#include <ralgo/cnc/flow_controller.h>
#include <ralgo/cnc/lookahead.h>
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
        cnc::flow_controller *flow = nullptr;

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
        bool _absolute_mode = true; // G90 by default (стандарт G-code)
        cnc_float_type _smooth_stop_acceleration = 0;
        igris::delegate<void> _external_final_shift_handle = {};
        planner_block lastblock = {};
        bool with_cleanup = true;
        volatile bool stop_procedure_started = false;

        // Look-ahead parameters (junction deviation algorithm)
        cnc::lookahead_params _lookahead;

        // Buffering controller for look-ahead optimization
        cnc::buffer_controller _buffer;

    public:
        interpreter(igris::ring<planner_block> *blocks,
                    cnc::planner *planner,
                    cnc::revolver *revolver,
                    cnc::feedback_guard *feedback_guard,
                    cnc::error_handler *errors = nullptr,
                    cnc::flow_controller *flow = nullptr)
            : planner(planner), revolver(revolver),
              feedback_guard(feedback_guard), errors(errors), flow(flow),
              blocks(blocks)
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
            return cnc::evaluate_external_accfeed_2(
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

            // Initialize flow controller with queue capacity
            if (flow)
                flow->set_queue_capacity(blocks->size());
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

                // Notify flow controller that motion is complete
                if (flow)
                    flow->report_complete();

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

            auto evalfeed = cnc::evaluate_external_accfeed_2(
                direction, task.feed, max_axes_velocities);
            auto evalacc = cnc::evaluate_external_accfeed_2(
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
            // Check queue has room before processing
            if (blocks->room() == 0)
            {
                report_error(error_code::queue_overflow);
                if (flow)
                    flow->report_full(blocks->avail());
                return;
            }

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

            // Запоминаем время первого блока для таймаута буферизации
            bool was_empty = blocks->empty() && planner->active_block == nullptr;
            if (was_empty)
            {
                _buffer.record_start_tick(planner->iteration_counter);

                // Включаем паузу если нужна автоматическая буферизация
                if (_buffer.should_pause_on_first_block())
                {
                    planner->set_pause_mode(true);
                }
            }

            auto &placeblock = blocks->head_place();
            lastblock.blockno = blockno++;
            placeblock = lastblock;

            system_lock();
            blocks->move_head_one();

            // Вызываем look-ahead пересчёт скоростей если он включен
            // (в явном режиме пересчёт будет при buffer_start)
            if (_lookahead.is_enabled() && !_buffer.is_explicit_mode())
            {
                planner->recalculate_block_velocities(_lookahead.junction_deviation_steps());
            }

            // Проверяем готовность буфера для автоматического режима
            if (_buffer.is_ready(blocks->avail(), planner->iteration_counter,
                                 planner->active_block != nullptr))
            {
                planner->set_pause_mode(false);
            }

            system_unlock();

            // Report successful command to flow controller
            if (flow)
                flow->report_ok(blocks->avail(), blocks->room(), lastblock.blockno);
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

        /// G0 - rapid move (uses max velocities)
        void command_rapid_move(const nos::argv &argv, nos::ostream &os)
        {
            if (stop_procedure_started)
                return;

            ralgo::info("command_rapid_move (G0)");

            if (check_correctness(os))
                return;

            auto poses = get_task_poses_from_argv(argv);
            cnc_float_type acc = get_task_acc_from_argv(argv);
            // G0 uses max velocity (feed=0 means use axis limits)
            cnc_float_type feed = 0;

            control_task task(total_axes);
            if (_absolute_mode)
                task = create_task_for_absolute_move(poses, feed, acc);
            else
                task = create_task_for_relative_move(poses, feed, acc);

            if (!task.isok)
                return;
            evaluate_task(task, os);
        }

        /// G1 - linear move with feed (respects G90/G91 mode)
        void command_linear_move(const nos::argv &argv, nos::ostream &os)
        {
            if (stop_procedure_started)
                return;

            ralgo::info("command_linear_move (G1)");

            if (check_correctness(os))
                return;

            auto poses = get_task_poses_from_argv(argv);
            cnc_float_type feed = get_task_feed_from_argv(argv);
            cnc_float_type acc = get_task_acc_from_argv(argv);

            control_task task(total_axes);
            if (_absolute_mode)
                task = create_task_for_absolute_move(poses, feed, acc);
            else
                task = create_task_for_relative_move(poses, feed, acc);

            if (!task.isok)
                return;
            evaluate_task(task, os);
        }

        /// G92 - set position without moving
        void command_G92(const nos::argv &argv, nos::ostream &os)
        {
            ralgo::info("command_G92 (set position)");

            auto poses = get_task_poses_from_argv(argv);
            if (poses.empty())
            {
                nos::println_to(os, "Usage: G92 <axis><pos>...");
                nos::println_to(os, "  Example: G92 X0 Y0 Z0");
                return;
            }

            system_lock();
            for (auto &p : poses)
            {
                int axno = p.idx;
                cnc_float_type val_mm = p.pos;
                _final_position[axno] = val_mm;
                steps_t steps = static_cast<steps_t>(val_mm * _steps_per_unit[axno]);
                revolver->get_steppers()[axno]->set_counter_value(steps);
                if (feedback_guard)
                    feedback_guard->set_feedback_position(axno, val_mm);
            }
            system_unlock();
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

        void g_command(const nos::argv &argv, nos::ostream &os);
        void m_command(const nos::argv &argv, nos::ostream &os);
        int gcode(const nos::argv &argv, nos::ostream &os);

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

        int command_help(nos::ostream &os);

        int gcode_help(nos::ostream &os);
        int print_interpreter_state(nos::ostream &os);

        // CLI command handlers (implemented in interpreter.cpp)
        int cmd_setprotect(const nos::argv &, nos::ostream &);
        int cmd_stop(const nos::argv &, nos::ostream &);
        int cmd_lastblock(const nos::argv &, nos::ostream &os);
        int cmd_relmove(const nos::argv &argv, nos::ostream &os);
        int cmd_absmove(const nos::argv &argv, nos::ostream &os);
        int cmd_abspulses(const nos::argv &argv, nos::ostream &os);
        int cmd_steps(const nos::argv &, nos::ostream &os);
        int cmd_finishes(const nos::argv &, nos::ostream &os);
        int cmd_gains(const nos::argv &, nos::ostream &os);
        int cmd_gears(const nos::argv &, nos::ostream &os);
        int cmd_setgear(const nos::argv &argv, nos::ostream &os);
        int cmd_set_control_gear(const nos::argv &argv, nos::ostream &os);
        int cmd_set_feedback_gear(const nos::argv &argv, nos::ostream &os);
        int cmd_setpos(const nos::argv &argv, nos::ostream &os);
        int cmd_disable_tandem_protection(const nos::argv &argv, nos::ostream &os);
        int cmd_enable_tandem_protection(const nos::argv &argv, nos::ostream &os);
        int cmd_tandem_info(const nos::argv &, nos::ostream &os);
        int cmd_drop_pulses_allowed(const nos::argv &argv, nos::ostream &os);
        int cmd_velmaxs(const nos::argv &argv, nos::ostream &os);
        int cmd_accmaxs(const nos::argv &argv, nos::ostream &os);
        int cmd_help(const nos::argv &, nos::ostream &os);
        int cmd_help_cmd(const nos::argv &, nos::ostream &os);
        int cmd_help_cnc(const nos::argv &, nos::ostream &os);
        int cmd_state(const nos::argv &, nos::ostream &os);
        int cmd_mode(const nos::argv &argv, nos::ostream &os);
        int cmd_guard_info(const nos::argv &, nos::ostream &os);
        int cmd_planner_pause(const nos::argv &argv, nos::ostream &os);
        int cmd_set_junction_deviation(const nos::argv &argv, nos::ostream &os);
        int cmd_queue_status(const nos::argv &, nos::ostream &os);
        int cmd_flow_control(const nos::argv &argv, nos::ostream &os);
        int cmd_buffer(const nos::argv &argv, nos::ostream &os);

        // CLI commands table - implementations in interpreter.cpp
        igris::static_callable_collection<int(const nos::argv &, nos::ostream &), 50>
            clicommands{
                {"setprotect", "Disable global protection",
                 [this](const nos::argv &a, nos::ostream &o) { return cmd_setprotect(a, o); }},
                {"stop", "Smooth stop all axes",
                 [this](const nos::argv &a, nos::ostream &o) { return cmd_stop(a, o); }},
                {"lastblock", "Print last motion block info",
                 [this](const nos::argv &a, nos::ostream &o) { return cmd_lastblock(a, o); }},
                {"relmove", "Relative move. Args: <axis><dist>... F<feed> M<accel>",
                 [this](const nos::argv &a, nos::ostream &o) { return cmd_relmove(a, o); }},
                {"absmove", "Absolute move. Args: <axis><pos>... F<feed> M<accel>",
                 [this](const nos::argv &a, nos::ostream &o) { return cmd_absmove(a, o); }},
                {"abspulses", "Absolute move in steps. Args: <axis><steps>... F<feed> M<accel>",
                 [this](const nos::argv &a, nos::ostream &o) { return cmd_abspulses(a, o); }},
                {"steps", "Print position in steps",
                 [this](const nos::argv &a, nos::ostream &o) { return cmd_steps(a, o); }},
                {"finishes", "Print position in mm",
                 [this](const nos::argv &a, nos::ostream &o) { return cmd_finishes(a, o); }},
                {"gains", "Print input scaling factors",
                 [this](const nos::argv &a, nos::ostream &o) { return cmd_gains(a, o); }},
                {"gears", "Print steps_per_unit",
                 [this](const nos::argv &a, nos::ostream &o) { return cmd_gears(a, o); }},
                {"setgear", "Set steps_per_unit. Args: <axis> <value>",
                 [this](const nos::argv &a, nos::ostream &o) { return cmd_setgear(a, o); }},
                {"set_control_gear", "Set control gear. Args: <axis> <value>",
                 [this](const nos::argv &a, nos::ostream &o) { return cmd_set_control_gear(a, o); }},
                {"set_feedback_gear", "Set feedback gear. Args: <axis> <value>",
                 [this](const nos::argv &a, nos::ostream &o) { return cmd_set_feedback_gear(a, o); }},
                {"setpos", "Set position without moving. Args: <axis> <pos_mm>",
                 [this](const nos::argv &a, nos::ostream &o) { return cmd_setpos(a, o); }},
                {"disable_tandem_protection", "Disable tandem protection. Args: <index>",
                 [this](const nos::argv &a, nos::ostream &o) { return cmd_disable_tandem_protection(a, o); }},
                {"enable_tandem_protection", "Enable tandem protection. Args: <master>,<slave>:<max_error>",
                 [this](const nos::argv &a, nos::ostream &o) { return cmd_enable_tandem_protection(a, o); }},
                {"tandem_info", "Print tandem axes info",
                 [this](const nos::argv &a, nos::ostream &o) { return cmd_tandem_info(a, o); }},
                {"drop_pulses_allowed", "Get/set max following error. Args: <axis> [<pulses>]",
                 [this](const nos::argv &a, nos::ostream &o) { return cmd_drop_pulses_allowed(a, o); }},
                {"velmaxs", "Set max velocities. Args: <idx>:<vel>...",
                 [this](const nos::argv &a, nos::ostream &o) { return cmd_velmaxs(a, o); }},
                {"accmaxs", "Set max accelerations. Args: <idx>:<acc>...",
                 [this](const nos::argv &a, nos::ostream &o) { return cmd_accmaxs(a, o); }},
                {"help", "Print all commands",
                 [this](const nos::argv &a, nos::ostream &o) { return cmd_help(a, o); }},
                {"help-cmd", "Print text commands",
                 [this](const nos::argv &a, nos::ostream &o) { return cmd_help_cmd(a, o); }},
                {"help-cnc", "Print G-code commands",
                 [this](const nos::argv &a, nos::ostream &o) { return cmd_help_cnc(a, o); }},
                {"state", "Print interpreter state",
                 [this](const nos::argv &a, nos::ostream &o) { return cmd_state(a, o); }},
                {"mode", "Print or set positioning mode. Args: [abs|rel]",
                 [this](const nos::argv &a, nos::ostream &o) { return cmd_mode(a, o); }},
                {"guard_info", "Print feedback guard state",
                 [this](const nos::argv &a, nos::ostream &o) { return cmd_guard_info(a, o); }},
                {"planner_pause", "Pause/resume planner. Args: <0|1>",
                 [this](const nos::argv &a, nos::ostream &o) { return cmd_planner_pause(a, o); }},
                {"set_junction_deviation", "Set junction deviation (mm). Args: [<value>]",
                 [this](const nos::argv &a, nos::ostream &o) { return cmd_set_junction_deviation(a, o); }},
                {"queue_status", "Print block queue status",
                 [this](const nos::argv &a, nos::ostream &o) { return cmd_queue_status(a, o); }},
                {"flow_control", "Enable/disable flow control. Args: [<0|1>]",
                 [this](const nos::argv &a, nos::ostream &o) { return cmd_flow_control(a, o); }},
                {"buffer", "Buffer control. Args: enable|start|cancel|status|config",
                 [this](const nos::argv &a, nos::ostream &o) { return cmd_buffer(a, o); }}};

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
            _buffer.set_frequency(freq);
        }

        void set_axes_count(int total)
        {
            total_axes = total;
        }

        size_t get_axes_count()
        {
            return total_axes;
        }

        // Positioning mode (G90/G91)
        void set_absolute_mode(bool absolute)
        {
            _absolute_mode = absolute;
        }

        bool is_absolute_mode() const
        {
            return _absolute_mode;
        }

        // Steps per mm management (gears)
        void set_steps_per_unit(int axis, cnc_float_type value)
        {
            if (axis >= 0 && axis < (int)NMAX_AXES)
            {
                _steps_per_unit[axis] = value;
                // Пересчитываем junction_deviation_steps при изменении gears
                _lookahead.update_steps_per_unit(_steps_per_unit, total_axes);
            }
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

        // === Look-ahead management ===

        /// Установить junction deviation в units (мм).
        /// Включает look-ahead если value > 0.
        /// @param value - допустимое отклонение в units (типично 0.01-0.05 мм)
        void set_junction_deviation(cnc_float_type value)
        {
            _lookahead.set_junction_deviation(value);
            _lookahead.update_steps_per_unit(_steps_per_unit, total_axes);
        }

        /// Получить текущее значение junction deviation в units.
        cnc_float_type get_junction_deviation() const
        {
            return _lookahead.junction_deviation();
        }

        /// Проверить, включен ли look-ahead.
        bool is_lookahead_enabled() const
        {
            return _lookahead.is_enabled();
        }

        // === Buffering management ===

        /// Установить минимальное количество блоков перед стартом.
        /// @param n - количество блоков (0 = без буферизации)
        void set_min_blocks_to_start(int n)
        {
            _buffer.set_min_blocks_to_start(n);
        }

        int get_min_blocks_to_start() const
        {
            return _buffer.min_blocks_to_start();
        }

        /// Установить таймаут буферизации в миллисекундах.
        /// @param ms - таймаут (0 = без таймаута)
        void set_buffer_timeout_ms(int ms)
        {
            _buffer.set_timeout_ms(ms);
        }

        int get_buffer_timeout_ms() const
        {
            return _buffer.timeout_ms();
        }

        /// Включить явный режим буферизации.
        /// Блоки накапливаются но не исполняются до вызова buffer_start().
        void buffer_enable()
        {
            _buffer.enable_explicit();
            planner->set_pause_mode(true);
        }

        /// Запустить накопленные блоки.
        /// Выполняет look-ahead пересчёт и запускает движение.
        void buffer_start()
        {
            if (_buffer.start_explicit())
            {
                // Пересчитываем скорости всех блоков перед стартом
                if (_lookahead.is_enabled())
                {
                    planner->recalculate_block_velocities(_lookahead.junction_deviation_steps());
                }
                planner->set_pause_mode(false);
            }
        }

        /// Отменить буферизацию и очистить очередь.
        void buffer_cancel()
        {
            _buffer.cancel_explicit();
            planner->set_pause_mode(false);
            planner->clear();
        }

        /// Проверить, активен ли режим буферизации.
        bool is_buffer_mode() const
        {
            return _buffer.is_explicit_mode();
        }

        /// Проверить, готов ли буфер к старту (автоматический режим).
        bool is_buffer_ready() const
        {
            return _buffer.is_ready(blocks->avail(), planner->iteration_counter,
                                    planner->active_block != nullptr);
        }

    };
}

#endif
