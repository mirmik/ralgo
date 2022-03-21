#ifndef RALGO_CNC_INTERPRETER_H
#define RALGO_CNC_INTERPRETER_H

#include <cstdlib>
#include <string.h>
#include <stdlib.h>
#include <igris/container/array_view.h>
#include <igris/container/ring.h>
#include <igris/datastruct/argvc.h>
#include <igris/container/static_vector.h>
#include <igris/sync/syslock.h>

#include <ralgo/cnc/planblock.h>
#include <ralgo/cnc/control_task.h>
#include <ralgo/linalg/vecops.h>
#include <ralgo/log.h>

#include <ralgo/global_protection.h>
#include <ralgo/cnc/planner.h>
#include <ralgo/cnc/revolver.h>
#include <nos/shell/executor.h>
#include <nos/io/string_writer.h>

#include <ralgo/cnc/util.h> 
#include <string>

namespace cnc
{
    class interpreter
    {
    public:
        cnc::planner *planner;
        cnc::revolver *revolver;
        
    private:
        igris::ring<planner_block> *blocks;
        int total_axes = 0;
        double revolver_frequency = 0;
        igris::static_vector<double, NMAX_AXES> ext2int_scale;
        igris::static_vector<double, NMAX_AXES> final_position;
        igris::static_vector<double, NMAX_AXES> max_axes_velocities;    
        igris::static_vector<double, NMAX_AXES> max_axes_acellerations;    
        int blockno = 0;
        double saved_acc = 0;
        double saved_feed = 0;

        igris::delegate<void> _external_final_shift_handle; 
        planner_block lastblock;
    
    public:
        interpreter(igris::ring<planner_block> *blocks, cnc::planner *planner,
                    cnc::revolver *revolver)
            : planner(planner), revolver(revolver), blocks(blocks)
        {
        }

        void set_final_shift_handler(const igris::delegate<void>& dlg) 
        {
            _external_final_shift_handle = dlg;
        }

        void init_axes(int total_axes) 
        {
            this->total_axes = total_axes;
            ext2int_scale.resize(total_axes); 
            final_position.resize(total_axes);
            max_axes_velocities.resize(total_axes);
            max_axes_acellerations.resize(total_axes);
            planner->set_axes_count(total_axes);
            ralgo::vecops::fill(ext2int_scale, 1);
            ralgo::vecops::fill(final_position, 0);
            ralgo::vecops::fill(max_axes_acellerations, 0);
            ralgo::vecops::fill(max_axes_velocities, 0);
            revolver->final_shift_pushed = igris::make_delegate(&interpreter::final_shift_handle, this);
        }

        void restore_finishes() 
        {
                auto steps = revolver->current_steps();
                for (int i=0 ; i < total_axes; ++i)
                    final_position[i] = steps[i] * planner->gears[i]; 
        }

        void final_shift_handle() 
        {
            if (blocks->avail() == 0)
                restore_finishes();

            if (_external_final_shift_handle)
                _external_final_shift_handle();
        }

        int check_correctness(nos::ostream& os) 
        {
            assert(total_axes == (int)planner->get_total_axes());
            assert(total_axes == revolver->steppers_total);

            if (ralgo::global_protection) { 
                os.println("need_to_disable_global_protection"); return 1; }    
            //if (saved_feed == 0) { 
            //    os.println("saved_feed is null"); return 1; }
            //if (saved_acc == 0) { 
            //    os.println("saved_acc is null"); return 1; }
            if (revolver_frequency == 0) { 
                os.println("revolver_frequency is null"); return 1; }
            return 0;
        }

        control_task g1_parse_task(const nos::argv& argv) 
        {
            control_task task;

            task.feed = saved_feed;
            task.acc = saved_acc;
            int sts = task.parse(argv);
            if (sts) 
            {
                ralgo::warn("wrong task format");
                task.isok = false;
                return task;
            }

            task.isok = true;
            return task;            
        }

        //void set_start_operation_handle(const igris::delegate<void>& dlg) 
        //{
        //    planner.set_start_operation_handle(dlg);
        //}

        control_task g1_parse_task(const nos::argv& argv, igris::array_view<double> fposes) 
        {
            control_task task;
            task.set_poses({fposes.data(), fposes.size()});

            task.feed = saved_feed;
            task.acc = saved_acc;
            int sts = task.parse(argv);
            if (sts) 
            {
                ralgo::warn("wrong task format");
                task.isok = false;
                return task;
            }
            
            saved_feed = task.feed;
            saved_acc = task.acc;
            task.isok = true;
            return task;            
        }

        double evaluate_external_accfeed(
            ralgo::vector_view<double> direction,
            double absmax,
            const igris::static_vector<double, NMAX_AXES>& elmax
        ) 
        {
            double minmul = std::numeric_limits<double>::max();
            auto vecnorm = ralgo::vecops::norm(elmax);
            
            if (absmax == 0 && vecnorm == 0)
                return 0;

            if (absmax != 0) 
            {
                minmul = absmax;
            }

            //auto muls = ralgo::vecops::div_vv(elmax, direction);
            for (int i = 0; i < total_axes; i++) 
            {
                double lmul = elmax[i] / fabs(direction[i]);
                if (lmul == 0) continue;
                if (minmul > lmul) minmul = lmul; 
            }
            return minmul;
        }

        bool evaluate_interpreter_task(
            const control_task& task, 
            planner_block& block, 
            nos::ostream&) 
        {
            double tasknorm = ralgo::vecops::norm(task.poses(total_axes));
            if (tasknorm == 0)
                return true;

            auto intdists = ralgo::vecops::mul_vv<ralgo::vector<double>>(
                task.poses(total_axes), ext2int_scale);
            auto direction = ralgo::vecops::normalize<ralgo::vector<double>>(
                task.poses(total_axes));
            auto dirgain = ralgo::vecops::norm(
                ralgo::vecops::mul_vv(direction, ext2int_scale));

            auto evalfeed = evaluate_external_accfeed(direction, task.feed, max_axes_velocities);
            auto evalacc = evaluate_external_accfeed(direction, task.acc, max_axes_acellerations);

            if (evalacc == 0) 
            {
                ralgo::info("Acceleration is not defined");
                return true;
            }

            if (evalfeed == 0) 
            {
                ralgo::info("Feed is not defined");
                return true;
            }

            double feed = evalfeed * dirgain;
            double acc = evalacc * dirgain;
            
            // scale feed and acc by revolver freqs settings. 
            double reduced_feed = feed / revolver_frequency;
            double reduced_acc =
                acc / (revolver_frequency * revolver_frequency);

            // Eval fractions 
            auto fractions = ralgo::vecops::normalize<>(intdists);

            // output
            block.set_state(intdists, total_axes, reduced_feed, reduced_acc,
                            fractions);
            return false;
        }

        std::vector<double> final_gained_position() 
        {
            std::vector<double> ret(total_axes);
            for (int i = 0; i < total_axes; ++i) 
                ret[i] = final_position[i] / ext2int_scale[i];
            return ret;   
        }

        void set_scale(const ralgo::vector_view<double>& vec) 
        {
            if (vec.size() != static_cast<size_t>(total_axes)) 
            {
                ralgo::warn("set_scale fail");
            }
            std::copy(vec.begin(), vec.end(), ext2int_scale.begin());
        }

        void set_saved_acc(double acc) 
        {
            saved_acc = acc;
        }

        void set_saved_feed(double feed) 
        {
            saved_feed = feed;
        }

        void evaluate_task(const control_task& task, 
            nos::ostream& os) 
        {
            //ralgo::info("evaluate_task");
            
            // lastblock - out
            bool fastfinish = evaluate_interpreter_task(task, lastblock, os);
            if (fastfinish) {
                //ralgo::info("fastfinish block");
                print_interpreter_state(os);
                return;
            }   

            for (int i=0; i < total_axes; ++i)
                final_position[i] += lastblock.axdist[i];

            auto &placeblock = blocks->head_place();
            lastblock.blockno = blockno++;
            placeblock = lastblock;
            
            system_lock();
            blocks->move_head_one();
            system_unlock();
        }

        void command_incremental_move(const nos::argv& argv, nos::ostream& os)
        {
            if(check_correctness(os)) return;
            auto task = g1_parse_task(argv);
            if (!task.isok) return;
            evaluate_task(task, os);
        }

        void command_absolute_move(const nos::argv& argv, nos::ostream& os)
        {
            if(check_correctness(os)) return;
            auto finalpos = final_gained_position();
            auto task = g1_parse_task(argv, {finalpos.data(), finalpos.size()});
            if (!task.isok) return;
            for (int i = 0; i < total_axes; ++i) 
            {
                task.poses(total_axes)[i] = task.poses(total_axes)[i] - finalpos[i];       
            }
            evaluate_task(task, os);
        }

        void inspect_args(const nos::argv& argv, nos::ostream& os) 
        {
            PRINTTO(os, argv.size());
            for (auto& arg : argv) 
            {
                PRINTTO(os,arg);
            }
        }

        void command_M204(const nos::argv& argv, nos::ostream& os)
        {
            if (argv.size() == 0)
            {
                os.print("%f", saved_acc);
                return;
            }

            inspect_args(argv,os);

            saved_acc = strtod(argv[0].data(), nullptr);
            PRINTTO(os, saved_acc);
        }

        // Включить режим остановки.
        void command_M112(const nos::argv&, nos::ostream&)
        {
            system_lock();
            //blocks->clear();
            //plan_stop_task();
            system_unlock();
        }

        void smooth_stop(nos::ostream& os)
        {
            system_lock();
            auto curvels = revolver->current_velocity_no_lock(os);
            
            if (planner->active_block == nullptr) 
            {
                restore_final_position_by_steps();
                planner->clear();
                revolver->clear();
                restore_finishes();
                system_unlock();
                return;
            }

            planner->clear_queue();
            planner->active_block->immedeate_smooth_stop(planner->iteration_counter);
            system_unlock();
        }

        void restore_final_position_by_steps() 
        {
            auto steps = revolver->current_steps();
            auto fpos = ralgo::vecops::mul_vv(steps, planner->get_gears());
            std::copy(fpos.begin(), fpos.end(), final_position.begin());
        }

        void g_command(const nos::argv& argv, nos::ostream& os)
        {
            int cmd = atoi(&argv[0].data()[1]);
            switch (cmd)
            {
            case 1:
                command_incremental_move(argv.without(1), os);
                break;
            default:
                os.println("Unresolved G command");
            }
        }

        void m_command(const nos::argv& argv, nos::ostream& os)
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
                os.println("Unresolved M command");
            }
        }

        int gcode(const nos::argv& argv, nos::ostream& os)
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
                    os.println("Unresolved command");
            }

            return 0;
        }

        int command(const nos::argv& argv, nos::ostream& os)
        {
            if (argv.size() == 0)
                return 0;

            if (argv[0] == "setprotect") 
            {
                ralgo::global_protection = false;
                return 0;
            }

            if (argv[0] == "stop") 
            {
                smooth_stop(os);
                return 0;
            }

            else if (argv[0] == "axmaxspd") 
            {
                os.println("TODO");
                return 0;
            }

            else if (argv[0] == "axmaxacc") 
            {
                os.println("TODO");
                return 0;
            }

            else if (argv[0] == "maxspd") 
            {
                os.println("TODO");
                return 0;
            }

            else if (argv[0] == "maxacc") 
            {
                os.println("TODO");
                return 0;
            }

            else if (argv[0] == "relmove") 
            {
                command_incremental_move(argv.without(1), os);
                return 0;
            }

            else if (argv[0] == "absmove") 
            {
                command_absolute_move(argv.without(1), os);
                return 0;
            }

            else if (argv[0] == "steps") 
            {
                 return os.println(current_steps());   
            }
            
            else if (argv[0] == "finishes") 
            {
                 nos::print_list_to(os,final_position);   
                 return os.println();
            }

            else if (argv[0] == "gains") 
            {
                 nos::print_list_to(os, ext2int_scale);   
                 return os.println();
            }

            else if (argv[0] == "setgear") 
            {
                auto axno = symbol_to_index(argv[1][0]);
                double val = strtod(argv[2].data(), NULL);
                planner->set_gear(axno, val);
                return 0;
            }

            else if (argv[0] == "setpos") 
            {
                auto axno = symbol_to_index(argv[1][0]);
                double val = strtod(argv[2].data(), NULL);
                system_lock();
                final_position[axno] = val * planner->gears[axno];                
                revolver->get_steppers()[axno]->set_counter_value(val);
                system_unlock();
                return 0;
            }

            else if (argv[0] == "velmaxs") 
            {
                auto axes = argv.size() - 1;
                if (axes != (size_t)total_axes) 
                {
                    os.println("wrong axes count");
                    return 0;
                }
                for (size_t i = 0; i < axes; ++i) 
                {
                    max_axes_velocities[i] = strtod(argv[i+1].data(), NULL);
                }
                return 0;
            }

            else if (argv[0] == "accmaxs") 
            {
                auto axes = argv.size() - 1;
                if (axes != (size_t)total_axes) 
                {
                    os.println("wrong axes count");
                    return 0;
                }
                for (size_t i = 0; i < axes; ++i) 
                {
                    max_axes_acellerations[i] = strtod(argv[i+1].data(), NULL);
                }
                return 0;
            }

            else if (argv[0] == "lastblock") 
            {
                 return os.println(lastblock);   
            }
            
            else if (argv[0] == "state") 
            {
                 return print_interpreter_state(os);   
            }

            else if (argv[0] == "simulator") 
            {
                revolver->enable_simulator_mode();
                return 0;   
            }
            
            os.println("Unresolved command");
            return 0;
        }

        int gcode_drop_first(const nos::argv& argv, nos::ostream& os)
        {
            return gcode(argv.without(1), os);
        }

        int command_drop_first(const nos::argv& argv, nos::ostream& os)
        {
            return command(argv.without(1), os);
        }

        std::vector<int64_t> current_steps()
        {
            std::vector<int64_t> vect(total_axes);
            revolver->current_steps(vect.data());
            return vect;
        }

        int print_interpreter_state(nos::ostream& os) 
        {
            PRINTTO(os, revolver_frequency);
            PRINTTO(os, saved_acc);
            PRINTTO(os, saved_feed);
            PRINTTO(os, revolver->shifts_ring->head_index());
            PRINTTO(os, revolver->shifts_ring->tail_index());
            nos::print_to(os, "vel: "); nos::print_list_to(os, planner->velocities); nos::println_to(os);
            nos::print_to(os, "acc:"); nos::print_list_to(os, planner->accelerations); nos::println_to(os);
            nos::print_to(os, "gains:"); nos::print_list_to(os, ext2int_scale); nos::println_to(os);
            nos::print_to(os, "gears:"); nos::print_list_to(os, planner->get_gears()); nos::println_to(os);
            nos::print_to(os, "active_block: "); nos::println_to(os, planner->active_block != nullptr);
            nos::print_to(os, "active: "); nos::println_to(os, planner->active);
            nos::print_to(os, "head: "); nos::println_to(os, planner->blocks->head_index());
            return 0;
        }

        nos::executor executor = nos::executor({
            {"cmd", "commands", nos::make_delegate(&interpreter::command_drop_first, this)},
            {"cnc", "gcode commands", nos::make_delegate(&interpreter::gcode_drop_first, this)},
        });

        void newline(const char *line, size_t)
        {
            nos::string_buffer output;
            executor.execute(nos::tokens(line), output);
        }

        void newline(const std::string &line)
        {
            newline(line.data(), line.size());
        }

        void set_revolver_frequency(double freq) { 
            revolver_frequency = freq; 
        }

        void set_axes_count(int total) { 
            total_axes = total;
        }

        size_t get_axes_count() 
        {
            return total_axes;
        }
    };
}

#endif
