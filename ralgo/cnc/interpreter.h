#ifndef RALGO_CNC_INTERPRETER_H
#define RALGO_CNC_INTERPRETER_H

#include <cstdlib>
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

#include <string>

namespace cnc
{
    class interpreter
    {
        //static constexpr char alphabet[9] = {'X', 'Y', 'Z', 'A', 'B',
        //                                     'C', 'I', 'J', 'K'};

    private:
        igris::ring<planner_block> *blocks;
        cnc::planner *planner;
        cnc::revolver *revolver;
        int total_axes = 0;
        double revolver_frequency = 0;
        igris::static_vector<double, NMAX_AXES> ext2int_scale;
        igris::static_vector<double, NMAX_AXES> final_position;
        int blockno = 0;
        double saved_acc = 0;
        double saved_feed = 0;

    public:
        interpreter(igris::ring<planner_block> *blocks, cnc::planner *planner,
                    cnc::revolver *revolver)
            : blocks(blocks), planner(planner), revolver(revolver)
        {}

        void init_axes(int total_axes) 
        {
            this->total_axes = total_axes;
            ext2int_scale.resize(total_axes); 
            final_position.resize(total_axes);
        }

        int check_correctness(nos::ostream& os) 
        {
            if (ralgo::global_protection) { 
                os.println("need_to_disable_global_protection"); return 1; }    
            if (saved_feed == 0) { 
                os.println("saved_feed is null"); return 1; }
            if (saved_acc == 0) { 
                os.println("saved_acc is null"); return 1; }
            if (revolver_frequency == 0) { 
                os.println("revolver_frequency is null"); return 1; }
            return 0;
        }

        control_task g1_parse_task(const nos::argv& argv) 
        {
            control_task task;
            memset(&task, 0, sizeof(task));

            task.feed = saved_feed;
            task.acc = saved_acc;
            int sts = task.parse(argv);
            if (sts) 
            {
                ralgo::warn("wrong task format");
                task.isok = false;
                return task;
            }

            if (task.feed == 0 || task.acc == 0)
            {
                ralgo::warn("nullvelocity block. ignore.");
                task.isok = false;
                return task;
            }
            task.isok = true;
            return task;            
        }

        /*double evaluate_steps_array(
            const control_task& task, 
            int64_t * steps) 
        {
            memset(steps, 0, sizeof(int64_t)*total_axes);
            double accum = 0;
            for (int i = 0; i < total_axes; ++i)
            {
                steps[i] = task.poses[i] * gains[i];
                accum += steps[i] * steps[i];
            }
            return accum;
        }*/

        /*double evaluate_dists_array(
            const control_task& task, 
            double * dists) 
        {
            memset(dists, 0, sizeof(double)*total_axes);
            double accum = 0;
            for (int i = 0; i < total_axes; ++i)
            {
                dists[i] = task.poses()[i];
                accum += dists[i] * dists[i];
            }   
            return accum;
        }*/

        void evaluate_interpreter_task(
            const control_task& task, 
            planner_block& block, 
            nos::ostream&) 
        {
            auto intdists = ralgo::vecops::mul_vv<ralgo::vector<double>>(
                task.poses(total_axes), ext2int_scale);
            auto direction = ralgo::vecops::normalize<ralgo::vector<double>>(
                task.poses(total_axes));
            auto dirgain = ralgo::vecops::norm(
                ralgo::vecops::mul_vv(direction, ext2int_scale));

            double feed = task.feed * dirgain;
            double acc = task.acc * dirgain;
            
            // scale feed and acc by revolver freqs settings. 
            double reduced_feed = feed / revolver_frequency;
            double reduced_acc =
                acc / (revolver_frequency * revolver_frequency);

            // Eval fractions 
            auto fractions = ralgo::vecops::normalize<>(intdists);

            // output
            block.set_state(intdists, total_axes, reduced_feed, reduced_acc,
                            fractions);
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
            ext2int_scale = vec;
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
            auto poses = task.poses(total_axes);
            auto &block = blocks->head_place();
            evaluate_interpreter_task(task, block, os);
            block.blockno = blockno++;

            for (int i=0; i < total_axes; ++i)
                final_position[i] += poses[i];

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
            auto task = g1_parse_task(argv);
            if (!task.isok) return;
            auto curpos = final_gained_position();
            for (int i = 0; i < total_axes; ++i) 
            {
                task.poses(total_axes)[i] = task.poses(total_axes)[i] - curpos[i];       
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

        /*void plan_stop_task()
        {
            float reduced_velocity[NMAX_AXES];
            revolver->current_velocity(reduced_velocity);
            auto fractions = fractions_from_velocity(reduced_velocity);
            auto feed = feed_from_reduced_vector(reduced_velocity);
            auto acc = saved_acc;
            auto time = feed / acc * 2;
            auto dists = stop_pattern_distance(feed, acc, time, fractions);
            auto steps = steps_from_dists(dists);

            auto &block = blocks->head_place();
            block.set_stop_pattern(
                steps, 
                total_axes, 
                reduced_feed,
                reduced_acc, 
                multipliers);
            system_lock();
            full_state_clean();
            blocks->move_head_one();
            system_unlock();
        }*/

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
                os.println("TODO");
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

            else if (argv[0] == "poses") 
            {
                 return os.println(current_poses());   
            }

            else if (argv[0] == "gains") 
            {
                 //return os.println(ext2int_scale);   
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
        //    std::vector<int64_t> vect(total_axes);
        //    revolver->current_steps(vect.data());
        //    return vect;
            return {0,0,0};
        }

        std::vector<double> current_poses()
        {
        //    std::vector<int64_t> steps(total_axes);
        //    std::vector<double> poses(total_axes);
        //    revolver->current_steps(steps.data());
        //    for (int i = 0; i < total_axes; ++i)
        //        poses[i] = steps[i] / ext2int_scale[i];
        //    return poses;
            return {0,0,0};
        }

        int print_interpreter_state(nos::ostream& os) 
        {
            PRINTTO(os, saved_feed);
            PRINTTO(os, saved_acc);
            //PRINTTO(os, gains);
            PRINTTO(os, revolver_frequency);
            return 0;
        }

        int cmdinfo(const nos::argv& argv, nos::ostream& os) 
        {
            if (argv.size() <= 1) 
            {
                os.println(R"(Need subcmd:
Commands:
    poses
)");
                return 0;
            }

            os.println(argv.size());
            for(auto& arg : argv) 
            {
                os.println(arg);
            }

            if (argv[1] == "steps") 
            {
                return os.println(current_steps());
            }
            
            if (argv[1] == "state") 
                return print_interpreter_state(os);
    
            os.println("Wrong subcommand");
            return 0;
        }

        nos::executor executor = nos::executor({
            {"cmd", "commands", nos::make_delegate(&interpreter::command_drop_first, this)},
            {"cnc", "gcode commands", nos::make_delegate(&interpreter::gcode_drop_first, this)},
            {"cncinfo", "cmdinfo", nos::make_delegate(&interpreter::cmdinfo, this)},
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
    };
}

#endif
