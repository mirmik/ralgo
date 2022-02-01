#ifndef RALGO_CNC_INTERPRETER_H
#define RALGO_CNC_INTERPRETER_H

#include <cstdlib>
#include <igris/container/array_view.h>
#include <igris/container/ring.h>
#include <igris/datastruct/argvc.h>
#include <igris/sync/syslock.h>

#include <ralgo/cnc/defs.h>
#include <ralgo/cnc/planblock.h>
#include <ralgo/log.h>

#include <ralgo/global_protection.h>
#include <ralgo/cnc/planner.h>
#include <ralgo/cnc/revolver.h>
#include <nos/shell/executor.h>
#include <nos/io/string_writer.h>

#include <string>

namespace cnc
{
    struct interpreter_control_task
    {
        bool isok;
        double poses[NMAX_AXES];
        double feed;
        double acc;

        int parse(const nos::argv& argv)
        {
            memset(poses, 0, sizeof(poses));

            for (unsigned int i = 0; i < argv.size(); ++i)
            {
                char symb = argv[i].data()[0];
                double val = atof(&argv[i].data()[1]);

                switch (symb)
                {
                case 'F':
                    feed = val;
                    continue;
                case 'M':
                    acc = val;
                    continue;
                case 'X':
                    poses[0] = val;
                    continue;
                case 'Y':
                    poses[1] = val;
                    continue;
                case 'Z':
                    poses[2] = val;
                    continue;
                case 'A':
                    poses[3] = val;
                    continue;
                case 'B':
                    poses[4] = val;
                    continue;
                case 'C':
                    poses[5] = val;
                    continue;
                case 'I':
                    poses[6] = val;
                    continue;
                case 'J':
                    poses[7] = val;
                    continue;
                case 'K':
                    poses[8] = val;
                    continue;
                default:
                    return -1;
                }
            }
            return 0;
        }
    };

    class interpreter
    {
        static constexpr char alphabet[9] = {'X', 'Y', 'Z', 'A', 'B',
                                             'C', 'I', 'J', 'K'};

    public:
        igris::ring<planner_block> *blocks;
        cnc::planner *planner;
        cnc::revolver *revolver;

        bool info_mode = false;

        int total_axes = 0;
        double revolver_frequency = 0;
        double gains[NMAX_AXES];
        int64_t final_steps[NMAX_AXES];
        int blockno = 0;
        double saved_acc = 0;
        double saved_feed = 0;

    public:
        interpreter(igris::ring<planner_block> *blocks, cnc::planner *planner,
                    cnc::revolver *revolver)
            : blocks(blocks), planner(planner), revolver(revolver)
        {
            memset(gains, 0, sizeof(gains));
            memset(final_steps, 0, sizeof(final_steps));
            for (auto &gain : gains) gain = 1;
        }

        void evaluate_fractions(double *fractions_out, const int64_t *steps)
        {
            int64_t accum = 0;
            for (int i = 0; i < total_axes; ++i)
            {
                accum += steps[i] * steps[i];
            }

            double dist = sqrt(accum);
            for (int i = 0; i < total_axes; ++i)
            {
                fractions_out[i] = (double)steps[i] / dist;
            }
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

        interpreter_control_task g1_parse_task(const nos::argv& argv) 
        {
            interpreter_control_task task;
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

        double evaluate_steps_array(
            const interpreter_control_task& task, 
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
        }

        double evaluate_dists_array(
            const interpreter_control_task& task, 
            double * dists) 
        {
            memset(dists, 0, sizeof(double)*total_axes);
            double accum = 0;
            for (int i = 0; i < total_axes; ++i)
            {
                dists[i] = task.poses[i];
                accum += dists[i] * dists[i];
            }   
            return accum;
        }

        void evaluate_interpreter_task(const interpreter_control_task& task, nos::ostream& os) 
        {
            int64_t steps[NMAX_AXES];
            double dists[NMAX_AXES];
            double Saccum = evaluate_steps_array(task, steps);
            double saccum = evaluate_dists_array(task, dists);

            // vecgain - scale coefficient with target direction
            double vecgain = sqrt(Saccum) / sqrt(saccum);
            double feed = task.feed * vecgain;
            double acc = task.acc * vecgain;

            // scale feed and acc by revolver freqs settings. 
            double reduced_feed = feed / revolver_frequency;
            double reduced_acc =
                acc / (revolver_frequency * revolver_frequency);

            // Eval fractions 
            double fractions[total_axes];
            evaluate_fractions(fractions, steps);

            auto &block = blocks->head_place();
            block.set_state(steps, total_axes, reduced_feed, reduced_acc,
                            fractions);
            block.blockno = blockno++;

            for (int i=0; i < total_axes; ++i)
                final_steps[i] += steps[i];

            os.println("Add new block:\r\n", block);
            system_lock();
            blocks->move_head_one();
            system_unlock();
        }

        std::vector<double> final_gained_position() 
        {
            std::vector<double> ret(total_axes);
            for (int i = 0; i < total_axes; ++i) 
                ret[i] = final_steps[i] / gains[i];
            return ret;   
        }

        void command_incremental_move(const nos::argv& argv, nos::ostream& os)
        {
            if(check_correctness(os)) return;
            auto task = g1_parse_task(argv);
            if (!task.isok) return;
            evaluate_interpreter_task(task, os);
        }

        void command_absolute_move(const nos::argv& argv, nos::ostream& os)
        {
            if(check_correctness(os)) return;
            auto task = g1_parse_task(argv);
            if (!task.isok) return;
            auto curpos = final_gained_position();
            for (int i = 0; i < total_axes; ++i) 
            {
                task.poses[i] = task.poses[i] - curpos[i];       
            }
            evaluate_interpreter_task(task, os);
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

            if (argv[0] == "gprotection") 
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
            for (int i = 0; i < total_axes; ++i)
                revolver->current_steps(vect.data());
            return vect;
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
