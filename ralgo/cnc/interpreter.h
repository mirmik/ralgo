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

#include <ralgo/cnc/planner.h>
#include <ralgo/cnc/revolver.h>
#include <nos/shell/executor.h>
#include <nos/io/string_writer.h>

#include <string>

namespace cnc
{
    struct interpreter_control_task
    {
        double poses[NMAX_AXES];
        double feed;
        double acc;

        void parse(const nos::argv& argv)
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
                }
            }
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
        double final_positions[NMAX_AXES];
        double revolver_frequency = 0;
        double gains[NMAX_AXES];
        int blockno = 0;
        double saved_acc = 0;
        double saved_feed = 0;

    public:
        interpreter(igris::ring<planner_block> *blocks, cnc::planner *planner,
                    cnc::revolver *revolver)
            : blocks(blocks), planner(planner), revolver(revolver)
        {
            memset(final_positions, 0, sizeof(final_positions));
            for (auto &gain : gains)
                gain = 1;
        }

        void evaluate_multipliers(double *multipliers, int64_t *steps)
        {
            int64_t accum = 0;
            for (int i = 0; i < total_axes; ++i)
            {
                accum += steps[i] * steps[i];
            }
            double dist = sqrt(accum);

            for (int i = 0; i < total_axes; ++i)
            {
                multipliers[i] = (double)steps[i] / dist;
            }
        }

        int check_correctness() 
        {
            if (saved_feed == 0) { 
                ralgo::warn("saved_feed is null"); return 1; }
            if (saved_acc == 0) { 
                ralgo::warn("saved_acc is null"); return 1; }
            if (revolver_frequency == 0) { 
                ralgo::warn("revolver_frequency is null"); return 1; }
            return 0;
        }

        void command_G1(const nos::argv& argv, nos::ostream& os)
        {
            if(check_correctness()) 
            {
                return;
            }

            interpreter_control_task task;
            memset(&task, 0, sizeof(task));

            task.feed = saved_feed;
            task.acc = saved_acc;
            task.parse(argv);

            if (task.feed == 0 || task.acc == 0)
            {
                ralgo::warn("nullvelocity block. ignore.");
                return;
            }

            auto &block = blocks->head_place();

            int64_t steps[NMAX_AXES];
            double dists[NMAX_AXES];
            double Saccum = 0;
            double saccum = 0;
            for (int i = 0; i < total_axes; ++i)
            {
                steps[i] = task.poses[i] * gains[i];
                dists[i] = task.poses[i];
                final_positions[i] += steps[i];

                saccum += dists[i] * dists[i];
                Saccum += steps[i] * steps[i];
            }

            double vecgain = sqrt(Saccum) / sqrt(saccum);
            double feed = task.feed * vecgain;
            double acc = task.acc * vecgain;

            double reduced_feed = feed / revolver_frequency;
            double reduced_acc =
                acc / (revolver_frequency * revolver_frequency);

            double multipliers[total_axes];
            evaluate_multipliers(multipliers, steps);

            block.set_state(steps, total_axes, reduced_feed, reduced_acc,
                            multipliers);
            block.blockno = blockno++;

            os.println("Add new block:\r\n", block);

            if (info_mode)
            {
                ralgo::info("interpreter: add new block");
            }

            system_lock();
            blocks->move_head_one();
            system_unlock();
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
            blocks->clear();
            plan_stop_task();
            system_unlock();
        }

        void plan_stop_task()
        {
            float velocity[NMAX_AXES];
            revolver->current_velocity(velocity);

            // auto &block = blocks->head_place();
            // planer
            // block.set_stop_pattern(steps, total_axes, reduced_feed,
            // reduced_acc,
            //        multipliers);
            blocks->move_head_one();
        }

        void g_command(const nos::argv& argv, nos::ostream& os)
        {
            int cmd = atoi(&argv[0].data()[1]);
            switch (cmd)
            {
            case 1:
                command_G1(argv.without(1), os);
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

        int command(const nos::argv& argv, nos::ostream& os)
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

        int command_drop_first(const nos::argv& argv, nos::ostream& os)
        {
            return command(argv.without(1), os);
        }

        std::vector<int64_t> command_get_current_steps()
        {
            std::vector<int64_t> vect(total_axes);
            for (int i = 0; i < total_axes; ++i)
                revolver->current_steps(vect.data());
            return vect;
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
                return os.println(command_get_current_steps());
            }
            
            os.println("Wrong subcommand");
            return 0;
        }

        nos::executor executor = nos::executor({
            {"cnc", "commands", nos::make_delegate(&interpreter::command_drop_first, this)},
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
