#ifndef RALGO_CNC_INTERPRETER_H
#define RALGO_CNC_INTERPRETER_H

#include <igris/container/array_view.h>
#include <igris/container/ring.h>
#include <igris/datastruct/argvc.h>
#include <igris/sync/syslock.h>
#include <igris/util/numconvert.h>

#include <ralgo/cnc/defs.h>
#include <ralgo/cnc/planblock.h>
#include <ralgo/log.h>

#include <string>

namespace cnc
{
    struct interpreter_control_task
    {
        double poses[NMAX_AXES];
        double feed;
        double acc;

        void parse(char **argv, int argc)
        {
            memset(poses, 0, sizeof(poses));

            for (int i = 0; i < argc; ++i)
            {
                char symb = argv[i][0];
                double val = atof(&argv[i][1]);

                switch (symb)
                {
                case 'F':
                    feed = val;
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
        bool info_mode = false;

        int total_axes = 3;
        double final_positions[NMAX_AXES];
        igris::ring<planner_block> *blocks;
        double revolver_frequency = 0;
        double gains[NMAX_AXES];

        int blockno = 0;

        double saved_acc = 1;
        double saved_feed = 1;

    public:
        interpreter(igris::ring<planner_block> *blocks) : blocks(blocks)
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

        void command_G1(int argc, char **argv, char *, int)
        {
            interpreter_control_task task;
            memset(&task, 0, sizeof(task));

            task.feed = saved_feed;
            task.acc = saved_acc;
            task.parse(argv, argc);

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

            if (info_mode)
            {
                ralgo::info("interpreter: add new block");
            }

            system_lock();
            blocks->move_head_one();
            system_unlock();
        }

        void command_M204(int argc, char **argv, char *ans, int ansmax)
        {
            if (argc == 0)
            {
                snprintf(ans, ansmax, "%f", saved_acc);
                return;
            }

            saved_acc = atof64(argv[0], nullptr);
        }

        void g_command(int argc, char **argv, char *ans, int ansmax)
        {
            int cmd = atoi(&argv[0][1]);
            switch (cmd)
            {
            case 1:
                command_G1(argc - 1, argv + 1, ans, ansmax);
                break;
            default:
                snprintf(ans, ansmax, "Unresolved G command");
            }
        }

        void m_command(int argc, char **argv, char *ans, int ansmax)
        {
            int cmd = atoi(&argv[0][1]);
            switch (cmd)
            {
            case 204:
                command_M204(argc - 1, argv + 1, ans, ansmax);
                break;
            default:
                snprintf(ans, ansmax, "Unresolved M command");
            }
        }

        int command(int argc, char **argv, char *ans, int ansmax)
        {
            assert(revolver_frequency != 0);

            if (argc == 0)
                return 0;

            char cmdsymb = argv[0][0];

            switch (cmdsymb)
            {
            case 'G':
            {
                g_command(argc, argv, ans, ansmax);
                break;
            }

            case 'M':
            {
                m_command(argc, argv, ans, ansmax);
                break;
            }
            }

            return 0;
        }

        void newline(const char *line, size_t size)
        {
            char buf[48];
            char ans[48];
            memcpy(buf, line, size);
            buf[size] = 0;

            char *argv[10];
            int argc = argvc_internal_split(buf, argv, size);

            command(argc, argv, ans, 48);
        }

        void newline(const std::string &line)
        {
            newline(line.data(), line.size());
        }

        void set_revolver_frequency(double freq) { revolver_frequency = freq; }

        void set_axes_count(int total) { total_axes = total; }
    };
}

#endif
