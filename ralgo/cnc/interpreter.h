#ifndef RALGO_CNC_INTERPRETER_H
#define RALGO_CNC_INTERPRETER_H

#include <igris/container/ring.h>
#include <igris/container/array_view.h>
#include <igris/datastruct/argvc.h>

#include <ralgo/cnc/planblock.h>
#include <ralgo/cnc/defs.h>

#include <nos/print.h>
#include <nos/fprint.h>

namespace cnc
{
	class token
	{

	};

	class interpreter
	{
		static constexpr char alphabet[9] =
		{ 'X', 'Y', 'Z', 'A', 'B', 'C', 'S', 'T', 'V' };

		struct control_task
		{
			double poses[NMAX_AXES];
			double feed;

			void parse(char ** argv, int argc)
			{
				memset(poses, 0, sizeof(poses));

				for (int i = 0; i < argc; ++i)
				{
					char symb = argv[i][0];
					double val = atof(&argv[i][1]);

					switch (symb)
					{
						case 'F' : feed = val; continue;
						case 'X' : poses[0] = val; continue;
						case 'Y' : poses[1] = val; continue;
						case 'Z' : poses[2] = val; continue;
						case 'A' : poses[3] = val; continue;
						case 'B' : poses[4] = val; continue;
						case 'C' : poses[5] = val; continue;
						case 'S' : poses[6] = val; continue;
						case 'T' : poses[7] = val; continue;
						case 'V' : poses[8] = val; continue;
					}
				}
			}

			template<class O>
			int print_to(O& os) const
			{
				return nos::fprint_to(os, "{},{}", igris::array_view(poses, NMAX_AXES), feed);
			}
		};

	public:
		int total_axes = 3;
		double final_positions[NMAX_AXES];
		igris::ring<planner_block> * blocks;
		double revolver_frequency;
		double gains[NMAX_AXES];

		int blockno = 0;
		double task_acc = 1;
	public:

		interpreter(igris::ring<planner_block> * blocks) : blocks(blocks)
		{
			memset(final_positions, 0, sizeof(final_positions));
			for (auto & gain : gains)
				gain = 1;
		}

		void evaluate_multipliers(double * multipliers, int64_t * steps)
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

		void command_g1(int argc, char ** argv)
		{
			control_task task;
			task.parse(argv, argc);

			auto & block = blocks->head_place();

			assert(task.feed != 0);

			PRINT(task.feed);

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
			double acc = task_acc * vecgain;

			PRINT(feed);
			PRINT(vecgain);
			PRINT(steps[0]);
			PRINT(dists[0]);

			nos::print_list(igris::array_view(steps, total_axes));
			PRINT(total_axes);
			PRINT(acc);

			double reduced_feed = feed / revolver_frequency;
			double reduced_acc = acc / (revolver_frequency * revolver_frequency);

			PRINT(reduced_feed);
			PRINT(reduced_acc);

			double multipliers[total_axes];
			evaluate_multipliers(multipliers, steps);

			block.set_state(steps, total_axes,
			                reduced_feed,
			                reduced_acc,
			                multipliers);
			block.blockno = blockno++;
			blocks->move_head_one();
		}

		void g_command(int argc, char ** argv)
		{
			nos::fprintln("g_command: {}", igris::array_view{argv, argc});

			int cmd = atoi(&argv[0][1]);
			switch (cmd)
			{
				case 1: command_g1(argc - 1, argv + 1); break;
			}
		}

		void m_command(int argc, char ** argv)
		{
			(void) argc;
			(void) argv;
		}

		void newline(const char * line, size_t size)
		{
			char buf[48];
			memcpy(buf, line, size);
			buf[size] = 0;

			nos::println("Line: ", buf);

			char * argv[10];
			int argc = argvc_internal_split(buf, argv, size);

			if (argc == 0)
				return;

			char cmdsymb = argv[0][0];

			switch (cmdsymb)
			{
				case 'G':
				{
					g_command(argc, argv);
					break;
				}

				case 'M':
				{
					m_command(argc, argv);
					break;
				}
			}
		}

		void newline(const std::string & line)
		{
			newline(line.data(), line.size());
		}
	};
}

#endif