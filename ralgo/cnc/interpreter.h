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
			float poses[NMAX_AXES];
			float feed;

			void parse(char ** argv, int argc)
			{
				memset(poses, 0, sizeof(poses));

				for (int i = 0; i < argc; ++i)
				{
					char symb = argv[i][0];
					float val = atof(&argv[i][1]);

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
		float last_positions[NMAX_AXES];
		igris::ring<planner_block> * blocks;

		interpreter(igris::ring<planner_block> * blocks) : blocks(blocks)
		{
			memset(last_positions, 0, sizeof(last_positions));
		}

		void command_g1(int argc, char ** argv)
		{
			control_task task;
			task.parse(argv, argc);
		
			auto & block = blocks->head_place();

			float acc = 1;
			assert(task.feed != 0);
			assert(acc > 0);

			int32_t steps[NMAX_AXES];
			for (int i = 0; i < total_axes; ++i) 
			{
				steps[i] = task.poses[i] * 1000;
				last_positions[i] += task.poses[i];
			}

			nos::print_list(igris::array_view(steps,total_axes));
			PRINT(total_axes);
			PRINT(task.feed);
			PRINT(acc);

			block.set_state(steps, total_axes, 
				task.feed, 
				acc);			
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