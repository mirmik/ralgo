#ifndef RALGO_HEIMER_INTERPOLATION_GROUP_H
#define RALGO_HEIMER_INTERPOLATION_GROUP_H

#include <ralgo/heimer/control.h>
#include <linalg/linalg.h>

namespace heimer
{
	template<class Position, class Speed>
	class linintctr_basic :
		public control_node
	{
	public:
		linintctr_basic(const char* name) :
			control_node(name)
		{}

		virtual int incmove(Position * mov) = 0;
		virtual int absmove(Position * pos) = 0;
		//virtual int stop() = 0;

		virtual int set_speed(Speed spd) = 0;
		virtual int set_accdcc(Speed acc, Speed dcc) = 0;
		virtual int dim() = 0;

		int set_accdcc(Speed acc) { return set_accdcc(acc,acc); }

		virtual Speed speed() = 0;
		virtual Speed acceleration() = 0;
		virtual Speed deceleration() = 0;

		virtual void set_gains(igris::array_view<float> arr) = 0;

		//virtual void debug_print_traj() = 0;
		//virtual void debug_print_state() = 0;

		int command(int argc, char** argv)
		{
			float fltargs[dim()];

			if (strcmp(argv[0], "mov") == 0)
			{
				if (argc != dim() + 1) 
				{
					nos::println("wrong args count");
					return -1;
				}

				for (int i = 0; i < dim(); ++i)
				{
					fltargs[i] = atof32(argv[1 + i], nullptr);
				}

				return absmove(fltargs);
			}

			else if (strcmp(argv[0], "setgain") == 0)
			{
				if (argc != dim() + 1) 
				{
					nos::println("wrong args count");
					return -1;
				}

				for (int i = 0; i < dim(); ++i)
				{
					fltargs[i] = atof32(argv[1 + i], nullptr);
				}

				set_gains({fltargs, (size_t)dim()});
				return 0;
			}

			else if (strcmp(argv[0], "setzone") == 0)
			{
				set_zone_command(argv[1]);
			}

			else if (strcmp(argv[0], "feed") == 0)
			{
				print_info();
			}

			else
			{
				nos::println("warn: unresolved command");
			}

			return 0;
		}

		//format a,b:c,d:e,g
		void set_zone_command(const char* cmd)
		{
			linalg::vec<Position, 2> pnts[8];

			const char* ptr = cmd;

			int i = 0;

			while (*ptr)
			{
				pnts[i].x = atof32(ptr, (char**)&ptr);
				ptr++; // skip comma

				pnts[i].y = atof32(ptr, (char**)&ptr);
				ptr++; // skip semicolon

				i++;
			}

			set_zone_protection(pnts);
		}

		virtual void set_zone_protection(
			igris::array_view<linalg::vec<Position, 2>> arr) = 0;

		// STOP
		virtual int hardstop() = 0;
		virtual int stop_impl() = 0;

		virtual bool can_operate() = 0;
		virtual void print_info() = 0;

		void stop()
		{
			stop_impl();
		}
	};
}


#endif