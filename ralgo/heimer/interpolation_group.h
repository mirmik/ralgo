#ifndef RALGO_HEIMER_INTERPOLATION_GROUP_H
#define RALGO_HEIMER_INTERPOLATION_GROUP_H

#include <ralgo/heimer/control.h>

namespace ralgo
{
	namespace heimer
	{
		template<class Position, class Speed>
		class interpolation_group
		{
			const char* _name;

		public:
			interpolation_group(const char* name) : _name(name) {}

			const char* name() { return _name; }

			virtual int incmove(Position * mov) = 0;
			virtual int absmove(Position * pos) = 0;
			//virtual int stop() = 0;

			virtual int set_speed(Speed spd) = 0;
			virtual int set_accdcc_value(float acc, float dcc) = 0;
			virtual int dim() = 0;

			virtual Speed speed() = 0;
			virtual Speed acceleration() = 0;
			virtual Speed deceleration() = 0;

			virtual void debug_print_traj() = 0;
			virtual void debug_print_state() = 0;

			virtual int try_operation_begin(int priority) = 0;


			int command(int argc, char** argv)
			{
				float fltargs[dim()];

				if (strcmp(argv[0], "mov") == 0)
				{
					//fltarg = atof32(argv[1], nullptr);
					for (int i = 0; i < dim(); ++i) 
					{
						fltargs[i] = atof32(argv[1+i], nullptr);
					}

					return absmove(fltargs);
				}

				else
				{
					nos::println("warn: unresolved command");
				}

				return 0;
			}

			// STOP
			virtual int hardstop() = 0;
			virtual int stop_impl() = 0;

			virtual bool can_operate() = 0;
			virtual int print_feed() = 0;

			void stop()
			{
				int sts = try_operation_begin(1);
				
				if (sts == 0)
					stop_impl();
			}
		};
	}
}

#endif