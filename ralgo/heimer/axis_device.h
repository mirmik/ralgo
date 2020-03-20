#ifndef RALGO_HEIMER_AXIS_DEVICE_H
#define RALGO_HEIMER_AXIS_DEVICE_H

#include <igris/util/numconvert.h>
#include <igris/util/bug.h>
#include <igris/math.h>

#include <nos/fprint.h>
#include <ralgo/info.h>

namespace ralgo
{
	namespace heimer
	{
		template <class Position, class Speed>
		class axis_device
		{
			using Time = int64_t;

		protected:
			bool _limited = false;
			Position _forw = 0;
			Position _back = 0;

			Speed _speed_protector = 0;
			Speed _accdcc_protector = 0;

			Speed _speed = 0;
			float _acc_val = 0;
			float _dcc_val = 0;

			bool _reverse = false;
			Position _offset = 0;
			float _gain = 1;

		public:
			// JOG
			virtual int jog(int direction) { BUG(); }

			// INCMODE
			virtual int incmove(Position dist)
			{
				dist = dist * _gain;

				if (try_operation_begin(0))
				{
					ralgo::warning("axis_device take_control fault");
					return -1;
				}

				Position curpos = current_position();
				Position tgtpos = curpos + dist;

				if (_limited)
					tgtpos = igris::clamp(tgtpos, _back, _forw);

				Position ndist = tgtpos - curpos;

				return incmove_unsafe(ndist);
			}
			virtual int incmove_unsafe(Position dist) = 0;

			virtual int absmove(Position tgtpos)
			{
				tgtpos = tgtpos * _gain;

				if (try_operation_begin(0))
				{
					ralgo::warning("axis_device take_control fault");
					return -1;
				}

				if (_limited)
					tgtpos = igris::clamp(tgtpos, _back, _forw);

				return absmove_unsafe(tgtpos);
			}
			virtual int absmove_unsafe(Position dist) = 0;

			// POSITION
			virtual Position current_position() = 0;
			virtual Speed current_speed() = 0;
			//virtual void set_current_position(Position pos) = 0;

			// HOME_POSITION
			virtual int set_home_position() { BUG(); }

			virtual void direct_control(Position pos, Speed spd) = 0;

			// LIMITS
			bool position_in_limits(Position pos) { return pos > _back && pos < _forw; }
			void set_limits(Position back, Position forw) { _back = back; _forw = forw; _limited = true; }
			void set_backward_limit(Position back) { _back = back; _limited = true; }
			void set_forward_limit (Position forw) { _forw = forw; _limited = true; }
			void disable_limits() { _limited = false; }

			Position backward_limit() { return _back; }
			Position forward_limit() { return _forw; }

			// REVERSE
			void set_reverse(bool en) { _reverse = en; }
			bool reverse() { return _reverse; }

			// SPEED
			virtual void set_speed(Speed spd)
			{
				spd = spd * _gain;
				_speed = protect_speed(spd);
			}

			void set_gain(float gain)
			{
				_gain = gain;
			}
			float gain() { return _gain; }

			Speed setted_speed() { return _speed; }
			Speed protect_speed(Speed spd)
			{
				if (_speed_protector != 0 && spd > _speed_protector)
					return _speed_protector;
				else
					return spd;
			}

			// ACCDCC
			virtual void set_accdcc_value(float acc, float dcc)
			{
				acc = acc * _gain;
				dcc = dcc * _gain;
				_acc_val = protect_accdcc(acc);
				_dcc_val = protect_accdcc(dcc);
			}
			//Time setted_accdcc() { return _accdcc; }
			Speed protect_accdcc(Speed accdcc)
			{
				if (accdcc < _accdcc_protector)
					return _accdcc_protector;
				else
					return accdcc;
			}

			Speed acceleration_value()
			{
				return _acc_val;
			}

			Speed deceleration_value()
			{
				return _dcc_val;
			}

			// STOP
			virtual int hardstop() = 0;

			void stop()
			{
				int sts = try_operation_begin(1);

				if (sts == 0)
					stop_impl();
			}

			virtual void stop_impl() = 0;

			virtual int try_operation_begin(int priority) = 0;
			virtual void operation_finish(int priority) = 0;

			int command(int argc, char** argv)
			{
				float fltarg;

				if (strcmp(argv[0], "mov") == 0)
				{
					fltarg = atof32(argv[1], nullptr);
					return absmove(fltarg);
				}

				else if (strcmp(argv[0], "incmov") == 0)
				{
					fltarg = atof32(argv[1], nullptr);
					return incmove(fltarg);
				}

				else if (strcmp(argv[0], "setspd") == 0)
				{
					fltarg = atof32(argv[1], nullptr);
					set_speed(fltarg);
					return 0;
				}

				else if (strcmp(argv[0], "setacc") == 0)
				{
					fltarg = atof32(argv[1], nullptr);
					set_accdcc_value(fltarg, fltarg);
					return 0;
				}

				else if (strcmp(argv[0], "feed") == 0)
				{
					print_feed();
					return 0;
				}

				else
				{
					nos::println("warn: unresolved command");
				}

				return 0;
			}
			virtual void print_feed() = 0;
		};

	}
}

#endif