#ifndef RALGO_HEIMER_AXIS_DEVICE_H
#define RALGO_HEIMER_AXIS_DEVICE_H

#include <igris/util/bug.h>
#include <igris/math.h>
#include <igris/dtrace.h>

#include <ralgo/info.h>

namespace ralgo
{
	namespace heimer
	{
		template <class Position, class Speed>
		class axis_device : public ralgo::heimer::device
		{
			using parent = ralgo::heimer::device;
			using Time = int64_t;

		protected:
			bool _limited = false;
			Position _forw = 0;
			Position _back = 0;

			Speed _speed_protector = 0;
			int32_t _accdcc_protector = 0;

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
				if (!take_control())
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
				DTRACE();

				if (!take_control())
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
			virtual Position current_position() { BUG(); }

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
			virtual void set_speed(Speed spd) { _speed = protect_speed(spd); }
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
				_acc_val = protect_accdcc(acc);
				_dcc_val = protect_accdcc(dcc);
			}
			//Time setted_accdcc() { return _accdcc; }
			Time protect_accdcc(Time accdcc)
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
				parent::take_control_force();
				stop_impl(); 
			}

			virtual void stop_impl() = 0;

		};
	}
}

#endif