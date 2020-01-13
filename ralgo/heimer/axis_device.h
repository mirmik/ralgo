#ifndef RALGO_HEIMER_AXIS_DEVICE_H
#define RALGO_HEIMER_AXIS_DEVICE_H

namespace ralgo
{
	namespace heimer
	{
		template <class Position>
		class axis_device : public ralgo::heimer::device
		{
			bool limited = false;
			Position forwlim = 0;
			Position backlim = 0;

			Speed _speed_protector = 0;
			int32_t _accdcc_protector = 0;

			Speed _speed = 0;
			int32_t _accdcc = 0;

			Position offset = 0;
			float gain = 1;

		public:
			// JOG
			virtual int jog(int direction) { BUG(); }

			// INCMODE
			virtual int incmove(Position dist) 
			{
				if (take_control()) 
					return false;

				Position curpos = current_position();

				if (limited)
					tgtpos = clamp(curpos + dist, _back, _forw);

				Position ndist = tgtpos - curpos;

				incmove_unsafe(ndist);
			}

			virtual int absmove(Position tgtpos) 
			{
				if (take_control()) 
					return false;

				if (limited)
					tgtpos = clamp(tgtpos, _back, _forw);

				return absmove_unsafe(tgtpos);	
			}
			
			// POSITION
			virtual Position current_position() { BUG(); }
			
			// STOP
			virtual int stop() { BUG(); }
			virtual int hardstop() { BUG(); }

			// HOME_POSITION
			virtual int set_home_position() { BUG(); }

			// LIMITS
			bool position_in_limits(Position pos) { return pos > _back && pos < _forw; }
			void set_limits(Position back, Position forw) { _back = back; _forw = forw; limited = true; }
			void set_backward_limit(Position back) { _back = back; limited = true; }
			void set_forward_limit (Position forw) { _forw = forw; limited = true; }
			void disable_limits() { limited = false; }
			
			Position backward_limit() { return _back; }
			Position forward_limit() { return _forw; }
			
			// REVERSE
			void set_reverse(bool en) { _reverse = en; }
			bool reverse() { return _reverse; }

			// SPEED
			virtual void set_speed(Speed spd) { DTRACE_ARGS(spd); _speed = protect_speed(spd); }
			Speed setted_speed() { return _speed; }
			Speed protect_speed(Speed spd) { if (spd > _speed_protector) return _speed_protector; else return spd; }

			// ACCDCC
			virtual void set_accdcc(Time accdcc) { _accdcc = protect_accdcc(accdcc); }
			Time setted_accdcc() { return _accdcc; }
			Time protect_accdcc(Time accdcc) { if (accdcc < _accdcc_protector) return _accdcc_protector; else return accdcc; }

		};
	}
}

#endif