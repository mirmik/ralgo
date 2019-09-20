#ifndef RALGO_PLANNING_AXIS_H
#define RALGO_PLANNING_AXIS_H

#include <ralgo/planning/traj1d.h>
#include <ralgo/planning/disctime.h>

#include <igris/event/delegate.h>
#include <igris/math.h>

namespace ralgo
{
	enum stop_pattern
	{
		immediate,
		smooth
	};
	
	enum axis_operation_status 
	{
		stoped,
		moved,
		micromove
	};

	/**
		Интерфейс управления осью.
		может предствавлять локальные и удаленные интерфейсы.
	*/
	// TODO должен быть нешаблонным.
	/*class axis
	{
	public:
		virtual void incmove_tstamp(P pulses, int64_t tstamp);
		virtual void incmove(P pulses, V speed);
		virtual void absmove_tstamp(P pulses, int64_t tstamp);
		virtual void absmove(P pulses, V speed);

		virtual void stop(ralgo::stop_pattern stpcode);
	}*/

	template <class P, class V = float>
	class axis_controller
	{
		ralgo::traj1d_line<P, V> line_traj;
		ralgo::traj1d<P, V> * current_trajectory = nullptr;

		igris::delegate<void> trajectory_finish_signal;

		P backward_limit = 0;
		P forward_limit = 0;

		axis_controller * mirror = nullptr;
		P mirror_reference = 0;

		axis_operation_status opstat;

	public:
		void enable_mirror_mode(axis_controller * mirror, P reference) 
		{
			this->mirror = mirror;
			this->mirror_reference = reference;
		}

		void set_limits(P back, P forw) 
		{
			backward_limit = back;
			forward_limit = forw;
		}

		axis_operation_status status() 
		{
			if (current_trajectory->is_finished(ralgo::discrete_time())) 
			{
				return axis_operation_status::stoped;
			}

			else 
			{
				return axis_operation_status::moved;
			}
		} 

		void incmove_tstamp(P incpos, int64_t tstamp)
		{
		}

		void set_current_position(P pos) 
		{
			//dprln("set_current_position(P pos)");
			//PRINT(pos);

			//exit(0);

			auto time = ralgo::discrete_time();
		
			line_traj.reset(pos, time, pos, time + 1);
			line_traj.spddeform.nullify();
		
			current_trajectory = & line_traj;
		}

		P current_position(int64_t time) 
		{
			P pos;
			V spd;

			attime(time, pos, spd);

			return pos; 
		}

		P current_position() 
		{
			return current_position(ralgo::discrete_time());
		}

		/*void incmove(P pulses, V speed)
		{

		}*/

		void absmove_tstamp(
			P tgtpos, int64_t tgttim)
		{
			auto curtim = ralgo::discrete_time();
			P curpos;
			V curspd;

			attime(curtim, curpos, curspd);			

			absmove_tstamp(curpos, curtim, tgtpos, tgttim);
		}

		void absmove_tstamp(
			P curpos, int64_t curtim, P tgtpos, int64_t tgttim)
		{
			tgtpos = igris::clamp(tgtpos, backward_limit, forward_limit);

			line_traj.reset(curpos, curtim, tgtpos, tgttim);

			line_traj.spddeform.reset(0.3, 0.3);
			//line_traj.set_standart_accdcc_patern(
			    //options.acctime,
			    //options.dcctime,
			    //options.nominal_speed
			//);

			current_trajectory = &line_traj;
		}

		/*void absmove(P pulses, V speed)
		{

		}*/

		void stop(ralgo::stop_pattern stpcode)
		{

		}

	public:
		void attime(int64_t time, P& pos, V& spd)
		{
			if (current_trajectory)
			{
				int sts = current_trajectory->attime(time, pos, spd);
				
				if (sts) 
				{
					trajectory_finish_signal.emit();
				}
			}

			else
			{
				pos = 0;
				spd = 0;
			}

		}

		float control_position_unit() 
		{
			P pos;
			V spd;

			attime(ralgo::discrete_time(), pos, spd);

			return pos;
		}
	};
}

#endif