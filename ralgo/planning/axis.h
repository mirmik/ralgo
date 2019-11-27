#ifndef RALGO_PLANNING_AXIS_H
#define RALGO_PLANNING_AXIS_H

#include <ralgo/planning/traj1d.h>
#include <ralgo/planning/disctime.h>
#include <ralgo/planning/axis_interface.h>

#include <igris/event/delegate.h>
#include <igris/math.h>

//#include <rabbit/interval.h>

#include <limits>

/*
	Axis controller реализует логику траекторного расчета фаз,
	в единицах IntPos. Используется для реализации объектов из папки 
	planning/coordctr.
*/

namespace ralgo
{
	template <class ExtPos, class IntPos, class Speed=float, class Time=int64_t>
	class axis_controller 
			: 
				public axis_interface<ExtPos, IntPos, Speed, Time>,
				public ralgo::named_buffer<16>
	{
		ralgo::traj1d_line<IntPos, Speed> line_traj;
		ralgo::traj1d<IntPos, Speed> * current_trajectory = nullptr;

		double gain = 1;
		bool reverse = false;

		//rabbit::interval<IntPos> limits_interal {0,0};

	public:
		axis_operation_status status() 
		{
			if (this->is_controlled()) 
			{
				return axis_operation_status::CONTROLLED;
			}

			if (current_trajectory->is_finished(ralgo::discrete_time())) 
			{
				return axis_operation_status::STOPED;
			}

			else 
			{
				return axis_operation_status::MOVED;
			}
		} 

		//void incmove_tstamp(IntPos incpos, int64_t tstamp)
		//{
		//}

		void set_current_position(IntPos pos) 
		{
			//dprln("set_current_position(IntPos pos)");
			//PRINT(pos);

			//exit(0);
			pos = pos;

			auto time = ralgo::discrete_time();
		
			line_traj.reset(pos, time, pos, time + 1);
			line_traj.spddeform.nullify();
		
			current_trajectory = & line_traj;
		}

		IntPos current_position(int64_t time) 
		{
			IntPos pos;
			Speed spd;

			attime(time, pos, spd);

			return pos; 
		}

		IntPos current_position() 
		{
			return current_position(ralgo::discrete_time());
		}

/*		IntPos control_position(int64_t time) 
		{
			return current_position() / control_multiplier;
		}*/

/*		IntPos control_position() 
		{
			return current_position(ralgo::discrete_time()) / control_multiplier;
		}

		void set_control_position(IntPos pos) 
		{
			set_current_position(pos * control_multiplier);
		}*/

		/*void incmove(IntPos pulses, Speed speed)
		{

		}*/


		/*void absmove(
			IntPos tgtpos, int64_t tim)
		{
			tgtpos = tgtpos * control_multiplier;
			_absmove_tstamp(tgtpos, ralgo::discrete_time() + tim);
		}*/

		int absmove_internal_unsafe(IntPos tgtpos) override 
		{
			return _absmove_by_speed(tgtpos, this->internal_speed());
		}

		int _absmove_by_speed(
			IntPos tgtpos, float spd)
		{
			//TODO PROTECT VIRTDEV
			//if (!task_checker(*this,  tgtpos))
			//	return;

			auto cpos = current_position();
			auto mpos = fabs(tgtpos - cpos);
			spd = fabs(spd);
			_absmove_tstamp(
				tgtpos, 
				ralgo::discrete_time() 
					+ (mpos / spd) * ralgo::discrete_time_frequency());

			return 0;
		}

		void _absmove_tstamp(
			IntPos tgtpos, int64_t tgttim)
		{

			auto curtim = ralgo::discrete_time();
			IntPos curpos;
			Speed curspd;

			attime(curtim, curpos, curspd);			

			_absmove_tstamp(curpos, curtim, tgtpos, tgttim);
		}

		void _absmove_tstamp(
			IntPos curpos, int64_t curtim, IntPos tgtpos, int64_t tgttim)
		{
			//tgtpos = igris::clamp(tgtpos, backward_limit, forward_limit);

			line_traj.reset(curpos, curtim, tgtpos, tgttim);

			line_traj.spddeform.reset(0.3, 0.3);
			//line_traj.set_standart_accdcc_patern(
			    //options.acctime,
			    //options.dcctime,
			    //options.nominal_speed
			//);

			current_trajectory = &line_traj;
		}

		/*void absmove(IntPos pulses, Speed speed)
		{

		}*/

		int stop() override
		{
			//VIRTDEV

			IntPos curpos = current_position();
			Speed curspd = this->setted_speed();

			BUG();
			return 0;
		}

	public:
		void attime(int64_t time, IntPos& pos, Speed& spd)
		{
			if (current_trajectory)
			{
				int sts = current_trajectory->attime(time, pos, spd);
				
				if (sts) 
				{
					//trajectory_finish_signal.emit();
				}
			}

			else
			{
				pos = 0;
				spd = 0;
			}

		}

		/*float control_position_unit() 
		{
			IntPos pos;
			Speed spd;

			attime(ralgo::discrete_time(), pos, spd);

			return pos;
		}*/

		ssize_t print_to(nos::ostream& os) const
		{
			return nos::fprint_to(os, "axis_controller");
		}
	};
}

#endif