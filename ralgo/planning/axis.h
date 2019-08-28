#ifndef RALGO_PLANNING_AXIS_H
#define RALGO_PLANNING_AXIS_H

#include <ralgo/planning/traj1d.h>

namespace ralgo
{
	enum stop_pattern
	{
		immediate,
		smooth
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
	class position_controlled_axis
	{
		ralgo::traj1d_line<P, V> line_traj;
		ralgo::traj1d<P, V> * current_trajectory = nullptr;

	public:
		void incmove_tstamp(P incpos, int64_t tstamp)
		{
		}

		/*void incmove(P pulses, V speed)
		{

		}*/

		void absmove_tstamp(
			int64_t tgttim, P tgtpos, V tgtspd = 0)
		{
			auto curtim = ralgo::discrete_time();
			P curpos;
			V curspd;

			attime(curtim, curpos, curspd);			

			absmove_tstamp(curtim, tgttim, curpos, tgtpos, curspd, tgtspd);
		}

		void absmove_tstamp(
			int64_t curtim, int64_t tgttim, 
			P curpos, P tgtpos, 
			V curspd = 0, V tgtspd = 0)
		{
			line_traj.reset(curpos, curtim, tgtpos, tgttim);

			//line_traj.set_standart_accdcc_patern(
			//    options.acctime,
			//    options.dcctime,
			//    options.nominal_speed
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
	};
}

#endif