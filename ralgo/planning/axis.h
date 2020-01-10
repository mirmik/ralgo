#ifndef RALGO_PLANNING_AXIS_H
#define RALGO_PLANNING_AXIS_H

#include <ralgo/planning/traj1d.h>
#include <ralgo/planning/disctime.h>
#include <ralgo/planning/axis_interface.h>

#include <igris/event/delegate.h>
#include <igris/math.h>

#define NODTRACE 0 
#include <igris/dtrace.h>

//#include <rabbit/interval.h>

#include <limits>

/*
	Axis controller реализует логику траекторного расчета фаз,
	в единицах IntPos. Используется для реализации объектов из папки 
	planning/coordctr.
*/

namespace ralgo
{
	template <class ExtPos, class IntPos=int64_t, class Speed=float, class Time=int64_t>
	class axis_controller 
			: 
				public axis_interface<ExtPos, IntPos, Speed, Time>,
				public ralgo::named_buffer<16>
	{
		ralgo::traj1d_line<IntPos, Speed> line_traj;
		ralgo::traj1d<IntPos, Speed> * current_trajectory = nullptr;

		double gain = 1;
		bool reverse = false;
		float poskoeff = 0.001;

		// Устанавливаются в процессе обработки.
		IntPos tgtpos;
		Speed tgtspd; 

		// Скорость с учетом компенсации позиции.
		// Используется в режиме self-driving.
		Speed compspd; 

		//IntPos current_position = 0;
		//IntPos target_position = 0;



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
			dprln(0);
			pos = pos;

			dprln(1);
			auto time = ralgo::discrete_time();
		
			dprln(2);
			line_traj.reset(pos, time, pos, time + 1);
			dprln(3);
			line_traj.spddeform.nullify();
		
			dprln(4);
			current_trajectory = & line_traj;
		}

		IntPos planned_position(int64_t time) 
		{
			IntPos pos;
			Speed spd;

			attime(time, pos, spd);

			return pos; 
		}

		IntPos planned_position() 
		{
			return current_position(ralgo::discrete_time());
		}

		// Driver must implement that
		// It can be real or virtual position
		// for different axis`s types
		virtual IntPos current_position() = 0;

		int absmove_internal_unsafe(IntPos tgtpos) override 
		{
			auto spd= this->internal_speed();
			DTRACE_ARGS_2(tgtpos, spd);
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
			DTRACE();
			//TODO
			hardstop();
			return 0;
		}

		int hardstop() override
		{
			DTRACE();
			auto curtim = ralgo::discrete_time();
			IntPos curpos;
			Speed curspd;

			attime(curtim, curpos, curspd);			

			_absmove_tstamp(curpos, curtim, curpos, curtim);
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

		// Установить постоянную времени компенсации ошибки позиционирования.
		void set_position_compensator_filter_timeconst(float T)
		{
			poskoeff = 1 / (T * ralgo::discrete_time_frequency());
		}

// Self Driving:
		// Вычислить желаемую скорость по текущей фазе
		Speed eval_compensated_speed(float tgtpos, float tgtspd)
		{
			// Счетчик меняется в прерывании, так что
			// снимаем локальную копию.
			auto current = current_position();

			// Ошибка по установленному значению.
			auto diff = tgtpos - current;

			// Скорость вычисляется как
			// сумма уставной скорости на
			auto evalspeed = tgtspd  + poskoeff * diff;
			return evalspeed;
		}

		// Вычислить текущую фазу.
		void update_phase() 
		{
			// Установить текущие целевые параметры.
			attime(ralgo::discrete_time(), tgtpos, tgtspd);
			compspd = eval_compensated_speed(tgtpos, tgtspd);
		}

		Speed compensated_speed() 
		{
			return compspd; 
		}

		ssize_t print_to(nos::ostream& os) const
		{
			return nos::fprint_to(os, "axis_controller");
		}
	};
}

#endif