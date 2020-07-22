#ifndef HEIMER_AXISCTR_H
#define HEIMER_AXISCTR_H

#include <ralgo/log.h>
#include <ralgo/heimer/axis.h>
#include <ralgo/trajectory/traj1d.h>

#include <igris/event/delegate.h>
#include <igris/math.h>

namespace heimer
{
	extern struct dlist_head axisctr_list;

	template <class P, class V>
	class axisctr : public control_node
	{
	private:
		P offset = 0;

		V spd = 1;
		V acc = 1;
		V dcc = 1;

		V maxspd = 1;
		V maxacc = 1;
		V maxdcc = 1;

		float gain = 1;

		heimer::axis_node<P, V> * controlled;

		ralgo::traj1d<P, V> *    curtraj = &lintraj;
		ralgo::traj1d_line<P, V> lintraj;

		bool _limited = false;
		P _forw;
		P _back;

		bool operation_finished_flag = true;

	public:
		igris::delegate<void, void*> operation_finish_signal;

	public:
		auto * current_trajectory() { return curtraj; }
		auto * linear_trajectory() { return &lintraj; }

		constexpr
		axisctr(
		    const char * mnemo,
		    heimer::axis_node<P, V> * controlled
		) :
			control_node(mnemo),
			controlled(controlled)
		{}

		int jog(int direction);
		int incmove(P dist);
		int absmove(P pos);

		int incmove_unsafe(P dist);
		int absmove_unsafe(P pos);

		int stop();

		void set_gain(V gain) { this->gain = gain; };

		P feedback_position() { return controlled->feedpos; }
		V feedback_speed()    { return controlled->feedspd; }

		P target_position() { return controlled->ctrpos; }
		V target_speed()    { return controlled->ctrspd; }

		V setted_speed()        { return spd; }
		V setted_acceleration() { return acc; }
		V setted_deceleration() { return dcc; }

		void set_speed(V spd)        { this->spd = spd; }
		void set_acceleration(V acc) { this->acc = acc; }
		void set_deceleration(V dcc) { this->dcc = dcc; }
		void set_offset(P offset)    { this->offset = offset; }

		void set_accdcc(V acc, V dcc)
		{
			this->acc = acc;
			this->dcc = dcc;
		}

		void serve();
		bool can_operate()
		{
			return
			    is_active() &&
			    (!curtraj->is_finished(ralgo::discrete_time()));
		};

	private:
		int _absmove_unsafe(P pos, P tgt);
	};

	template <class P, class V>
	int axisctr<P, V>::incmove(P dist)
	{
		dist = dist * gain;

		if (flags & HEIM_IS_ACTIVE)
		{
			ralgo::warn("axisctr: not active");
			return -1;
		}

		P curpos = target_position();
		P tgtpos = curpos + dist;

		if (_limited)
			tgtpos = igris::clamp(tgtpos, _back, _forw);

		P ndist = tgtpos - curpos;

		return incmove_unsafe(ndist);
	}

	template <class P, class V>
	int axisctr<P, V>::absmove(P tgtpos)
	{
		tgtpos = tgtpos * gain;

		if (flags & HEIM_IS_ACTIVE)
		{
			ralgo::warn("axisctr: not active");
			return -1;
		}

		//P curpos = target_position();
		//P tgtpos = curpos + dist;

		if (_limited)
			tgtpos = igris::clamp(tgtpos, _back, _forw);

		//P ndist = tgtpos - curpos;

		return absmove_unsafe(tgtpos);
	}

	template <class P, class V>
	int axisctr<P, V>::_absmove_unsafe(P curpos, P tgtpos)
	{
		auto dist = tgtpos - curpos;
		int64_t curtim = ralgo::discrete_time();

		V dist_mul_freq = (V)fabs(dist) * ralgo::discrete_time_frequency();
		int64_t tgttim = curtim + (int64_t)(dist_mul_freq / spd);

		if (curtim >= tgttim)
		{
			operation_finished_flag = true;
			operation_finish_signal(this);
			lintraj.set_point_hold(curpos);
			curtraj = &lintraj;

			return 0;
		}

		lintraj.reset(curpos, curtim, tgtpos, tgttim);
		lintraj.set_speed_pattern(acc, dcc, spd);

		operation_finished_flag = false;
		curtraj = &lintraj;
		return 0;
	}

	template <class P, class V>
	int axisctr<P, V>::incmove_unsafe(P dist)
	{
		// TODO: Это должно работать с gain
		auto curpos = target_position();
		return _absmove_unsafe(curpos, curpos + dist);
	}

	template <class P, class V>
	int axisctr<P, V>::absmove_unsafe(P pos)
	{
		// TODO: Это должно работать с gain
		auto curpos = target_position();
		return _absmove_unsafe(curpos, pos);
	}

	template <class P, class V>
	void axisctr<P, V>::serve()
	{
		P ctrpos;
		V ctrspd;

		// Установить текущие целевые параметры.
		int sts = curtraj->attime(ralgo::discrete_time(), ctrpos, ctrspd);
		if (sts && !operation_finished_flag)
		{
			operation_finished_flag = true;
			operation_finish_signal(this);
			lintraj.set_point_hold(ctrpos);
			curtraj = &lintraj;
		}

		controlled->ctrpos = ctrpos;
		controlled->ctrspd = ctrspd;
	}

	template<class P, class V>
	int axisctr<P, V>::stop()
	{
		if (flags & HEIM_IS_ACTIVE)
		{
			ralgo::warn("axisctr: not active");
			return -1;
		}

		if (curtraj == nullptr)
			return 0;

		lintraj.set_stop_trajectory(
		    feedback_position(),
		    feedback_speed(),
		    dcc);

		curtraj = & lintraj;
		return 0;
	}
}

#endif