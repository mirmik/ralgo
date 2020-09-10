#ifndef HEIMER_AXISCTR_H
#define HEIMER_AXISCTR_H

#include <ralgo/log.h>
#include <ralgo/trajectory/traj1d.h>
#include <ralgo/heimer/axis.h>
#include <ralgo/heimer/protect.h>

#include <igris/event/delegate.h>
#include <igris/math.h>

namespace heimer
{
	extern struct dlist_head axisctr_list;

	template <class P, class V>
	class axisctr : public control_node
	{
	public:
		bool debug_mode = false;

	private:

		P offset = 0;

		V spd = 1;
		V acc = 1;
		V dcc = 1;

		//notused
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
		igris::delegate<void, axisctr*> operation_finish_signal;
		igris::delegate<void, axisctr*> operation_start_signal;

		igris::delegate<bool, P, P, char*> move_protector;

	public:
		auto * current_trajectory() { return curtraj; }
		auto * linear_trajectory() { return &lintraj; }

		bool in_operate() { return !operation_finished_flag; }

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

		void set_limits(P a, P b) 
		{
			_back = a;
			_forw = b;
			_limited = true;
		}

		int incmove_unsafe(P dist);
		int absmove_unsafe(P pos);

		int stop();
		void hardstop();

		void set_gain(V gain) { this->gain = gain; };

		P feedback_position() { return controlled->feedpos / gain; }
		V feedback_speed()    { return controlled->feedspd / gain; }

		P target_position() { return controlled->ctrpos / gain; }
		V target_speed()    { return controlled->ctrspd / gain; }

		V setted_speed()        { return spd / gain; }
		V setted_acceleration() { return acc / gain; }
		V setted_deceleration() { return dcc / gain; }

		void set_speed(V spd)        { this->spd = spd * gain; }
		void set_acceleration(V acc) { this->acc = acc * gain; }
		void set_deceleration(V dcc) { this->dcc = dcc * gain; }
		void set_offset(P offset)    { this->offset = offset; }

		void set_accdcc(V acc, V dcc)
		{
			this->acc = acc * gain;
			this->dcc = dcc * gain;
		}

		void serve_impl() override;
		bool can_operate()
		{
			return
			    is_active() &&
			    (!curtraj->is_finished(ralgo::discrete_time()));
		};

		int command(int argc, char** argv);

		control_node* iterate(control_node* it) override
		{
			if (it == NULL)
				return controlled;
			return NULL;
		}

		void print_info() override
		{
			int sts;
			P ctrpos = 0;
			V ctrspd = 0;

			nos::println(lintraj);

			if (curtraj)
				sts = curtraj->attime(ralgo::discrete_time(), ctrpos, ctrspd);

			nos::println(sts);
			nos::println("phasepos: ", ctrpos);
			nos::println("phasespd: ", ctrspd);
			nos::println("ctrpos: ", controlled->ctrpos);
			nos::println("ctrspd: ", controlled->ctrspd);
			nos::println("feedpos: ", controlled->feedpos);
			nos::println("limited: ", _limited);
			nos::println("flim: ", _forw);
			nos::println("blim: ", _back);
			nos::println("gain: ", gain);
			nos::println("spd: ", spd);
			nos::println("acc: ", acc);
			nos::println("dcc: ", dcc);
			nos::println("maxspd: ", maxspd);
			nos::println("maxacc: ", maxacc);
			nos::println("maxdcc: ", maxdcc);
		}

		bool on_interrupt(
			control_node * slave,
		    control_node * source,
		    interrupt_args * args) override
		{
			if (args->code() == HEIMER_INTERRUPT_TYPE_CONTROL_UPDATE)
			{
				stop();
			}

			else 
			{
				hardstop();
			}

			// локируем, так как это объект высшего уровня
			return true;
		}

	private:
		int _absmove_unsafe(P pos, P tgt);
	};

	template <class P, class V>
	int axisctr<P, V>::incmove(P dist)
	{
		dist = dist * gain;

		P curpos = controlled->ctrpos;
		P tgtpos = curpos + dist;

		if (_limited)
			tgtpos = igris::clamp(tgtpos, _back * gain, _forw * gain);

		P ndist = tgtpos - curpos;

		return incmove_unsafe(ndist);
	}

	template <class P, class V>
	int axisctr<P, V>::absmove(P tgtpos)
	{
		tgtpos = tgtpos * gain;

		if (_limited)
			tgtpos = igris::clamp(tgtpos, _back * gain, _forw * gain);

		return absmove_unsafe(tgtpos);
	}

	template <class P, class V>
	int axisctr<P, V>::_absmove_unsafe(P curpos, P tgtpos)
	{
		auto dist = tgtpos - curpos;
		int64_t curtim = ralgo::discrete_time();

		if (heimer::global_protection) 
		{
			ralgo::warn(mnemo(), ": cannot start: global protection is setted");
			return -1;
		}

		if (!is_active())
		{
			ralgo::warn(mnemo(), ": not active");
			return -1;
		}

		if (move_protector.armed()) 
		{
			char msg[64];
			if (move_protector(curpos, tgtpos, msg)) 
			{
				ralgo::warn(mnemo(), ": ", msg);
				return -1;
			}
		}

		if (dist == 0)
		{
			operation_finished_flag = true;
			operation_finish_signal(this);
			lintraj.set_point_hold(curpos);
			curtraj = &lintraj;

			return 0;
		}

		operation_finished_flag = false;

		ralgo::traj1d_nominal_speed_params<P, V> nm_params =
		{
			.stim = curtim,
			.spos = curpos,
			.fpos = tgtpos,
			.speed = spd,
			.acc = acc,
			.dcc = dcc,
		};

		lintraj.init_nominal_speed_mode(&nm_params);

		if (debug_mode)
			nos::println(lintraj);

		operation_finished_flag = false;
		operation_start_signal(this);
		curtraj = &lintraj;
		return 0;
	}

	template <class P, class V>
	int axisctr<P, V>::incmove_unsafe(P dist)
	{
		// TODO: Это должно работать с gain
		auto curpos = controlled->ctrpos;
		return _absmove_unsafe(curpos, curpos + dist);
	}

	template <class P, class V>
	int axisctr<P, V>::absmove_unsafe(P pos)
	{
		// TODO: Это должно работать с gain
		auto curpos = controlled->ctrpos;
		return _absmove_unsafe(curpos, pos);
	}

	template <class P, class V>
	void axisctr<P, V>::serve_impl()
	{
		P ctrpos;
		V ctrspd;

		if (is_alarmed()) 
		{
			ralgo::warn("axisctr is_alarmed");
			return;
		}

		// Установить текущие целевые параметры.
		int sts = curtraj->attime(ralgo::discrete_time(), ctrpos, ctrspd);
		controlled->ctrpos = ctrpos;
		controlled->ctrspd = ctrspd;

		if (sts && !operation_finished_flag)
		{
			if (debug_mode)
				nos::println("axisctr:", mnemo(), "finish signal");
			operation_finished_flag = true;
			operation_finish_signal(this);
			lintraj.set_point_hold(ctrpos);
			curtraj = &lintraj;
		}
	}

	template<class P, class V>
	int axisctr<P, V>::stop()
	{
		if (!is_active())
		{
			ralgo::warn("axisctr: not active");
			return -1;
		}

		if (curtraj == nullptr)
			return 0;

		lintraj.set_stop_trajectory(
		    controlled->feedpos,
		    controlled->feedspd,
		    dcc);

		operation_finished_flag = false;
		curtraj = & lintraj;
		return 0;
	}

	template<class P, class V>
	void axisctr<P, V>::hardstop()
	{
		lintraj.set_point_hold(controlled->feedpos);

		controlled->ctrspd = 0;
		controlled->ctrpos = controlled->feedpos;

		operation_finish_signal(this);
		operation_finished_flag = true;
		controlled->hardstop();
		curtraj = & lintraj;
	}

	template<class P, class V>
	int axisctr<P, V>::command(int argc, char** argv)
	{
		float fltarg;

		if (strcmp(argv[0], "mov") == 0)
		{
			fltarg = atof32(argv[1], nullptr);
			int ret = absmove(fltarg);
			return ret;
		}

		else if (strcmp(argv[0], "incmov") == 0)
		{
			fltarg = atof32(argv[1], nullptr);
			int ret = incmove(fltarg);
			return ret;
		}

		else if (strcmp(argv[0], "stop") == 0)
		{
			int ret = stop();
			return ret;
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
			set_accdcc(fltarg, fltarg);
			return 0;
		}

		else if (strcmp(argv[0], "setgain") == 0)
		{
			fltarg = atof32(argv[1], nullptr);
			set_gain(fltarg);
			return 0;
		}

		else if (strcmp(argv[0], "setlim") == 0)
		{
			if (argc != 3) 
			{
				nos::println("setlim:wrong args count");
				return -1;
			}

			P a = atof32(argv[1], nullptr);
			P b = atof32(argv[2], nullptr);
		
			if (a > b) 
				return -1;  

			set_limits(a, b);
			return 0;
		}

		else if (strcmp(argv[0], "feed") == 0)
		{
			print_info();
			return 0;
		}

		else if (strcmp(argv[0], "pos") == 0)
		{
			char buf[128];
			nos::format_buffer(buf, "{}\n", feedback_position());
			printf("%s", buf);
			return 0;
		}


		else if (strcmp(argv[0], "name") == 0)
		{
			nos::println("name:", this->mnemo());
			nos::println("controlled:", this->controlled->mnemo());
			return 0;
		}

		else
		{
			nos::println("warn: unresolved command");
		}

		return 0;
	}
}

#endif