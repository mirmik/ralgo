#ifndef RALGO_PLANNING_TRAJ1D_H
#define RALGO_PLANNING_TRAJ1D_H

#include <ralgo/planning/traj.h>
#include <ralgo/planning/phase.h>
#include <ralgo/planning/speed_deformer.h>

namespace ralgo
{

	template <class P, class V>	
	class traj1d
	{
	public:
		virtual int attime(int64_t time, P& pos, V& spd) = 0;
		virtual bool is_finished(int64_t time) = 0;
	};


	template <class P, class V>
	class traj1d_line : public traj1d<P,V>
	{
	public:
		int64_t stim = 0;
		int64_t ftim = 0;

		P spos = 0;
		P fpos = 0;

		V setted_speed = 0;

	public:
		ralgo::speed_deformer spddeform;

		traj1d_line() {}

		void reset(P spos, int64_t stim, P fpos, int64_t ftim)
		{
			this->spos = spos;
			this->fpos = fpos;

			this->stim = stim;
			this->ftim = ftim;

			if (stim == ftim)
				setted_speed = 0;
			else
			{
				setted_speed = (float)(fpos - spos) / (ftim - stim);
			}
		}

		bool is_finished(int64_t time) override
		{
			return time > ftim;
		}

		int attime(int64_t time, P& pos, V& spd) override
		{
			float time_unit = (float)(time - stim) / (ftim - stim);

			auto posmod = spddeform.posmod(time_unit);
			auto spdmod = spddeform.spdmod(time_unit);

			pos = fpos * posmod + spos * (1 - posmod);
			spd = setted_speed * spdmod;

			if (posmod >= 1) return 1;
			else return 0;
		}

		void set_standart_accdcc_patern(int64_t acctime, int64_t dcctime, float nominal_speed)
		{
			float koeff = setted_speed / nominal_speed;

			float acc = acctime * koeff / (ftim - stim);
			float dcc = dcctime * koeff / (ftim - stim);

			if (acc + dcc > 1)
			{
				//dprln("warn: acc + dcc > 1");

				acc = 0.5;
				dcc = 0.5;
			}
			//dprln(acc, dcc);

			spddeform.reset(
			    acc, dcc

			    //	acc,
			    //	dcc);
			);
		}
	};
}

#endif