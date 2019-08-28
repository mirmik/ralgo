#ifndef RALGO_PLANNING_TRAJ1D_H
#define RALGO_PLANNING_TRAJ1D_H

#include <ralgo/planning/traj.h>
#include <ralgo/planning/phase.h>
#include <ralgo/planning/speed_deformer.h>

namespace ralgo
{

	template <class P, class V>	
	class traj1d : public speed_deformed
	{
	public:
		virtual int attime(int64_t time, P& pos, V& spd) = 0;
	};


	template <class P, class V>
	class traj1d_line : public traj1d<P,V>
	{
		int64_t stim;
		int64_t ftim;

		int64_t spos;
		int64_t fpos;

		float setted_speed;

	public:
		ralgo::speed_deformer spddeform;

		traj1d_line() {}

		void reset(int64_t spos, int64_t stim, int64_t fpos, int64_t ftim)
		{
			this->spos = spos;
			this->fpos = fpos;

			this->stim = stim;
			this->ftim = ftim;

			setted_speed = (fpos - spos) / (ftim - stim);
		}

		int attime(int64_t time, P& pos, V& spd) override
		{
			float time_unit = (float)time / (ftim - stim);

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

			DPRINT(koeff);
			DPRINT(nominal_speed);
			DPRINT(setted_speed);
			DPRINT(acc);
			DPRINT(dcc);
			DPRINT(stim);
			DPRINT(ftim);
			DPRINT(acctime);
			DPRINT(dcctime);

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