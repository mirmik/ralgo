#ifndef RALGO_PLANNING_HTRANS_TRAJ_H
#define RALGO_PLANNING_HTRANS_TRAJ_H

#include <ralgo/planning/speed_deformer.h>

namespace ralgo 
{
	class htrans2_traj : public speed_deformed
	{
	public:
		rabbit::htrans2<float> spos, fpos;

		htrans2_traj() :
			speed_deformed(int64_t stim, int64_t ftim)
		{
			
		}

		virtual void attime(int64_t time,
			rabbit::htrans2& pos,
			rabbit::screw2& spd) = 0;
	};

	class htrans2_traj_linear : public htrans2_traj 
	{
		htrans2_traj_linear(int64_t stim, int64_t ftim, 
			rabbit::htrans2<float> spos, 
			rabbit::htrans2<float> fpos) 
		{

		}

		void attime(int64_t time,
			rabbit::htrans2& pos,
			rabbit::screw2& spd) override
		{
			float time_unit = (float)(time - stim) / (ftim - stim);

			auto posmod = spddeform.posmod(time_unit);
			auto spdmod = spddeform.spdmod(time_unit);

			auto pos = spos + (fpos - spos) * posmod;
			auto spd = setted_speed * spdmod;

			phs[0].d0 = pos;
			phs[0].d1 = spd;

			if (posmod >= 1) return 1;
			else return 0;
		}	
	}
}

#endif