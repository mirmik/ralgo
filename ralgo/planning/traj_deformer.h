#ifndef RALGO_TRAJ_DEFORMER_H
#define RALGO_TRAJ_DEFORMER_H

namespace ralgo
{
	template<class T_tim_t>
	class traj_deformer
	{
		T fin;
		V max_spd;
	};


	template<class P = pos_t, class V = spd_t, class T = tim_t>
	class acc_dcc_time_deformer
	{
		float acc;
		float dcc;

		float fini_acc_pos;
		float strt_dcc_pos;

		float strt_spd;
		float fini_spd;

		acc_dcc_time_deformer(tim_t acc, tim_t lin, tim_t dcc,
		                      spd_t sspd = 0, spd_t fspd = 0)
			: acc(acc), lin(lin), dcc(dcc),
			  strt_spd(sspd), fini_spd(fspd)
		{
			fin = 1 + (1 - strt_spd) * acc / 2 + (1 - fini_spd) * dcc / 2;
			max_spd = 1;

			fini_acc_pos = (1 + strt_spd) / 2 * acc;
			strt_dcc_pos = 1 - (fini_spd + 1) / 2 * dcc;
		}

		float posmod(T t)
		{
			if (t < acc)
			{
				return
				    t * strt_spd
				    + t * t * (1 - strt_spd) / acc / 2;
			}

			else if (t < fin - dcc)
			{
				t = t - acc;  
				return 
					fini_acc_pos + 
					t;// * 1;
			}

			else 
			{
				t = t - (fin - dcc);
				return 
					strt_dcc_pos 
					+ t
					+ t * t * (fini_spd - 1) / dcc / 2; 
			}
		}

		float spdmod() 
		{

			if (t < acc)
			{
				return
				    strt_spd
				    + t * (1 - strt_spd) / acc;
			}

			else if (t < fin - dcc)
			{
				return 1;
			}

			else 
			{
				t = t - (fin - dcc);
				return 
					1 + 
					t * (fini_spd - 1) / dcc; 
			}
		};
	};
}

#endif