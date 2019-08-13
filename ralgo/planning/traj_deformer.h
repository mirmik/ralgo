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

		float a_acc_div_2;
		float a_dcc_div_2;

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

			a_acc_div_2 = (1 - strt_spd) / acc / 2;
			a_dcc_div_2 = (fini_spd - 1) / dcc / 2;
		}

		float posmod(T t)
		{
			/**
				Расчет траекторного коэффициента идёт по формуле
				x = x0 + v0*t + a*t*t/2
				по кусочным участкам.
			*/

			if (t < acc)
			{
				// x0=0 
				// v0=sspd 
				// a=(1-sspd)/acc 
				return
				    t * strt_spd
				    + t * t * a_acc_div_2;
			}

			else if (t < fin - dcc)
			{
				// x0=facc 
				// v0=1 
				// a=0 
				t = t - acc;  
				return 
					fini_acc_pos + 
					t;
			}

			else 
			{
				// x0=sdcc 
				// v0=1 
				// a=(fspd-1)/dcc 
				t = t - (fin - dcc);
				return 
					strt_dcc_pos 
					+ t
					+ t * t * a_dcc_div_2; 
			}
		}

		float spdmod() 
		{
			/**
				Расчет скоростного коэффициента идёт по формуле
				x = v0 + a*t
				по кусочным участкам.
			*/

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