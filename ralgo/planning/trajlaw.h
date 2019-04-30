#ifndef TRAJLAW_H
#define TRAJLAW_H

namespace ralgo
{
	struct paramlaw_current
	{
		float progress; // real trajectory parametr
		float spdmod; // speed modifier
	};

	template <class P = int64_t, class V = float, class A = float,
	          class T = time_t>
	class paramlaw
	{
		virtual int operator()(float t, struct paramlaw_current * out) = 0;
	};


	struct acc_dcc_paramlaw : public paramlaw
	{
		float acc;
		float lin;
		float dcc;

		float full;

		float pacc;
		float pdcc;
		float spd_acceleration_factor;

		acc_dcc_paramlaw(float acc, float lin, float dcc)
			: acc(acc), lin(lin), dcc(dcc), full(acc + lin + dcc)
		{
			spd_acceleration_factor = full / ((acc + dcc) / 2 + lin);

			pacc = acc / full * spd_acceleration_factor;
			pdcc = dcc / full * spd_acceleration_factor;
		}

		int operator()(float t, struct paramlaw_current * out)
		{
			float p;

			if (t > full)
			{
				out->progress = 1;
				out->spdmod = 0;
				return -1;
			}

			else if (t < acc)
			{
				p = t / acc;
				out->progress = p * p * pacc;
				out->spdmod = p * spd_acceleration_factor;
			}

			else if (t < lin)
			{
				float local = t - acc;
				p = local / lin;

				out->progress = lerp(pacc, (1 - pdcc), p);
				out->spdmod = spd_acceleration_factor;
			}

			else //t< dcc
			{
				float local = t - acc - lin;
				p = 1 - local / dcc;
				out->progress = (1 - p * p) * pdcc;
				out->spdmod = p * spd_acceleration_factor;
			}
		}
	}
}

#endif