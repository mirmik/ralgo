#ifndef RALGO_PLANNING_TRAJ_ND_H
#define RALGO_PLANNING_TRAJ_ND_H

#include <nos/fprint.h>
#include <ralgo/planning/phase.h>
#include <ralgo/planning/speed_deformer.h>

namespace ralgo
{
	template <size_t Dim, class Position, class Speed>
	class trajNd
	{
	protected:
		int64_t stim;
		int64_t ftim;

	public:
		virtual int attime(
		    int64_t time,
		    igris::array_view<Position> pos,
		    igris::array_view<Speed> spd
		) = 0;
		
		virtual bool is_finished(int64_t time) 
		{
			return time > ftim;
		}
	};

	template <size_t Dim, class Position, class Speed>
	class trajNd_line : public trajNd<Dim, Position, Speed>
	{
		using trajNd<Dim, Position, Speed>::stim;
		using trajNd<Dim, Position, Speed>::ftim;

		Position spos[Dim];
		Position fpos[Dim];

		Speed setted_speed[Dim];

	public:
		ralgo::speed_deformer spddeform;

		void set_start_position(int i, Position pos) 
		{
			spos[i] = pos;
		}

		void set_finish_position_inc(int i, Position inc) 
		{
			fpos[i] = spos[i] + inc;
		}

		// Инициализировать траекторию, когда spos и fpos уже установлены.
		int reset(
		    int64_t stim,
		    int64_t ftim
		)
		{
			this->stim = stim;
			this->ftim = ftim;

			for (int i = 0; i < Dim; ++i)
			{
				if (stim == ftim)
					setted_speed[i] = 0;
				else
				{
					DPRINT(spos[i]);
					DPRINT(fpos[i]);
					DPRINT(stim);
					DPRINT(ftim);
					setted_speed[i] = (float)(fpos[i] - spos[i]) / (ftim - stim);
					DPRINT(setted_speed[i]);
				}
			}
		}

		int reset(
		    igris::array_view<Position>& spos,
		    int64_t stim,
		    igris::array_view<Position>& fpos,
		    int64_t ftim
		)
		{
			std::copy(std::begin(spos), std::end(spos), std::begin(this->spos));
			std::copy(std::begin(fpos), std::end(fpos), std::begin(this->fpos));
			reset(stim, ftim);
		}

		int attime(int64_t time, 
			igris::array_view<Position> pos, 
			igris::array_view<Speed> spd) override
		{
			float time_unit = (float)(time - stim) / (ftim - stim);
			auto posmod = spddeform.posmod(time_unit);
			auto spdmod = spddeform.spdmod(time_unit);

			for (int i = 0; i < Dim; ++i) {
				pos[i] = fpos[i] * posmod + spos[i] * (1 - posmod);
				spd[i] = setted_speed[i] * spdmod;
			}

			if (posmod >= 1) return 1;
			else return 0;
		}

		ssize_t print_to(nos::ostream& os) const 
		{
			return nos::fprint_to(os, "({},{},{})", spos, fpos, setted_speed);
		}
	};
}

#endif