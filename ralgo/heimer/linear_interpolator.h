#ifndef RALGO_HEIMER_LINEAR_INTERPOLATOR_H
#define RALGO_HEIMER_LINEAR_INTERPOLATOR_H

#include <iterator>

#include <ralgo/heimer/interpolation_group.h>
#include <ralgo/planning/trajNd.h>
#include <ralgo/linalg/vecops.h>

#include <igris/dtrace.h>

#include <ralgo/heimer/coordinate_checker.h>

namespace ralgo
{
	namespace heimer
	{
		template<int Dim, class Position, class Speed>
		class linear_interpolator : public interpolation_group<Position, Speed>
		{
			// Линейная интерполяция в декартовой метрике.
			// Disclaimer: Линейная интерполяция - это немного другое...
			// ее тут пока нет...

			using parent = interpolation_group<Position, Speed>;
			interpolation_coordinate_checker<Position> * coord_checker = nullptr;

			Speed _speed = 0;
			//float _accdcc = 0;
			float _acc_val = 0;
			float _dcc_val = 0;

			trajNd<Dim, Position, Speed> * trajectory;
			trajNd_line<Dim, Position, Speed> lintraj;

			Position ctrpos[Dim];
			Speed compspd[Dim];
			Speed ctrspd[Dim];
			float poskoeff = 0.01;

			union {
				igris::array_view<heimer::axis_device<Position,Speed>*> _axes;
				igris::array_view<heimer::controlled*> _controlled_device;
			};

		public:
			linear_interpolator(
				const char* name,
			    igris::array_view<heimer::axis_device<Position,Speed>*> axes) :
					parent(name), _axes(axes)
			{
				//parent::set_controlled(axes);
			}

			constexpr int dim() override { return Dim; }

			axis_device<float, float> * get_axis(int index)
			{
			//	return static_cast<axis_device<Position, Speed>*>(
			//		parent::controlled()[index]);
				return _axes[index];
			}

			int move(
				igris::array_view<Position> curpos, 
				igris::array_view<Position> tgtpos) 
			{

				int success = parent::take_control();
				if (success == false) 
				{
					return -1;
				}

				auto dist = ralgo::vecops::distance(curpos, tgtpos);
				int64_t time = (int64_t)((Speed)abs(dist) / _speed * ralgo::discrete_time_frequency());
				int64_t curtime = ralgo::discrete_time();
				int64_t tgttim = curtime + time;

//				for (int i = 0; i < mov.size(); ++i)
//				{
//					auto curpos = get_axis(i)->current_position();
//					lintraj.set_start_position(i, curpos);
//				}

//				for (int i = 0; i < tgtpos.size(); ++i)
//				{
//					lintraj.set_finish_position_inc(i, tgtpos[i]);
//				}
				DPRINT(dist);
				DPRINT(time);
				DPRINT(curtime);
				DPRINT(tgttim);

				for (int i = 0; i < Dim; ++i) 
				{
					DPRINT(tgtpos[i]);
					DPRINT(curpos[i]);
				}


				lintraj.reset(curpos, curtime, tgtpos, tgttim);
				lintraj.set_speed_pattern(_acc_val, _dcc_val, _speed);
				//lintraj.spddeform.reset(_accdcc, _accdcc);

				trajectory = &lintraj;

				return 0;
			}

			int incmove(
			    igris::array_view<Position> mov
			)
			{
				Position curpos[Dim];
				Position tgtpos[Dim];
			
				if (!parent::take_control()) 
				{
					ralgo::warning("linint take_control fault");
					return -1;
				}

				for (unsigned int i = 0; i < mov.size(); ++i) 
				{
					curpos[i] = get_axis(i)->current_position();
					tgtpos[i] = curpos[i] + mov[i];
				}

				return move(curpos, tgtpos);	
			}

			int absmove(
			    igris::array_view<Position> pos
			)
			{
				Position curpos[Dim];
				Position tgtpos[Dim];

				if (!parent::take_control()) 
				{
					ralgo::warning("linint take_control fault");
					return -1;
				}
			
				for (int i = 0; i < pos.size(); ++i) 
				{
					curpos[i] = get_axis(i)->current_position();
					tgtpos[i] = pos[i];
				}

				return move(curpos, tgtpos);	
			}

			int incmove(Position * mov) override 
			{
				return incmove({mov,Dim});
			}

			int absmove(Position * pos) override 
			{
				return incmove({pos,Dim});
			}

			int set_speed(Speed speed) override
			{
				_speed = speed;
				return 0;
			}

			// Установить ускорение в собственных единицах 1/c^2. 
			int set_accdcc_value(float acc, float dcc) override
			{
				_acc_val = acc;
				_dcc_val = dcc; 
				return 0;
			}

			void update_phase()
			{
				int is_finish = trajectory->attime(ralgo::discrete_time(), ctrpos, ctrspd, 
					ralgo::discrete_time_frequency() /*time_multiplier*/);

				(void) is_finish;
			}

			void apply_phase()
			{
				for (int i = 0; i < Dim; ++i)
				{
					get_axis(i)->direct_control(ctrpos[i], ctrspd[i]);
				}
			}

			void serve()
			{
				if (trajectory && parent::controller() == this)
				{
					update_phase();
					apply_phase();
				}
			}

			Speed speed() override { return _speed; }
			Speed acceleration() override { return _acc_val; }
			Speed deceleration() override { return _dcc_val; }

			void debug_print_traj() 
			{
				for (int i = 0; i < Dim; i++) 
				{
					DPRINT(i);
					DPRINT(lintraj.spos[i]);
					DPRINT(lintraj.fpos[i]);
				}
			}

			void debug_print_state() 
			{
				for (int i = 0; i < Dim; i++) 
				{
					DPRINT(i);
					DPRINT(ctrpos[i]);
					DPRINT(ctrspd[i]);
				}
			}

		private:
			void after_take_control_handle() override {}
			igris::array_view<controlled*> controlled_devices() override { return _controlled_device; }
		};
	}
}

#endif