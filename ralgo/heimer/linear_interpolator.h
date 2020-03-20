#ifndef RALGO_HEIMER_LINEAR_INTERPOLATOR_H
#define RALGO_HEIMER_LINEAR_INTERPOLATOR_H

#include <iterator>

#include <igris/dtrace.h>

#include <ralgo/linalg/vecops.h>

#include <ralgo/heimer/interpolation_group.h>
#include <ralgo/heimer/coordinate_checker.h>
#include <ralgo/heimer/control.h>

#include <ralgo/planning/trajNd.h>

namespace ralgo
{
	namespace heimer
	{
		template<int Dim, class Position, class Speed>
		class linear_interpolator :
			public interpolation_group<Position, Speed>,
			public external_controller,
			public control_served,
			public control_info_node
		{
			// Линейная интерполяция в декартовой метрике.

			using parent = interpolation_group<Position, Speed>;
			interpolation_coordinate_checker<Position> * coord_checker = nullptr;

			Speed _speed = 0.1;
			//float _accdcc = 0;
			float _acc_val = 0.1;
			float _dcc_val = 0.1;

			trajNd<Dim, Position, Speed> * trajectory;
			trajNd_line<Dim, Position, Speed> lintraj;

//			Speed compspd[Dim];

			Position ctrpos[Dim];
			Speed ctrspd[Dim];

			Position feedpos[Dim];
			Speed feedspd[Dim];

			float _gains[Dim];

			float poskoeff = 0.01;
			bool _in_operation = false;


			igris::array_view<heimer::axis_driver<Position, Speed>*> _axes;

		public:
			linear_interpolator(
			    const char* name,
			    igris::array_view<heimer::axis_driver<Position, Speed>*> axes) :
				parent(name),
				control_info_node(name, this, this, nullptr), _axes(axes)
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
				auto dist = ralgo::vecops::distance(curpos, tgtpos);

				int64_t time = (int64_t)(((Speed)fabs(dist)) / _speed * ralgo::discrete_time_frequency());
				int64_t curtime = ralgo::discrete_time();
				int64_t tgttim = curtime + time;

				if (dist == 0 || curtime == tgttim) 
				{	
					operation_finish(0);
					return 0;
				}

				lintraj.reset(curpos, curtime, tgtpos, tgttim);
				lintraj.set_speed_pattern(_acc_val, _dcc_val, _speed);

				trajectory = &lintraj;
				_in_operation = true;

				return 0;
			}

			int incmove(
			    igris::array_view<Position> mov
			)
			{
				Position curpos[Dim];
				Position tgtpos[Dim];

				if (try_operation_begin(0))
				{
					return -1;
				}

				for (unsigned int i = 0; i < mov.size(); ++i)
				{
					curpos[i] = get_axis(i)->current_position();
					tgtpos[i] = curpos[i] + mov[i] * _gains[i];
				}

				return move(curpos, tgtpos);
			}

			int absmove(
			    igris::array_view<Position> pos
			)
			{
				Position curpos[Dim];
				Position tgtpos[Dim];

				if (try_operation_begin(0))
				{
					return -1;
				}

				for (unsigned int i = 0; i < pos.size(); ++i)
				{
					curpos[i] = get_axis(i)->current_position();
					tgtpos[i] = pos[i] * _gains[i];
				}

				return move(curpos, tgtpos);
			}

			int incmove(Position * mov) override
			{
				return incmove({mov, Dim});
			}

			int absmove(Position * pos) override
			{
				return absmove({pos, Dim});
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

			void update_control_by_trajectory()
			{
				int is_finish = trajectory->attime(ralgo::discrete_time(), ctrpos, ctrspd,
				                                   ralgo::discrete_time_frequency() /*time_multiplier*/);

				//(void) is_finish;
				if (is_finish)
				{
					_in_operation = false;
				}
			}

			void apply_phase()
			{
				for (int i = 0; i < Dim; ++i)
				{
					_axes[i]->direct_control(ctrpos[i], ctrspd[i]);
				}
			}

			void collect_gains()
			{
				for (unsigned int i = 0; i < _axes.size(); ++i)
				{
					_gains[i] = _axes[i]->gain();
				}
			}

			void update_control_model()
			{
				update_state();
			}

			void update_state()
			{
				for (int i = 0; i < Dim; ++i)
				{
					feedpos[i] = _axes[i]->feedpos;
					feedspd[i] = _axes[i]->feedspd;
				}
			}

			void serve_impl()
			{
				if (trajectory)
					update_control_by_trajectory();

				apply_phase();
			}

			int print_feed() 
			{
				for(int i=0; i<Dim; ++i) 
				{
					nos::fprintln("\tfeed:(pos:{},spd:{})", feedpos[i], feedspd[i]);
				}
				return 0;
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

			void control_interrupt_from(external_control_slot * slot)  override
			{
				BUG();
			}

			external_control_slot* iterate(external_control_slot* slt) override
			{
				if (slt == nullptr)
					return _axes[0]->as_controlled();

				for (unsigned int i = 0; i < _axes.size(); ++i)
				{
					if (slt == _axes[i])
					{
						if (i == _axes.size() - 1) return nullptr;
						else return _axes[i + 1];
					}
				}
				BUG();
			}

			bool is_extern_controlled() { return false; }
			bool in_operation() { return _in_operation; }

			int try_operation_begin(int priority) override
			{
				if (!is_active()) return -1;
				if (is_extern_controlled()) return -1;
				if (in_operation() && priority == 0) {
					parent::stop();
					return -1;
				}
				return 0;
			}

			void on_activate_handle() override { update_control_model(); }
			void on_deactivate_handle() override {  }

		private:
			//void after_take_control_handle() override {}
			//igris::array_view<controlled*> controlled_devices() override { return _controlled_device; }

			int hardstop() override
			{
				trajectory = nullptr;
				operation_finish(1);

				return 0;
			}

			void operation_finish(int priority)
			{
			}

			int stop_impl() override
			{
				if (trajectory == nullptr)
					return 0;

				lintraj.set_stop_trajectory(
				    feedpos,
				    feedspd,
				    _dcc_val);

				trajectory = & lintraj;
				return 0;
			}

			int try_activate_impl() override
			{
				return take_control();
			}

			int try_deactivate_impl() override
			{
				return release_control();
			}

			bool can_operate() override 
			{
				return is_active() && !is_extern_controlled() && !in_operation();
			}
		};
	}
}

#endif