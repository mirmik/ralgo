#ifndef RALGO_HEIMER_LINEAR_INTERPOLATOR_H
#define RALGO_HEIMER_LINEAR_INTERPOLATOR_H

#include <iterator>

#include <igris/dtrace.h>

#include <ralgo/linalg/vecops.h>

#include <ralgo/heimer/interpolation_group.h>
#include <ralgo/heimer/coordinate_checker.h>
#include <ralgo/heimer/control.h>

#include <ralgo/trajectory/trajNd.h>

namespace heimer
{
	template<class Position, class Speed, int Dim>
	class linintctr :
		public linintctr_basic<Position, Speed>
	{
		using parent = linintctr_basic<Position, Speed>;

		// Линейная интерполяция в декартовой метрике.

		//using parent = interpolation_group<Position, Speed>;
		interpolation_coordinate_checker<Position> * coord_checker = nullptr;

		linalg::vec<Position, 2> zone_polygon[8];
		plane_zone_checker<Position> polygon_checker;

		Speed _speed = 1;
		//float _accdcc = 0;
		Speed _acc_val = 1;
		Speed _dcc_val = 1;

		ralgo::trajNd<Dim, Position, Speed> * trajectory;
		ralgo::trajNd_line<Dim, Position, Speed> lintraj;

//			Speed compspd[Dim];

		Position ctrpos[Dim] = {};
		Speed ctrspd[Dim] = {};

		Position feedpos[Dim] = {};
		Speed feedspd[Dim] = {};

		float _gains[Dim] = {};

		float poskoeff = 0.01;
		bool _in_operation = false;

		igris::array_view<heimer::axis_node<Position, Speed>*> _axes;

	public:
		bool in_operate() 
		{
			return _in_operation;
		}

		linintctr(
		    const char* name,
		    igris::array_view<heimer::axis_node<Position, Speed>*> axes
		) :
			linintctr(name, axes.data())
		{}

		linintctr(
		    const char* name,
		    heimer::axis_node<Position, Speed>** axes
		) :
			linintctr_basic<Position,Speed>(name),
			polygon_checker(zone_polygon),
			_axes(axes, Dim)
		{
			vecops::fill(_gains, float(1));
		}

		constexpr int dim() { return Dim; }

		void set_task_checker(interpolation_coordinate_checker<Position> * coord_checker)
		{
			this->coord_checker = coord_checker;
		}

		axis_node<float, float> * get_axis(int index)
		{
			//	return static_cast<axis_device<Position, Speed>*>(
			//		parent::controlled()[index]);
			return _axes[index];
		}

		int move(
		    igris::array_view<Position> curpos,
		    igris::array_view<Position> tgtpos)
		{
			if (!parent::is_active()) 
			{
				dprln(1);
				return HEIM_ERR_IS_NOACTIVE;
			}

			auto dist = ralgo::vecops::distance(curpos, tgtpos);
			DPRINT(dist);

			int64_t time = (int64_t)(((Speed)fabs(dist)) / _speed * ralgo::discrete_time_frequency());
			int64_t curtime = ralgo::discrete_time();
			int64_t tgttim = curtime + time;

			DPRINT(time);
			DPRINT(curtime);
			DPRINT(tgttim);
			if (dist == 0 || curtime == tgttim)
			{
				dprln(2);
				return 0;
			}

			lintraj.reset(curpos, curtime, tgtpos, tgttim);
			lintraj.set_speed_pattern(_acc_val, _dcc_val, _speed);

			trajectory = &lintraj;
			_in_operation = true;

				dprln(3);
			return 0;
		}

		int incmove(
		    igris::array_view<Position> mov
		)
		{
			Position curpos[Dim];
			Position tgtpos[Dim];

			for (unsigned int i = 0; i < mov.size(); ++i)
			{
				curpos[i] = get_axis(i)->target_position();
				tgtpos[i] = curpos[i] + mov[i] * _gains[i];
			}

			return move(curpos, tgtpos);
		}

		void set_zone_protection(
		    igris::array_view<linalg::vec<Position, 2>> arr
		)
		{
			std::copy(arr.begin(), arr.end(), std::begin(zone_polygon));
		}


		int absmove(
		    igris::array_view<Position> pos
		)
		{
			Position curpos[Dim];
			Position tgtpos[Dim];

			for (unsigned int i = 0; i < pos.size(); ++i)
			{
				curpos[i] = get_axis(i)->target_position();
				tgtpos[i] = pos[i] * _gains[i];
			}

			return move(curpos, tgtpos);
		}

		int incmove(Position * mov)
		{
			return incmove({mov, Dim});
		}

		int absmove(Position * pos)
		{
			return absmove({pos, Dim});
		}

		int set_speed(Speed speed)
		{
			_speed = speed;
			return 0;
		}

		// Установить ускорение в собственных единицах 1/c^2.
		int set_accdcc(float acc, float dcc)
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
				_axes[i]->control(ctrpos[i], ctrspd[i]);
			}
		}

		/*void collect_gains()
		{
			for (unsigned int i = 0; i < _axes.size(); ++i)
			{
				_gains[i] = _axes[i]->gain();
			}
		}*/

		void set_gains(igris::array_view<float> arr) 
		{
			for (int i = 0; i < arr.size(); ++i) 
				_gains[i] = arr[i];
		} 

		void update_control_model()
		{
			feedback();
		}

		void feedback()
		{
			for (int i = 0; i < Dim; ++i)
			{
				feedpos[i] = _axes[i]->feedpos;
				feedspd[i] = _axes[i]->feedspd;
			}
		}

		void serve()
		{
			if (trajectory)
				update_control_by_trajectory();

			apply_phase();
		}

		int print_feed()
		{
			for (int i = 0; i < Dim; ++i)
			{
				nos::fprintln("\tfeed:(pos:{},spd:{})", feedpos[i], feedspd[i]);
			}
			return 0;
		}

		Speed speed() { return _speed; }
		Speed acceleration() { return _acc_val; }
		Speed deceleration() { return _dcc_val; }

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

		/*void control_interrupt_from(external_control_slot * slot)  override
		{
			BUG();
		}*/

		control_node* iterate(control_node * slt) override
		{
			if (slt == nullptr)
				return _axes[0];

			for (unsigned int i = 0; i < _axes.size() - 1; ++i)
			{
				if (slt == _axes[i])
				{
					return _axes[i + 1];
				}
			}

			return nullptr;
		}

		bool is_extern_controlled() { return false; }
		bool in_operation() { return _in_operation; }

		int try_operation_begin(int priority) override
		{
			if (!parent::is_active()) return -1;
			if (in_operation() && priority == 0)
			{
				parent::stop();
				return -1;
			}
			return 0;
		}
		/*
		void on_activate_handle() override { update_control_model(); }
		void on_deactivate_handle() override {  }
		*/
	public:
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
		/*
				int try_activate_impl() override
				{
					return take_control();
				}

				int try_deactivate_impl() override
				{
					return release_control();
				}
*/
		bool can_operate() override
		{
			return parent::is_active() 
				&& !in_operation();
		}
	};
}

#endif