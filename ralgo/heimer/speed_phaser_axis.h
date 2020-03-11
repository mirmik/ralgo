#ifndef RALGO_HEIMER_SPEED_PHASER_AXIS_H
#define RALGO_HEIMER_SPEED_PHASER_AXIS_H

#include <ralgo/heimer/speed_phaser.h>
#include <ralgo/heimer/axis_driver.h>

namespace ralgo
{
	namespace heimer
	{
		template < class Position, class IntPos, class Speed >
		class speed_phaser_axis : public axis_driver<Position, Speed>, public heimer::device
		{
			using parent = axis_driver<Position, Speed>;

			speed_phaser<Position, IntPos, Speed> * _phaser = nullptr;
			Speed compspd = 0;

		public:
			speed_phaser_axis(const char* name, speed_phaser<Position, IntPos, Speed>* phaser)
				: device(name), _phaser(phaser)
			{}

			Position current_position() override
			{
				return _phaser->target_position();
			}

			/*void set_current_position(Position pos) override
			{
				_phaser->set_current_position(pos);
			}*/

			void apply_speed(Speed spd)
			{
				_phaser->set_speed(spd);
			}

			speed_phaser<Position, IntPos, Speed> * phaser()
			{
				return _phaser;
			}

			bool try_operation_begin(int priority) override
			{
				DTRACE();
				switch (priority)
				{
					case 0: return take_control();
					case 1: take_control_force(); break;
					default: BUG();
				}
				return true;
			}

			void operation_finish(int priority) override
			{
				DTRACE();
				switch (priority)
				{
					case 0: release_control(); break;
					case 1: release_control_force(); break;
					default: BUG();
				}
			}


			void serve()
			{
				//DPRINTPTR(parent::controller());
				if (parent::_current_trajectory && controller() == this)
				{
					parent::update_control_by_trajectory();
				}

				apply_control();
			}

//			virtual void apply_speed(Speed spd) {}

			void apply_control()
			{
				compspd = parent::eval_compensated_speed();
				apply_speed(compspd);
			}

			Speed compensated_speed()
			{
				return compspd;
			}

			Speed current_speed()
			{
				return compspd;
			}

		private:
			heimer::controlled* as_controlled() override { return this; }

		private:
			void after_take_control_handle() override {}
			igris::array_view<controlled*> controlled_devices() override
			{
				return igris::array_view<controlled*>(nullptr, 0);
			}
		};
	}
}

#endif