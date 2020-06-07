#ifndef RALGO_HEIMER_SPEED_PHASER_AXIS_H
#define RALGO_HEIMER_SPEED_PHASER_AXIS_H

//#error "modernize with axis model"

#include <ralgo/heimer/speed_phaser.h>
#include <ralgo/heimer/axis_model.h>

#include <ralgo/heimer/control.h>

namespace ralgo
{
	namespace heimer
	{
		template < class Position, class IntPos, class Speed >
		class speed_phaser_axis : public axis_model<Position, Speed>
		{
			using parent = axis_model<Position, Speed>;

			speed_phaser<Position, IntPos, Speed> * _phaser = nullptr;
			Speed compspd = 0;

		public:
			speed_phaser_axis(const char* name, speed_phaser<Position, IntPos, Speed>* phaser)
				: axis_model<Position, Speed>(name), _phaser(phaser)
			{}

			void apply_speed(Speed spd)
			{
				_phaser->set_speed(spd);
			}

			speed_phaser<Position, IntPos, Speed> * phaser()
			{
				return _phaser;
			}

			int try_operation_begin(int priority) override
			{
				// Вынести в класс axis_driver.
				if (!parent::is_active()) return -1;
				if (parent::is_extern_controlled()) return -1;
				if (parent::in_operation() && priority == 0) 
				{
					parent::stop();
					return -1;
				}
				return 0; 
			}

			void operation_finish(int priority) override
			{
			}


			void serve_impl()
			{
				parent::evaluate_ctrvars();

				apply_control();
			}

			void apply_control() override
			{
				compspd = parent::eval_compensated_speed();
				apply_speed(compspd);
			}

			int try_activate_impl() override 
			{
				return 0;
			}

			int try_deactivate_impl() override 
			{
				return 0;
			}

			Speed compensated_speed()
			{
				return compspd;
			}

			void update_state()
			{
				parent::feedpos = _phaser->target_position();
				parent::feedspd = _phaser->feedback_speed();
			}

		private:
			int try_take_external_control_impl(external_controller * controller) override 
			{
				if (parent::is_active() == false) return -1;
				if (parent::in_operation()) return -1;
				return 0;
			}

			int try_release_external_control_impl(external_controller * controller) override 
			{
				return 0;
			}

			void on_activate_handle() override {}
			void on_deactivate_handle() override {}
			external_control_slot* as_controlled() override { return this; }

			const char * name() { return parent::mnemo(); }

			bool can_operate() override 
			{
				return parent::is_active() 
					&& !parent::is_extern_controlled() 
					&& !parent::in_operation();
			}
		};
	}
}

#endif