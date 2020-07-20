#ifndef RALGO_HEIMER_SPEED_PHASER_AXIS_H
#define RALGO_HEIMER_SPEED_PHASER_AXIS_H

#include <ralgo/heimer/phaser.h>
#include <ralgo/heimer/axis.h>

namespace heimer
{
	template <class P, class IntPos, class V>
	class phaser_axis : public axis_node<P,V>
	{
		using parent = axis_node<P,V>;

	public:
		phaser<P,IntPos,V> * controlled = nullptr;
		V compspd = 0;
		float compkoeff = 0;

	public:
		constexpr 
		phaser_axis(const char* name) : axis_node<P,V>(name) {}
		
		constexpr 
		phaser_axis(const char* name, phaser<P,IntPos,V>* phaser)
			: axis_node<P,V>(name), controlled(phaser)
		{}

		void apply_speed(V spd)
		{
			controlled->set_speed(spd);
		}

		void feedback() 
		{
			parent::feedpos = controlled->feedback_position();
			parent::feedspd = controlled->feedback_speed();
		}

		void serve() 
		{
			// Счетчик меняется в прерывании, так что
			// снимаем локальную копию.
			P current = parent::feedpos;

			// Ошибка по установленному значению.
			P diff = parent::ctrpos - current;

			// Скорость вычисляется как
			// сумма уставной скорости на
			compspd = parent::ctrspd + compkoeff * diff;
			
			controlled->set_speed(compspd);
		}

		void set_compkoeff(float val) { compkoeff = val; }

		/*int try_operation_begin(int priority) override
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
		}*/

		/*void operation_finish(int priority) override
		{
		}*/


		/*void serve_impl()
		{
			parent::evaluate_ctrvars();

			apply_control();
		}*/

		/*void apply_control() override
		{
			compspd = parent::eval_compensated_speed();
			apply_speed(compspd);
		}*/

		V compensated_speed()
		{
			return compspd;
		}

		void update_state()
		{
			parent::feedpos = controlled->target_position();
			parent::feedspd = controlled->feedback_speed();
		}

	private:
	/*	int try_take_external_control_impl(external_controller * controller) override
		{
			if (parent::is_active() == false) return -1;
			if (parent::in_operation()) return -1;
			return 0;
		}

		int try_release_external_control_impl(external_contrte:
	/*	int try_take_external_control_impl(external_controller * controller) override
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
		}oller * controller) override
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
		}*/
	};
}

#endif