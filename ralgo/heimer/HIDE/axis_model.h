#ifndef RALGO_HEIMER_AXIS_MODEL_H
#define RALGO_HEIMER_AXIS_MODEL_H

#include <ralgo/heimer/axis_driver.h>

namespace ralgo
{
	namespace heimer
	{
		template <class P, class V>
		class axis_model : 
			public axis_driver<P, V>,
			public control_served,
			public control_info_node
		{
		public:
			using parent = axis_driver<P, V>;
			axis_model(const char* name)
				: control_info_node(name, this, 0, this)
			{}

			virtual void update_state()
			{
				parent::feedpos = parent::ctrpos;
				parent::feedspd = parent::ctrspd;
			}

		protected:		
			int try_operation_begin(int priority) override
			{
				// Вынести в класс axis_driver.
				if (!is_active()) return -1;
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

			void serve_impl() override
			{
				parent::evaluate_ctrvars();
				apply_control();
			}

			virtual void apply_control()
			{}

		private:
			int try_take_external_control_impl(external_controller * controller) override 
			{
				if (is_active() == false) return -1;
				if (parent::in_operation()) return -1;
				return 0;
			}

			int try_release_external_control_impl(external_controller * controller) override 
			{
				return 0;
			}

			int try_activate_impl() override 
			{
				return 0;
			}

			int try_deactivate_impl() override 
			{
				return 0;
			}

			void on_activate_handle() override {}
			void on_deactivate_handle() override {}
			external_control_slot* as_controlled() override { return this; }

			const char * name() { return mnemo(); }

			bool can_operate() override 
			{
				return is_active() && !parent::is_extern_controlled() && !parent::in_operation();
			}

			void print_info() override 
			{ 
				nos::fprintln("{} feed:{},{}", parent::current_trajectory(), parent::feedpos, parent::feedspd);
			}
		};
	}
}

#endif