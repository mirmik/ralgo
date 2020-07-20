#ifndef RALGO_HEIMER_MULTIAX_H
#define RALGO_HEIMER_MULTIAX_H

#include <ralgo/heimer/axis.h>

namespace heimer
{
	//template <class P, class V> class virtual_multiax;

	template <class P, class V>
	class virtual_axis_node : public heimer::axis_node<P, V>
	{
		control_node * parent;

	public:
		virtual_axis_node(
			const char * mnemo, 
			control_node * parent
		) :
			axis_node<P,V>(mnemo),
			parent(parent)
		{}

		/*void init(const char* axname, virtual_multiax<P, V>* parent, int index)
		{}

		bool can_operate() override
		{
			return parent->is_active() && !driver::is_extern_controlled() && !driver::in_operation();
		}

		int try_operation_begin(int priority) override
		{
			if (!parent->is_active()) return -1;
			if (driver::is_extern_controlled()) return -1;
			if (driver::in_operation() && priority == 0)
			{
				driver::stop();
				return -1;
			}
			return 0;
		}

		void operation_finish(int priority) override
		{
		}

		int try_take_external_control_impl(external_controller * controller) override

		if (parent->is_active() == false) return -1;
		if (driver::in_operation()) return -1;
		return 0;
		}

		int try_release_external_control_impl(external_controller * controller) override
		{
		return 0;
		}

		external_control_slot * as_controlled() override { return this; }
		const char * name() { return mnemo(); }*/
	};


	/*template <class P, class V>
	class virtual_multiax :
		public external_controller,
		public control_served
	{
	public:
		virtual const char * name() = 0;

	protected:
		void on_activate_handle() override
		{
			restore_control_model();
		}

		int try_activate_impl() override
		{
			return take_control();
		}

		int try_deactivate_impl() override
		{
			return release_control();
		}

	protected:
		virtual void restore_control_model() = 0;
	};*/
}

#endif