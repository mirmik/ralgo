#ifndef RALGO_HEIMER_MULTIAX_H
#define RALGO_HEIMER_MULTIAX_H

#include <ralgo/heimer/control.h>

namespace ralgo
{
	namespace heimer
	{
		template <class P, class V> class virtual_multiax;

		template <class P, class V>
		class virtual_multiax_axis :
			public heimer::axis_driver<P, V>,
			public control_info_node
		{
			using driver = heimer::axis_driver<P, V>;
			virtual_multiax<P, V> * parent=nullptr;
			int index=0;

			char _name[10];

		public:
			virtual_multiax_axis() : control_info_node(_name, nullptr, nullptr, this) {}

			virtual_multiax_axis(const char* axname, virtual_multiax<P, V>* parent, int index) :
				virtual_multiax_axis()
			{
				init(axname, parent, index);
			}

			void init(const char* axname, virtual_multiax<P, V>* parent, int index)
			{
				this->parent = parent;
				this->index = index;

				strcpy(_name, parent->name());
				strcat(_name, "_");
				strcat(_name, axname);
			}

			V current_speed() override
			{
				BUG();
			}

			int try_operation_begin(int priority) override
			{
				if (!parent->is_active()) return -1;
				if (driver::in_operation()) return -1;
				if (driver::is_extern_controlled()) return -1;
				return 0;
			}

			void operation_finish(int priority) override
			{
			}

			int try_take_external_control_impl(external_controller * controller) override
			{
				if (parent->is_active() == false) return -1;
				if (driver::in_operation()) return -1;
				return 0;
			}

			int try_release_external_control_impl(external_controller * controller) override
			{
				return 0;
			}

			external_control_slot * as_controlled() override { return this; }
			const char * name() { return mnemo(); }
		};


		template <class P, class V>
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
		};
	}
}

#endif