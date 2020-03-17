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
			using driver = heimer::axis_driver<P,V>;
			virtual_multiax<P, V> * parent;
			int index;

			char _name[10];

		public:
			virtual_multiax_axis(const char* axname, virtual_multiax<P, V>* parent, int index) :
				control_info_node(_name, nullptr, nullptr, this),
				parent(parent), index(index)
			{
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
				BUG();
			}

			void operation_finish(int priority) override
			{
				BUG();
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
			virtual_multiax_axis<float, float> * axes;
			size_t axes_size;

			virtual_multiax(
			    virtual_multiax_axis<float, float> * axarr,
			    int arrsz) :
				axes(axarr),
				axes_size(arrsz) {}

			int try_operation_begin()
			{
				BUG();
			}

			void operation_finish()
			{
				BUG();

				/*int in_operation_state = 0;

				for (unsigned int i = 0; i < axes_size; ++i)
					if (axes[i].is_in_operation_state())
						in_operation_state++;

				if (in_operation_state == 0)
				{
					release_control();
				}*/
			}
			virtual const char * name() = 0;

		protected:
			void on_activate_handle() override
			{
				// Кто-то (возможно, сам) взял контроль.
				// восстанавливаем данные по положению и прочему.
				restore_control_model();

				//for (unsigned int i = 0; i < axes_size; ++i)
				//	axes[i].as_controlled()->set_controller_force(controller());
			}

		protected:
			virtual void restore_control_model() = 0;
		};
	}
}

#endif