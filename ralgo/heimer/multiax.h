#ifndef RALGO_HEIMER_MULTIAX_H
#define RALGO_HEIMER_MULTIAX_H

namespace ralgo
{
	namespace heimer
	{
		template <class P, class V> class virtual_multiax;

		template <class P, class V>
		class virtual_multiax_axis : public heimer::axis_driver<P, V>, public heimer::controlled
		{
			virtual_multiax<P, V> * parent;
			int index;

		public:
			virtual_multiax_axis(virtual_multiax<P, V>* parent, int index) :
				parent(parent), index(index)
			{}

			V current_speed() override
			{
				BUG();
			}

			bool try_operation_begin(int priority) override
			{
				return parent->try_operation_begin();
			}

			void operation_finish(int priority) override
			{
				parent->operation_finish();
			}

			heimer::controlled* as_controlled() { return this; }

			bool take_control(device * dev) override { return parent->take_control(dev); }
			bool take_control_force(device * dev) override { return parent->take_control_force(dev); }
			bool release_control(device * dev) override { return parent->release_control(dev); }
			void release_control_force(device * dev) override { parent->release_control_force(dev); }

			const char* name() { BUG(); }
		};


		template <class P, class V>
		class virtual_multiax : public heimer::device
		{
		public:
			axis_driver<float, float> * axes;
			P* poses;
			size_t axes_size;

			virtual_multiax(
				const char* name,
			    axis_driver<float, float> * axarr,
			    P* poses,
			    int arrsz) :
				device(name),
				axes(axarr),
				poses(poses),
				axes_size(arrsz) {}

			bool try_operation_begin()
			{
				if (controller())
					if (controller() != this)
						return CONTROL_ERROR;
					else
						return CONTROL_SUCCESS;
				else
				{
					return take_control();
				}
			}

			void operation_finish()
			{
				int in_operation_state = 0;

				for (unsigned int i = 0; i < axes_size; ++i)
					if (axes[i].is_in_operation_state())
						in_operation_state++;

				if (in_operation_state == 0)
					release_control();
			}

		protected:
			void after_take_control_handle() override 
			{
				// Кто-то (возможно, сам) взял контроль.
				// восстанавливаем данные по положению и прочему. 
				restore_control_model();
			}

		protected:
			virtual void restore_control_model() = 0; 
		};
	}
}

#endif