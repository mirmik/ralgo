#ifndef RALGO_HEIMER_LINCOMBO_H
#define RALGO_HEIMER_LINCOMBO_H

#include <ralgo/heimer/multiax.h>
#include <ralgo/heimer/control.h>

namespace ralgo
{
	namespace heimer
	{
		template <class P, class V>
		class lincombo_controller : 
			public virtual_multiax<P, V>,
			public control_info_node
		{
		public:
			virtual_multiax_axis<P, V> axis;

			igris::array_view<axis_driver<P, V>*> _controlled_axes;
			igris::array_view<double> koeffs;
			igris::array_view<double> offsets;

			bool finish_inited = false;

			lincombo_controller(
			    const char * name,
			    igris::array_view<heimer::axis_driver<P, V>*> _controlled_axes,
			    igris::array_view<double> koeffs,
			    igris::array_view<double> offsets
			)
				:
				virtual_multiax<P, V>(&axis, 1),
				control_info_node(name, this, this, nullptr),
				axis("ax", this, 0),
				_controlled_axes(_controlled_axes),
				koeffs(koeffs),
				offsets(offsets)
			{
				assert(koeffs[0] == 1);
				memset(offsets.data(), 0, offsets.size() * sizeof(P));
			}

			//igris::array_view<controlled*>
			//controlled_devices() override
			//{
			//	return _controlled_devices;
			//}

			igris::array_view<axis_driver<P, V>*>
			controlled_axes()
			{
				return _controlled_axes;
			}

			void restore_control_model() override
			{
				finish_inited = false;
				for (unsigned int i = 0; i < _controlled_axes.size(); ++i)
				{
					offsets[i] =
					    _controlled_axes[i]->current_position() * koeffs[i]
					    - _controlled_axes[0]->current_position();
				}

				axis._set_point_trajectory(
				    _controlled_axes[0]->current_position());
			}

			void update_state()
			{
				axis.feedpos = _controlled_axes[0]->current_position();

				if (finish_inited) 
				{
					BUG();
					//device::release_control_self();
					//axis.set_controller_force(nullptr);
					finish_inited = false;
				}
			}

			void serve_impl()
			{
				P pos;
				V spd;
//
//				if (heimer::device::controller())
//				{
					finish_inited = axis.attime(ralgo::discrete_time(), pos, spd);

					for (unsigned int i = 0; i < _controlled_axes.size(); ++i)
					{
						_controlled_axes[i]->direct_control(pos * koeffs[i] + offsets[i], spd * koeffs[i]);
					}
//				}
			}


			void print_info() override
			{
			//	nos::fprintln("device: {}", device::name());
				nos::fprintln("pos: {}", axis.feedpos);				
			}

			void on_activate_handle() override
			{
				restore_control_model();
			}

			void on_deactivate_handle() override
			{}

			void control_interrupt_from(external_control_slot * slot)  override
			{
				BUG();
			}

			external_control_slot* iterate(external_control_slot* slt) override
			{
				if (slt == nullptr) 
					return _controlled_axes[0]->as_controlled();
				
				for (unsigned int i = 0; i < _controlled_axes.size(); ++i) 
				{
					if (slt == _controlled_axes[i]) 
					{
						if (i == _controlled_axes.size() - 1) return nullptr;
						else return _controlled_axes[i+1];
					}
				}
				BUG();
			}

			const char * name() override { return mnemo(); }
		};
	}
}

#endif