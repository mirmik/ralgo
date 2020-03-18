#ifndef RALGO_HEIMER_TANDEM_H
#define RALGO_HEIMER_TANDEM_H

#include <ralgo/heimer/multiax.h>
#include <ralgo/heimer/control.h>

namespace ralgo
{
	namespace heimer
	{
		template <class P, class V, int N>
		class tandem_controller :
			public virtual_multiax<P, V>,
			public control_info_node
		{
		public:
			virtual_multiax_axis<P, V> axis[N];
			axis_driver<P, V>** _controlled_axes;
			double koeffs[N];

			tandem_controller(
			    const char * name,
			    igris::array_view<heimer::axis_driver<P, V>*> _controlled_axes,
			    igris::array_view<double> koeffs
			)
				:
				control_info_node(name, this, this, nullptr),
				_controlled_axes(_controlled_axes.data())
				//	koeffs(koeffs)
			{
				assert(_controlled_axes.size() == N);
				assert(koeffs[0] == 1);

				for (int i = 0; i < N; ++i)
				{
					char buf[10];
					sprintf(buf, "%d", i);
					this->koeffs[i] = koeffs[i];
					axis[i].init(buf, this, i);
				}
			}


			igris::array_view<axis_driver<P, V>*>
			controlled_axes()
			{
				return _controlled_axes;
			}

			void restore_control_model() override
			{
				P mainpos = _controlled_axes[0]->feedpos;
				V mainspd = _controlled_axes[0]->feedspd;

				axis[0].set_ctrphase(mainpos,
				                     mainspd);

				for (int i = 1; i < N; ++i)
				{
					axis[i].set_ctrphase(mainpos * (-koeffs[i]) + _controlled_axes[i]->feedpos,
					                     mainspd * (-koeffs[i]) + _controlled_axes[i]->feedspd);
				}
			}

			void update_state()
			{
				P mainpos = _controlled_axes[0]->feedpos;
				V mainspd = _controlled_axes[0]->feedspd;

				axis[0].feedpos = mainpos;
				axis[0].feedspd = mainspd;

				for (int i = 1; i < N; ++i)
				{
					axis[i].feedpos = mainpos * (-koeffs[i]) + _controlled_axes[i]->feedpos;
					axis[i].feedspd = mainspd * (-koeffs[i]) + _controlled_axes[i]->feedspd;
				}
			}

			void serve_impl()
			{
				for (int i = 0; i < N; ++i) axis[i].evaluate_ctrvars();

				_controlled_axes[0]->direct_control(axis[0].ctrpos, axis[0].ctrspd);

				for (int i = 1; i < N; ++i)
				{
					_controlled_axes[i]->direct_control(axis[0].ctrpos * (koeffs[i]) + axis[i].ctrpos,
					                                     axis[0].ctrspd * (koeffs[i]) + axis[i].ctrspd);
				}
			}


			void print_info() override
			{
				for (int i = 0; i < N; ++i)
					nos::fprintln("ax: {} pos: {}", axis[i].name(), axis[i].feedpos);
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

				for (unsigned int i = 0; i < N; ++i)
				{
					if (slt == _controlled_axes[i])
					{
						if (i == N - 1) return nullptr;
						else return _controlled_axes[i + 1];
					}
				}
				BUG();
			}

			const char * name() override { return mnemo(); }
		};
	}
}

#endif