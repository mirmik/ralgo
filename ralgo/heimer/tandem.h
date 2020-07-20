#ifndef RALGO_HEIMER_TANDEM_H
#define RALGO_HEIMER_TANDEM_H

#include <ralgo/heimer/multiax.h>
#include <ralgo/heimer/control.h>

namespace heimer
{
	template <class P, class V, int N>
	class tandem : public control_node
	{
	public:
		virtual_axis_node<P, V> master;
		virtual_axis_node<P, V> slave;

		axis_node<P, V> * master_controlled;
		axis_node<P, V> * slave_controlled;

		float koeff;

		constexpr
		tandem(
		    const char * mnemo,
		    const char * master_mnemo,
		    const char * slave_mnemo,
		    axis_node<P, V> * master,
		    axis_node<P, V> * slave,
		    float             koeff
		) :
			control_node(mnemo),
			master(master_mnemo, this),
			slave(slave_mnemo, this),
			master_controlled(master),
			slave_controlled(slave),
			koeff(koeff)
		{}

		void feedback()
		{
			float mx = master_controlled->feedpos;
			float mv = master_controlled->feedspd;

			float sx = slave_controlled->feedpos;
			float sv = slave_controlled->feedspd;

			master.feedpos = mx;
			master.feedspd = mv;

			slave.feedpos = sx - koeff * mx;
			slave.feedspd = sv - koeff * mv;
		};

		void serve()
		{
			float mx = master_controlled->ctrpos;
			float mv = master_controlled->ctrspd;

			float sx = slave_controlled->ctrpos;
			float sv = slave_controlled->ctrspd;

			master.ctrpos = mx;
			master.ctrspd = mv;

			slave.ctrpos = sx + koeff * mx;
			slave.ctrspd = sv + koeff * mv;
		};

		control_node * iterate(control_node * slt) override
		{
			if (slt == nullptr)
				return master_controlled;

			if (slt == master_controlled)
				return slave_controlled;

			return nullptr;
		}
	};
}

#endif