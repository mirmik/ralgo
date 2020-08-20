#ifndef RALGO_HEIMER_MULTIAX_H
#define RALGO_HEIMER_MULTIAX_H

#include <ralgo/heimer/axis.h>

namespace heimer
{
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

		int on_activate() override 
		{
			return HEIM_ERR_IS_PARTED;
		}
	};
}

#endif