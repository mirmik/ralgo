#ifndef RALGO_INOUT_H
#define RALGO_INOUT_H

namespace ralgo
{
	template <class V> struct inout
	{
		virtual V operator()(V in) = 0;
		virtual void print_internal() {
			//nos::println("TODO");
		};
	};
}

#endif