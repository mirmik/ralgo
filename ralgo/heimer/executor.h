#ifndef RALGO_HEIMER_EXECUTOR_H
#define RALGO_HEIMER_EXECUTOR_H

#include <ralgo/heimer/signal_processor.h>

namespace heimer 
{
	class executor 
	{
		signal_processor ** order_table = nullptr;
		int order_table_size = 0;
		int order_table_capacity = 0;

	public:
		void order_sort();
	};
}

#endif