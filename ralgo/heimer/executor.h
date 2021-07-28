#ifndef RALGO_HEIMER_EXECUTOR_H
#define RALGO_HEIMER_EXECUTOR_H

#include <ralgo/heimer/signal_processor.h>

namespace heimer 
{
	class executor 
	{
	public:
		signal_processor ** order_table = nullptr;
		int order_table_size = 0;
		int order_table_capacity = 0;

	public:
		void set_order_table(signal_processor ** order_table, int capacity, int size);
		void append_processor(signal_processor * proc);

		int order_sort();

		int serve(disctime_t curtime);
		int feedback(disctime_t curtime);
		int exec(disctime_t curtime);
	};
}

#endif