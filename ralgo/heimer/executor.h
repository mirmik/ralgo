#ifndef RALGO_HEIMER_EXECUTOR_H
#define RALGO_HEIMER_EXECUTOR_H

#include <ralgo/heimer/signal_processor.h>

#if HEIMER_CROW_SUPPORT_ENABLED
#include <crow/pubsub/publisher.h>
#endif

namespace heimer 
{
	class executor 
	{
	public:
		signal_processor ** order_table = nullptr;
		int order_table_size = 0;
		int order_table_capacity = 0;

		union 
		{
			uint8_t flags;
			struct 
			{
				uint8_t dynamic;
			} f;
		};

#if HEIMER_CROW_SUPPORT_ENABLED
		crow::publisher coordinate_publisher;
#endif

	public:
		void set_order_table(signal_processor ** order_table, int capacity, int size);

		void allocate_order_table(int size);
		void append_processor(signal_processor * proc);

		int order_sort();

		int serve(disctime_t curtime);
		int feedback(disctime_t curtime);
		int exec(disctime_t curtime);

		~executor();
	};
}

#endif