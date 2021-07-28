#include <ralgo/heimer/executor.h>
#include <ralgo/heimer/signal.h>

#include <igris/dprint.h>

void heimer::executor::set_order_table(signal_processor ** order_table, int capacity, int size)
{
	this->order_table = order_table;
	this->order_table_capacity = capacity;
	this->order_table_size = size;
}


int heimer::executor::order_sort()
{
	signal_head * it;
	dlist_for_each_entry(it, &signals_list, signal_list_lnk)
	{
		it->sorting_mark = it->listener == nullptr;
	}

	for (int i = 0; i < order_table_size; ++i)
	{
		int j;
		signal_processor * sigproc;

		for (j = i; j < order_table_size; ++j)
		{
			sigproc = order_table[j];

			int has_unordered = 0;

			signal_head * h = nullptr;
			while ((h = sigproc->iterate_left(h)))
			{
				if (h->sorting_mark == 0) 
				{
					has_unordered = 1;
				}
			}

			if (has_unordered == 0) 
			{
				break;
			}
		}

		if (j == order_table_size)
			return -1;

		order_table[j] = order_table[i];
		order_table[i] = sigproc;

		signal_head * h = nullptr;
		while ((h = sigproc->iterate_left(h)))
		{
			h->sorting_mark = 1;
		}

	}

	return 0;
}

void heimer::executor::append_processor(signal_processor * proc)
{
	if (order_table_size >= order_table_capacity)
		return;

	order_table[order_table_size++] = proc;
}