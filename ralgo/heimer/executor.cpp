#include <ralgo/heimer/executor.h>
#include <ralgo/heimer/signal.h>
#include <ralgo/heimer/axis_state.h>
#include <ralgo/heimer/sigtypes.h>
#include <ralgo/log.h>

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
	dlist_for_each_entry(it, &signals_list, list_lnk)
	{
		it->sorting_mark = 0;
		if (it->listener == nullptr)
		{
			ralgo::warn("signal without listener. name: ", it->name);
			return -1;
		}
	}

	for (int i = 0; i < order_table_size; ++i)
	{
		int j;
		signal_processor * sigproc = order_table[i];

		for (j = i; j < order_table_size; ++j)
		{
			sigproc = order_table[j];

			int has_unordered = 0;

			signal_head * h = nullptr;
			while ((h = sigproc->iterate_left(h)))
			{
				if (h->sorting_mark == 0)
				{
					has_unordered++;
				}
			}

			if (has_unordered == 0)
			{
				break;
			}
		}

		if (j == order_table_size)
			break;

		order_table[j] = order_table[i];
		order_table[i] = sigproc;

		signal_head * h = nullptr;
		while ((h = sigproc->iterate_right(h)))
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

int heimer::executor::serve(disctime_t curtime)
{
	int retcode;

	for (int i = order_table_size - 1; i >= 0; --i)
	{
		if (order_table[i]->need_activation() && !order_table[i]->is_active())
			continue;

		// serve исполняется только если контроллер включен.
		retcode = order_table[i]->serve(curtime);

		switch (retcode)
		{
			case SIGNAL_PROCESSOR_RETURN_OK: break;
			case SIGNAL_PROCESSOR_RETURN_NOT_ACTIVE: break;
			case SIGNAL_PROCESSOR_RETURN_RUNTIME_ERROR: return SIGNAL_PROCESSOR_RETURN_RUNTIME_ERROR;
			default: return SIGNAL_PROCESSOR_RETURN_RUNTIME_ERROR;
		}
	}

	return 0;
}

int heimer::executor::feedback(disctime_t curtime)
{
	int retcode;

	for (int i = 0; i < order_table_size; ++i)
	{
		retcode = order_table[i]->feedback(curtime);

		switch (retcode)
		{
			case SIGNAL_PROCESSOR_RETURN_OK: break;
			case SIGNAL_PROCESSOR_RETURN_NOT_ACTIVE: break;
			case SIGNAL_PROCESSOR_RETURN_RUNTIME_ERROR: return SIGNAL_PROCESSOR_RETURN_RUNTIME_ERROR;
			default: return SIGNAL_PROCESSOR_RETURN_RUNTIME_ERROR;
		}
	}

	return 0;
}

int heimer::executor::exec(disctime_t curtime)
{
	char buf[16];
	int retcode;

	retcode = feedback(curtime);
	if (retcode)
	{
		sprintf(buf, "%d", retcode);
		ralgo::warn("executor: feedback set retcode ", buf);
	}

	retcode = serve(curtime);
	if (retcode)
	{
		sprintf(buf, "%d", retcode);
		ralgo::warn("executor: feedback set retcode ", buf);
	}

	return retcode;
}

heimer::executor::~executor()
{
	if (f.dynamic)
		delete[] order_table;
}

void heimer::executor::allocate_order_table(int size)
{
	order_table = new signal_processor * [size];
	order_table_size = 0;
	order_table_capacity = size;
	f.dynamic = 1;
}

#if HEIMER_CROW_SUPPORT
void heimer::executor::notification_prepare(const char * theme, crow::hostaddr_view addrview)
{
	count_of_axstates = 0;

	signal_head * sig;
	dlist_for_each_entry(sig, &signals_list, list_lnk)
	{
		if (sig->type == SIGNAL_TYPE_AXIS_STATE)
		{
			++count_of_axstates;
		}
	}

	coordinate_publisher.init(addrview, theme, 0, 50);
}

void heimer::executor::notify()
{
	float arr[count_of_axstates];
	float * it = arr;

	signal_head * sig;
	dlist_for_each_entry(sig, &signals_list, list_lnk)
	{
		if (sig->type == SIGNAL_TYPE_AXIS_STATE)
		{
			*it++ = static_cast<heimer::axis_state*>(sig)->feedpos;
		}
	}

	coordinate_publisher.publish({(void*)arr, sizeof(arr)});
}
#endif