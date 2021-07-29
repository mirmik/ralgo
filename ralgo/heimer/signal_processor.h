/** file */

#ifndef RALGO_HEIMER_SIGNAL_PROCESSOR_H
#define RALGO_HEIMER_SIGNAL_PROCESSOR_H

#include <igris/datastruct/dlist.h>
#include <ralgo/heimer/signal.h>
#include <ralgo/disctime.h>

#define SIGNAL_PROCESSOR_RETURN_OK 0
#define SIGNAL_PROCESSOR_RETURN_NOT_ACTIVE 1
#define SIGNAL_PROCESSOR_RETURN_RUNTIME_ERROR 2

#define SIGNAL_PROCESSOR_NAME_MAX_LENGTH 8

namespace heimer
{
	class signal_processor
	{
	private:
		struct dlist_head list_lnk;
		char name[SIGNAL_PROCESSOR_NAME_MAX_LENGTH];
		const struct signal_processor_operations * ops;
		uint8_t active;

	public:
		/// feedback отвечает за движение сигнала слева направо.  physical ----> virtual
		virtual int feedback(disctime_t time) = 0;

		/// serve отвечает за движение сигнала справа налево. physical <---- virtual
		virtual int serve(disctime_t time) = 0;

		virtual int  command(int argc, char ** argv, char * output, int outmax) = 0;
		virtual void deinit() = 0;
		virtual signal_head * iterate_left(signal_head *) = 0;
		virtual signal_head * iterate_right(signal_head *) = 0;

		void init(const char * name);
		int activate();
		int deactivate();
	};

	int heimer_signal_processors_count();
	void signal_processors_list_reinit();
	int command_signal_processors(int argc, char ** argv, char * output, int maxsize);
}

#endif