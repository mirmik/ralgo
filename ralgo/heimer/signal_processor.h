/** file */

#ifndef RALGO_HEIMER_SIGNAL_PROCESSOR_H
#define RALGO_HEIMER_SIGNAL_PROCESSOR_H

#include <igris/datastruct/dlist.h>
#include <igris/buffer.h>

#include <ralgo/heimer/signal.h>
#include <ralgo/disctime.h>

#define SIGNAL_PROCESSOR_RETURN_OK 0
#define SIGNAL_PROCESSOR_RETURN_NOT_ACTIVE 1
#define SIGNAL_PROCESSOR_RETURN_RUNTIME_ERROR 2

#define SIGNAL_PROCESSOR_NAME_MAX_LENGTH 12

namespace heimer
{
	extern struct dlist_head signal_processor_list;

	class signal_processor
	{
	public:
		struct dlist_head list_lnk;

	private:
		char _name[SIGNAL_PROCESSOR_NAME_MAX_LENGTH];

		union
		{
			uint8_t flags;
			struct
			{
				uint8_t active : 1;
				uint8_t need_activation : 1;
				uint8_t dynamic_resources : 1;
			} f;
		};

	public:
		signal_processor() = default;
		signal_processor(const char * name);

		/// feedback отвечает за движение сигнала слева направо.  physical ----> virtual
		virtual int feedback(disctime_t time) = 0;

		/// serve отвечает за движение сигнала справа налево. physical <---- virtual
		virtual int serve(disctime_t time) = 0;

		virtual int  command(int argc, char ** argv, char * output, int outmax) = 0;
		virtual void deinit() = 0;
		virtual signal_head * iterate_left(signal_head *) = 0;
		virtual signal_head * iterate_right(signal_head *) = 0;

		virtual void on_activate(disctime_t);

		void init(const char * name);

		int activate(disctime_t);
		int deactivate();

		igris::buffer name();
		bool is_active();
		bool need_activation();
		void set_need_activation(bool en);

		bool is_dynamic_resources();
		void set_dynamic_resources_flag(bool en);

		void release_signals();
	};

	int signal_processors_count();
	void signal_processors_list_reinit();
	int command_signal_processors(int argc, char ** argv, char * output, int maxsize);

	signal_processor * signal_processor_get_by_name(const char * name);
}

#endif