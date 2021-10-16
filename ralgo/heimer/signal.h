#ifndef RALGO_HEIMER_SIGNAL_H
#define RALGO_HEIMER_SIGNAL_H

#include <igris/compiler.h>
#include <igris/datastruct/dlist.h>

#include <errno.h>
#include <ralgo/disctime.h>


#define SIGNAL_NAME_MAX_LENGTH 12

namespace heimer
{
	class signal_processor;
	extern struct dlist_head signals_list;

	class signal_head
	{
	public:
		struct dlist_head list_lnk;
		char name[SIGNAL_NAME_MAX_LENGTH];
		uint8_t type;

		/// Количество ссылок. Каждый контроллер, имеющий указатель
		/// на этот сигнал должен инкрементировать счётчик и сбросить при
		/// отпуске ссылки.
		int16_t refs;

		/// Процессор, который сейчас контолирует этот сигнал.
		/// Активируется при активации контроллера и отключается в ином случае.
		signal_processor * current_controller = nullptr;

		/// Процессор, который слушает этот сигнал.
		/// Устанавливается во время биндинга и потом не меняется.
		signal_processor * listener = nullptr;

		uint8_t sorting_mark; /// < Метка для алгоритма сортировки (см. executor)

	public:
		signal_head() = default;
		signal_head(uint8_t type);
		signal_head(const char * name, uint8_t type);
		void set_name(const char * name);

		virtual int info(char * buffer, int maxsize) = 0;
		int ctrinfo(char * buffer, int maxsize);

		void init(const char * name, uint8_t type);
		void rebind();
		void deinit();

		int attach_listener(signal_processor * );
		void attach_possible_controller(signal_processor * );
		int deattach_listener(signal_processor * );
		void deattach_possible_controller(signal_processor * );

		virtual int command_v(int argc, char ** argv, char * output, int maxsize);

		int activate(signal_processor * proc, disctime_t);
		int deactivate(signal_processor * proc, disctime_t);

		/// Распространить прерывание вверх по цепочке контроллеров.
		void provide_interrupt(disctime_t);
	};

	int signals_count();
	void signal_head_list_reinit();
	signal_head * signal_get_by_name(const char * name);
}


#endif
