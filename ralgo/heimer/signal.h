#ifndef RALGO_HEIMER_SIGNAL_H
#define RALGO_HEIMER_SIGNAL_H

#include <igris/compiler.h>
#include <igris/datastruct/dlist.h>

#define SIGNAL_NAME_MAX_LENGTH 8

class signal_head
{
public:
	struct dlist_head signal_list_lnk;
	char name[SIGNAL_NAME_MAX_LENGTH];
	uint8_t type;

	/// Количество ссылок. Каждый контроллер, имеющий указатель
	/// на этот сигнал должен инкрементировать счётчик и сбросить при
	/// отпуске ссылки.
	int16_t refs;

	/// Процессор, который сейчас контолирует этот сигнал.
	/// Активируется при активации контроллера и отключается в ином случае.
	struct signal_processor * current_controller;

	/// Процессор, который слушает этот сигнал.
	/// Устанавливается во время биндинга и потом не меняется.
	struct signal_processor * listener;

	/// Статус активации.
	/// Если статус установлен, это значит, что сигнал получает команды свыше.
	/// и ожидает деактивации в какой-то момент времени.
	uint8_t active;

public:
	virtual int info(char * buffer, int maxsize) = 0;

	void init(const char * name, uint8_t type);
	void deinit();

	void get();
	void put();

	int command_v(int argc, char ** argv, char * output, int maxsize);

	int activate(signal_processor * proc);
	int deactivate(signal_processor * proc);
};

__BEGIN_DECLS

int heimer_signals_count();
void signal_head_list_reinit();
struct signal_head * signal_get_by_name(const char * name);

__END_DECLS

#endif