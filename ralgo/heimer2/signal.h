#ifndef RALGO_HEIMER_SIGNAL_H
#define RALGO_HEIMER_SIGNAL_H

#include <igris/compiler.h>
#include <igris/datastruct/dlist.h>

#define SIGNAL_NAME_MAX_LENGTH 8

struct signal_head;

struct signal_head_operations 
{
	int (* info)(struct signal_head *, char * output, int maxsize); 
};

struct signal_head
{
	struct dlist_head signal_list_lnk;
	char name[SIGNAL_NAME_MAX_LENGTH];
	uint8_t type;

	const struct signal_head_operations * ops;

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
};

__BEGIN_DECLS

void signal_head_init(
	struct signal_head * sig, 
	const char * name,
	uint8_t type, 
	const struct signal_head_operations * ops
);

void signal_head_deinit(struct signal_head * sig);

void signal_head_get(struct signal_head * sig);
void signal_head_put(struct signal_head * sig);

int heimer_signals_count();

void signal_head_list_reinit();

struct signal_head * signal_get_by_name(const char * name);
int signal_command_v(struct signal_head * head, int argc, char ** argv, char * output, int maxsize);

int signal_head_activate(struct signal_head * sig, struct signal_processor * proc);
int signal_head_deactivate(struct signal_head * sig, struct signal_processor * proc);

__END_DECLS

#endif