#ifndef HEIMER_CONTROL2_H
#define HEIMER_CONTROL2_H

#include <igris/shell/conscmd.h>
#include <igris/datastruct/dlist.h>

#define HEIM_ERR_IN_OPERATE  (-1)
#define HEIM_ERR_IS_BUSY     (-2)
#define HEIM_ERR_IS_NOACTIVE (-3)
#define HEIM_ERR_IS_CONTROL_FAULT (-4)
#define HEIM_ERR_IS_ALARM (-5)

#define HEIM_IS_ACTIVE      (1<<0)
#define HEIM_ALARM          (1<<1)
#define HEIM_MASTER         (1<<2)
#define HEIM_SLAVE          (1<<3)

namespace heimer
{
	class control_node
	{
	protected:
		const char * _mnemo;
		uint16_t flags = 0;

	public:
		constexpr 
		control_node(const char * mnemo) : _mnemo(mnemo) {}

		// обратное уведомления о событиях
		virtual int interrupt(
		    control_node * slave, // подчинённый, переславший сигнал
		    control_node * source, // источник сигнала
		    int code,
		    int subcode)
		{ return 0; }

		// итератор подчинённых устройств
		virtual control_node * iterate (control_node * it)
		{ return nullptr; }

		// отобразить информацию об устройстве.
		virtual void print_info()
		{}

		// Вызывается при после успешной активации устройства
		int activate();
		int deactivate();

		bool is_active() 
		{
			return flags & HEIM_IS_ACTIVE;
		}

	protected:
		// вызывается при взятии внешнего управления нодом
		virtual int on_take(
		    control_node * master)
		{ return 0; }

		// вызывается при отпускании внешнего управления
		virtual int on_release(
		    control_node * master)
		{ return 0; }

		virtual int on_activate() { return 0; }
		virtual int on_deactivate() { return 0; }
	};

	extern igris::console_command info_node_commands[];
}

#endif
