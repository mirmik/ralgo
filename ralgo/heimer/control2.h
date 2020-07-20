#ifndef HEIMER_CONTROL2_H
#define HEIMER_CONTROL2_H

class control_node {
protected:
	const char * _mnemo;

public:
	// Вызывается при после успешной активации устройства
	virtual int activate(control_node_t * node) 
	{ return 0; }

	// Вызывается при попытке деактивировать устройство
	virtual int deactivate(control_node_t * node) 
	{ return 0; }

	// обратное уведомления о событиях
	virtual int interrupt(
		control_node_t * master, 
		control_node_t * slave, // подчинённый, переславший сигнал
		control_node_t * source, // источник сигнала
		int code, 
		int subcode) 
	{ return 0; }
	
	// итератор подчинённых устройств
	virtual int iterate (control_node_t * master, control_node_t * it) 
	{ return NULL; }

	// отобразить информацию об устройстве.
	virtual void print_info() 
	{} 

protected:
	// вызывается при взятии внешнего управления нодом
	virtual int on_take(
		control_node_t * slave, 
		control_node_t * master)
	{ return 0; }
	
	// вызывается при отпускании внешнего управления
	virtual int on_release(
		control_node_t * slave, 
		control_node_t * master)
	{ return 0; }
};

#endif
