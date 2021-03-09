#ifndef DRIVER_CONTROLLER_H
#define DRIVER_CONTROLLER_H

namespace ralgo 
{
	class driver_controller_api
	{
	public:
		virtual void update_control() = 0;
		virtual int stop() = 0;
		virtual int hardstop() = 0;
	};
}

#endif