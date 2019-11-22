#ifndef RALGO_SERVED_H
#define RALGO_SERVED_H

namespace ralgo 
{
	class served 
	{
		virtual void serve() = 0;
		virtual void activate() = 0;
		virtual void deactivate() = 0;
	};
}

#endif 