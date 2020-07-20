#include <ralgo/log.h>
#include <stdlib.h>

namespace ralgo 
{
	void fault(const char* message) 
	{
		abort();
	}
}