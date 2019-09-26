#include <ralgo/fault.h>
#include <stdlib.h>

namespace ralgo 
{
	void fault(const char* message) 
	{
		abort();
	}
}