#include <ralgo/fault.h>
#include <stdexcept>

namespace ralgo 
{
	void fault(const char* message) 
	{
		throw std::runtime_error(message);
	}
}