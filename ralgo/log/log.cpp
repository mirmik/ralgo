#include <ralgo/log.h>
#include <igris/dprint.h>

void ralgo::debug(const char * str) 
{
	ralgo::log(RALGO_DEBUG, str);
}

void ralgo::info(const char * str) 
{
	ralgo::log(RALGO_INFO, str);
}

void ralgo::warn(const char * str) 
{
	ralgo::log(RALGO_WARN, str);
}

void ralgo::fault(const char * str) 
{
	ralgo::log(RALGO_FAULT, str);
}
