#include <ralgo/log.h>
#include <igris/dprint.h>

void ralgo::debug(const char * str, const char * a, const char * b) 
{
	ralgo::log(RALGO_DEBUG, str, a, b);
}

void ralgo::info(const char * str, const char * a, const char * b) 
{
	ralgo::log(RALGO_INFO, str, a, b);
}

void ralgo::warn(const char * str, const char * a, const char * b) 
{
	ralgo::log(RALGO_WARN, str, a, b);
}

void ralgo::fault(const char * str, const char * a, const char * b) 
{
	ralgo::log(RALGO_FAULT, str, a, b);
}
