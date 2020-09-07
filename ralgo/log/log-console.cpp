#include <ralgo/log.h>

#include <string.h>
#include <unistd.h>

void ralgo::log(ralgo::LogLevel lvl, const char * data) 
{
	write(STDOUT_FILENO, data, strlen(data));
}