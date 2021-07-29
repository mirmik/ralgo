#include <ralgo/log.h>

#include <string.h>
#include <unistd.h>

void ralgo::log(ralgo::LogLevel lvl, 
	const char * a,
	const char * b,
	const char * c
) 
{
	(void) lvl;

	write(STDOUT_FILENO, a, strlen(a));
	
	if (b)
		write(STDOUT_FILENO, b, strlen(b));
	
	if (c)
		write(STDOUT_FILENO, c, strlen(c));
	
	write(STDOUT_FILENO, "\r\n", 2);
}