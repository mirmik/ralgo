#ifndef RALGO_INFO_H
#define RALGO_INFO_H

namespace ralgo
{
	enum LogLevel 
	{
		RALGO_DEBUG,
		RALGO_INFO,
		RALGO_WARN,
		RALGO_FAULT,
	};

	void log(LogLevel lvl, const char *);

	void debug(const char *);
	void info(const char *);
	void warn(const char *);
	void fault(const char *);
} 

#endif