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

	void log(LogLevel lvl, const char *, const char * = nullptr, const char * = nullptr);

	void debug(const char *, const char * = nullptr, const char * = nullptr);
	void info(const char *, const char * = nullptr, const char * = nullptr);
	void warn(const char *, const char * = nullptr, const char * = nullptr);
	void fault(const char *, const char * = nullptr, const char * = nullptr);
} 

#endif