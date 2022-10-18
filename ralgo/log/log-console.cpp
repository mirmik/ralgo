#include <ralgo/log.h>
#include <string.h>
#include <string>
#include <unistd.h>

void ralgo::log(ralgo::LogLevel lvl,
                const char *a,
                const char *b,
                const char *c)
{
    std::string str;
    str += a;
    if (b)
    {
        str += " ";
        str += b;
    }
    if (c)
    {
        str += " ";
        str += c;
    }

    switch (lvl)
    {
    case RALGO_DEBUG:
        ralgo::logger->debug(str.c_str());
        break;
    case RALGO_INFO:
        ralgo::logger->info(str.c_str());
        break;
    case RALGO_WARN:
        ralgo::logger->warn(str.c_str());
        break;
    case RALGO_FAULT:
        ralgo::logger->error(str.c_str());
        break;
    }
}
