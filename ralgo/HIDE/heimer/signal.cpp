/** @file */

#include <igris/dprint.h>
#include <ralgo/heimer/signal.h>

void heimer::send_signal(int signal, void *source)
{
    dprln("WARNING: %d", signal);
}
