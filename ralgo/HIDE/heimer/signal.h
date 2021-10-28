/** @file */

#ifndef RALGO_HEIMER_SIGNAL_SIGNAL_H
#define RALGO_HEIMER_SIGNAL_SIGNAL_H

#define HEIMER_SIGNAL_WRONG_NODE_MODE -100

using handler_ptr = void (*)(int signal, void *source);

namespace heimer
{
    // extern std::pair<int, handler_ptr> signal_handlers;
    void send_signal(int signal, void *source);
}

#endif
