/** @file */

#ifndef RALGO_HEIMER_SIGNAL_H
#define RALGO_HEIMER_SIGNAL_H

#include <ralgo/heimer/types.h>

namespace heimer
{
    struct servo_wishfeed
    {
        real wishpos;
        real wishspd;
        // real wishacc;

        real feedctr;
        real feedpos;
        real feedspd;
    };
}

#endif
