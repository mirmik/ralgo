/** @file */

#ifndef RALGO_HEIMER_TYPES_H
#define RALGO_HEIMER_TYPES_H

#ifndef HEIMER_REAL_TYPE
#define HEIMER_REAL_TYPE float
#endif

#ifndef HEIMER_PHASE_TYPE
#define HEIMER_PHASE_TYPE 2
#endif

#include <ralgo/linalg/linalg.h>

namespace heimer
{
    using real = HEIMER_REAL_TYPE;
    const real epsilon = 1e-10;

    namespace detail
    {
        template <class type> struct __phase
        {
            type pos;
            type spd;

            __phase operator-(const __phase &oth) const
            {
                return {pos - oth.pos, spd - oth.spd};
            }
        };

        template <class type> struct __phase_with_acc
        {
            type pos;
            type spd;
            type acc;

            __phase_with_acc operator-(const __phase_with_acc &oth) const
            {
                return {pos - oth.pos, spd - oth.spd, acc - oth.acc};
            }
        };
    }

#if (HEIMER_PHASE_TYPE == 2)
    using phase = detail::__phase<real>;
    using phase2 = detail::__phase<linalg::vec<real, 2>>;
#else
    using phase = detail::__phase_with_acc<real>;
    using phase2 = detail::__phase_with_acc<linalg::vec<real, 2>>;
#endif
}

#endif
