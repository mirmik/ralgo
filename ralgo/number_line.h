#ifndef RALGO_NUMBER_LINE_H
#define RALGO_NUMBER_LINE_H

#include <vector>

namespace ralgo
{
    // for bounds = {a, b, c}
    //                0       a        1      b        2     c        3
    //         <--------------|---------------|--------------|------------>
    template <class T>
    size_t number_line_interval_for_point(std::vector<T> bounds, T coord)
    {
        auto it = std::lower_bound(bounds.begin(), bounds.end(), coord);
        return std::distance(bounds.begin(), it);
    }
}

#endif