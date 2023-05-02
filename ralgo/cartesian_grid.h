#ifndef RALGO_CARTESIAN_GRID_H
#define RALGO_CARTESIAN_GRID_H

#include <ralgo/linalg/vector.h>
#include <vector>

namespace ralgo
{

    // for bounds = {a, b, c}
    //                0       a        1      b        2     c        3
    //         <--------------|---------------|--------------|------------>
    template <class T>
    size_t
    number_of_cartesian_grid_interval_for_line_point(std::vector<T> bounds,
                                                     T coord)
    {
        auto it = std::lower_bound(bounds.begin(), bounds.end(), coord);
        return std::distance(bounds.begin(), it);
    }

    template <class T, class Coords = std::vector<T>>
    std::vector<size_t> number_of_cartesian_grid_interval_for_ndim_point(
        const std::vector<std::vector<T>> &bounds, const Coords &coord)
    {
        std::vector<size_t> result(bounds.size());
        for (size_t i = 0; i < coord.size(); i++)
        {
            result[i] = number_of_cartesian_grid_interval_for_line_point(
                bounds[i], coord[i]);
        }
        return result;
    }

}

#endif