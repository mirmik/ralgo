#ifndef RALGO_CARTESIAN_GRID_H
#define RALGO_CARTESIAN_GRID_H

#include <ralgo/linalg/vector.h>
#include <ralgo/number_line.h>

namespace ralgo
{
    template <class T> class cartesian_grid
    {
        std::vector<std::vector<T>> coords;

    public:
        cartesian_grid(std::vector<std::vector<T>> coords) : coords(coords) {}
        cartesian_grid(const cartesian_grid &oth) : coords(oth.coords) {}
        cartesian_grid(cartesian_grid &&oth) : coords(std::move(oth.coords)) {}
        cartesian_grid &operator=(const cartesian_grid &oth)
        {
            coords = oth.coords;
            return *this;
        }
        cartesian_grid &operator=(cartesian_grid &&oth)
        {
            coords = std::move(oth.coords);
            return *this;
        }

        std::vector<T> cell_for_point(const ralgo::vector<T> &pnt)
        {
            std::vector<T> ret(pnt.size());
            for (size_t i = 0; i < pnt.size(); ++i)
            {
                auto &dimension_coords = coords[i];
                auto pnt_dimension_project = pnt[i];
                ret[i] = ralgo::number_line_interval_for_point(
                    dimension_coords, pnt_dimension_project);
            }
            return ret;
        }
    };
}

#endif