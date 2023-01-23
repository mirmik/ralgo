#ifndef RALGO_GEOM3_LINE_H
#define RALGO_GEOM3_LINE_H

namespace ralgo
{
    namespace geom3
    {

        template <class T> class line
        {
        private:
            linalg::vec<T, 3> _direction;
            linalg::vec<T, 3> _normal_to_center;

        public:
            line(const linalg::vec<T, 3> &direction,
                 const linalg::vec<T, 3> &point)
                : _direction(direction),
                  _normal_to_center(linalg::cross(direction, point))
            {
            }
        };
    }
}

#endif