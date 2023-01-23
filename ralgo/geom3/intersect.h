#ifndef RALGO_GEOM3_INTERSECT_H
#define RALGO_GEOM3_INTERSECT_H

#include <ralgo/geom3/plane.h>
#include <ralgo/geom3/ray.h>

namespace ralgo
{
    struct point_intersection_result
    {
        bool is_intersect;
        linalg::vec<float, 3> point;
    };

    namespace geom3
    {
        point_intersection_result intersect_triangle_and_ray(
            const std::array<linalg::vec<float, 3>, 3> &triangle,
            const ray<float> &ray)
        {
            plane<float> triangle_plane(triangle);
            float distance = triangle_plane.distance(ray.center());
            if (distance < 0)
            {
                return {false, {}};
            }
            linalg::vec<float, 3> intersection_point =
                ray.center() + ray.direction() * distance;
            linalg::vec<float, 3> v0 = triangle[1] - triangle[0];
            linalg::vec<float, 3> v1 = triangle[2] - triangle[0];
            linalg::vec<float, 3> v2 = intersection_point - triangle[0];
            float dot00 = linalg::dot(v0, v0);
            float dot01 = linalg::dot(v0, v1);
            float dot02 = linalg::dot(v0, v2);
            float dot11 = linalg::dot(v1, v1);
            float dot12 = linalg::dot(v1, v2);
            float inv_denom = 1 / (dot00 * dot11 - dot01 * dot01);
            float u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
            float v = (dot00 * dot12 - dot01 * dot02) * inv_denom;
            if (u >= 0 && v >= 0 && u + v < 1)
            {
                return {true, intersection_point};
            }
            else
            {
                return {false, {}};
            }
        }
    }
}

#endif