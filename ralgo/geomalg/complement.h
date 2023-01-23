#ifndef RALGO_GEOMALG_COMPLEMENT_H
#define RALGO_GEOMALG_COMPLEMENT_H

#include <ralgo/geomalg/bivector3d.h>
#include <ralgo/geomalg/bivector4d.h>
#include <ralgo/geomalg/trivector4d.h>
#include <ralgo/geomalg/vector2d.h>
#include <ralgo/geomalg/vector3d.h>
#include <ralgo/geomalg/vector4d.h>

namespace ralgo
{
    namespace geomalg
    {
        template <class T> const bivector3d<T> &complement(const vector3d<T> &v)
        {
            return reinterpret_cast<const bivector3d<T> &>(v);
        }

        template <class T> const vector3d<T> &complement(const bivector3d<T> &m)
        {
            return reinterpret_cast<const vector3d<T> &>(m);
        }
    }
}

#endif