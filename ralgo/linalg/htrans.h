#ifndef RALGO_LINALG_HTRANS_H
#define RALGO_LINALG_HTRANS_H

#include <linalg-v3/linalg.h>
#include <linalg-v3/linalg-ext.h>

#include <ralgo/linalg/uvec.h>

namespace ralgo
{
	using namespace ::linalg;
	using namespace ::linalg::ostream_overloads;

	template<typename T>
	struct htrans
	{
		quat<T> 	orient;
		vec<T, 3> 	center;

		htrans() : orient{0, 0, 0, 1}, center{0, 0, 0} {}
		htrans(const quat<T>& r, const vec<T, 3> m) : orient(r), center(m) {}
		htrans(const mat<T, 3, 3>& r, const vec<T, 3> m) : orient(rotation_quat(r)), center(m) {}

		htrans operator*(const htrans& oth) const
		{
			return htrans(orient * oth.orient, qrot(orient, oth.center) + center);
		}

		mat<T, 4, 4> matrix() const
		{
			return {{qxdir(orient), 0}, {qydir(orient), 0}, {qzdir(orient), 0}, {center, 1}};
		}

		vec<T, 6> vector6() const
		{
			auto angle = qangle(orient);
			auto rotvec = angle == 0 ? vec<T, 3> {0, 0, 0} : qaxis(orient) * angle;
			return { rotvec[0], rotvec[1], rotvec[2], center[0], center[1], center[2] };
		}

		static htrans rotation(uvec<T, 3> axis, T angle)
		{
			return htrans(rotation_quat(axis, angle), vec<T, 3>());
		}

		static htrans translation(vec<T, 3> axis)
		{
			return htrans(quat<T>(identity), axis);
		}

		vec<T, 3> transform_vector (const vec<T, 3>& v) const
		{
			return qrot(orient, v);
		}

		vec<T, 3> transform_point (const vec<T, 3>& v) const
		{
			return qrot(orient, v) + center;
		}

		auto vector6_to(const htrans& target) const
		{
			return (target * inverse()).vector6();
		}

		htrans inverse() const
		{
			auto q = conjugate(orient); return { q, qrot(q, -center) };
		}
	};



	namespace ostream_overloads
	{
		using linalg::ostream_overloads::operator <<;

		template<class C, class T>
		std::basic_ostream<C> & operator << (std::basic_ostream<C> & out, const htrans<T> & tr)
		{
			return out << '{' << tr.orient << ',' << tr.center << '}';
		}
	}

	//template<class C, class T> std::basic_ostream<C> & operator << (std::basic_ostream<C> & out, const bivec<T,3> & tr) { return out << '{' << tr.a << ',' << tr.b << '}'; }
}

#endif