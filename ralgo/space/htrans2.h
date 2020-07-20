#ifndef RABBIT_TRANS2_H
#define RABBIT_TRANS2_H

#include <ralgo/space/screw.h>
#include <ralgo/linalg/linalg.h>

#include <nos/fprint.h>

namespace ralgo
{
	using linalg::ostream_overloads::operator <<;

	template <typename T>
	struct htrans2
	{
		T orient;
		linalg::vec<T, 2> center;

		htrans2() : orient(0), center{0, 0} {}
		htrans2(T rotate_, linalg::vec<T, 2> translate_)
			: orient(rotate_), center(translate_)
		{}

		linalg::vec<T, 2> operator()(linalg::vec<T, 2> vec)
		{
			return linalg::rot(orient, vec) + center;
		};

		screw2<T> rotate_screw(screw2<T> scr)
		{
			return { scr.ang, linalg::rot(orient, scr.lin) };
		};

		htrans2 operator * (const htrans2& oth)
		{
			return htrans2(
			           orient + oth.orient,
			           linalg::rot(orient, oth.center) + center);
		}

		screw2<T> operator - (const htrans2<T>& oth)
		{
			return { orient - oth.orient,
			         center - oth.center
			       };
		}

		htrans2 integrate_speed(const screw2<T>& spd, T delta)
		{
			return *this + spd * delta;
		}

		htrans2 inverse()
		{
			return { -orient, linalg::rot(-orient, -center) };
		}

		linalg::vec<T,2> rotate(const linalg::vec<T,2>& v)
		{
			return linalg::rot(orient, v);
		}

		linalg::vec<T,2> inverse_rotate(const linalg::vec<T,2>& v)
		{
			return linalg::rot(-orient, v);
		}

		T rotation() { return orient; }

		linalg::vec<T, 2> translation() { return center; }

		ssize_t print_to(nos::ostream& out) const
		{
			return nos::fprint_to(out, "htrans({},{})", orient, center );
		}
	};

	template<class C, class T>
	std::basic_ostream<C> & operator << (
	    std::basic_ostream<C> & out,
	    const ralgo::htrans2<T> & tr)
	{
		return out << '{' << tr.orient << ',' << tr.center << '}';
	}
}

#endif