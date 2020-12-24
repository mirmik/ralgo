#ifndef RALGO_HTRANS_H
#define RALGO_HTRANS_H

#include <ralgo/space/htrans2.h>

namespace ralgo 
{
	template <class T> htrans2 rotate(T ang) 
	{
		return htrans2<T>(ang, {0,0});
	};

	template <class T> htrans2 translate(linalg::vec<T,2> vec) 
	{
		return htrans2<T>(0, vec);
	};
}

#endif