#ifndef RALGO_FLOW_WINDOW_H
#define RALGO_FLOW_WINDOW_H

#include <assert.h>
#include <string.h>

#include <igris/container/array_view.h>

// Массив для реализации алгоритмов скользящего окна.
//
// Описание алгоритма.
// Массив состоит из двух смежных (последовательно расположенных в памяти) буфферов,
// каждый из которых имеет размер окна. Новые данные пишутся в каждый буффер, то есть записываются дважды.
// Окно скользит по пространству обоих буфферов, постепенно переползая из первого буффера во второй.
// По достижению конца второго буфера, окно смещается к началу первого (в позицию begin+1).
// При этом, данные в окне выглядят так, как будто оно продолжает ползти по данным последовательно.

namespace ralgo
{
	template <class T>
	class sliding_array
	{
	public:
		size_t halfsize;
		T* dataarr;
		int cursor;

	public:
		sliding_array(size_t size) : cursor(0)
		{
			allocate_buffer(size);
		}

		void allocate_buffer(size_t halfsize)
		{
			this->halfsize = halfsize;
			dataarr = new T[halfsize * 2];
			memset(dataarr, 0, halfsize * 2 * sizeof(T));
		}

		igris::array_view<double> window()
		{
			return igris::array_view<double>(dataarr + cursor, halfsize);
		}

		void _push_part(T* dt, size_t sz)
		{
			T* ptr1 = dataarr + cursor;
			T* ptr2 = dataarr + halfsize + cursor;

			std::copy(dt, dt + sz, ptr1);
			std::copy(dt, dt + sz, ptr2);
		}

		size_t right_room()
		{
			return halfsize - cursor;
		}

		void push(T* dt, size_t sz)
		{
			assert(sz <= halfsize);

			size_t rroom = right_room();
			size_t d0_size = sz > rroom < 0 ? sz - rroom : sz;
			size_t d1_size = sz - d0_size;

			_push_part(dt, d0_size);
			cursor = 0;
			_push_part(dt + d0_size, d1_size);
		}
	};
}

#endif