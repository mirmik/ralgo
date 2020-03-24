#ifndef RALGO_GEN_H
#define RALGO_GEN_H

namespace ralgo 
{
	namespace gen 
	{
		class range_iterator 
		{
			int val;
			int step;

		public:
			range_iterator(int val, int step) : val(val), step(step) {}
			range_iterator operator++() { val += step; return *this; }
			range_iterator operator++(int i) { range_iterator ret = *this; ++(*this); return ret; }

			int operator*() { return val; }
			bool operator!=(const range_iterator& oth) { return val != oth.val; }
		};

		class range 
		{
			int _start;
			int _size;
			int _step;

		public:
			using value_type = int;

			range(int start, int stop, int step) : _start(start), _step(step) { _size = (stop - start) / step; }	
			range(int start, int stop) : range(start, stop, 1) {}
			range(int stop) : range(0, stop, 1) {}

			int operator[] (int n) const { return _start + _step * n; }

			range_iterator begin() const { return range_iterator(_start, _step); }
			const range_iterator end() const { return range_iterator(_start + _step * _size, _step); }
		
			int size() const { return _size; }
		};
	}
}

#endif