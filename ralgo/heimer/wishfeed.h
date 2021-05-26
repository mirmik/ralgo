#ifndef RALGO_WISHFEED_H
#define RALGO_WISHFEED_H

#define SIGNAL_NAME_MAXSIZE 8
#define MAXIMUM_WISHFEED_SIGNAL_SIZE 4

#include <ralgo/heimer/types.h>
#include <assert.h>

namespace heimer
{
	class wishfeed
	{
		char _name[SIGNAL_NAME_MAXSIZE];
		int _dim;

		real _wish[MAXIMUM_WISHFEED_SIGNAL_SIZE];
		real _feed[MAXIMUM_WISHFEED_SIGNAL_SIZE];

	public:
		wishfeed(const char * name, int dim) : _wish{}, _feed{}
		{
			_dim = dim;
			set_name(name, strlen(name));
		}

		real * wish(real * wish)
		{
			return _wish;
		}

		real * feed(const phase & feed)
		{
			return feed;
		}

		template <class T>
		T& wish_() 
		{
			return *(T*) _wish;
		}

		template <class T>
		T& feed_() 
		{
			return *(T*) _feed;
		}
	};
}

#endif