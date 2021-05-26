#ifndef RALGO_WISHFEED_H
#define RALGO_WISHFEED_H

#define SIGNAL_NAME_MAXSIZE 8

#include <ralgo/heimer/types.h>
#include <assert.h>

namespace heimer
{
	class wishfeed
	{
		char _name[SIGNAL_NAME_MAXSIZE];

		union
		{
			phase  phs;
			phase2 phs2;
		} _wish, _feed;

		int _dim;

	public:
		wishfeed(const char * name, int dim) : _wish{}, _feed{}
		{
			_dim = dim;
			set_name(name, strlen(name));
		}

		void set_name(const char * name, int len)
		{
			strncpy(_name, name, len < SIGNAL_NAME_MAXSIZE ? len : SIGNAL_NAME_MAXSIZE);
		}

		phase error_dim1()
		{
			assert(_dim == 1);
			return _wish.phs - _feed.phs;
		}

		phase2 error_dim2()
		{
			assert(_dim == 2);
			return _wish.phs2 - _feed.phs2;
		}

		const phase & wish_dim1()
		{
			assert(_dim == 1);
			return _wish.phs;
		}

		const phase2 & feed_dim2()
		{
			assert(_dim == 2);
			return _feed.phs2;
		}

		void set_wish(const phase & wish)
		{
			assert(_dim == 1);
			_wish.phs = wish;
		}

		void set_wish(const phase2 & wish)
		{
			assert(_dim == 2);
			_wish.phs2 = wish;
		}

		void set_feed(const phase & feed)
		{
			assert(_dim == 1);
			_feed = feed;
		}

		void set_feed(const phase2 & feed)
		{
			assert(_dim == 2);
			_feed.phs2 = feed;
		}
	};
}

#endif