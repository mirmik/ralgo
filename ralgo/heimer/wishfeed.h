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
		int _size;

		real _wish[MAXIMUM_WISHFEED_SIGNAL_SIZE];
		real _feed[MAXIMUM_WISHFEED_SIGNAL_SIZE];

	public:
		wishfeed(){}

		wishfeed(const char * name, int size) : _wish{}, _feed{}
		{
			_size = size;
			set_name(name);
		}

		void set_name(const char * name) 
		{
			strncpy(_name, name, SIGNAL_NAME_MAXSIZE);
		}

		void set_size(int size) 
		{
			_size = size;
		}

		std::string_view name()  
		{
			int len = strnlen(_name, SIGNAL_NAME_MAXSIZE);
			return std::string_view(_name, len);
		}

		real * wish()
		{
			return _wish;
		}

		real * feed()
		{
			return _feed;
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