/** @file */

#ifndef RALGO_WISHFEED_H
#define RALGO_WISHFEED_H

#define SIGNAL_NAME_MAXSIZE 16
#define MAXIMUM_WISHFEED_SIGNAL_SIZE 4

#include <assert.h>
#include <ralgo/heimer/types.h>

namespace heimer
{
    class wishfeed_node;

    class wishfeed
    {
        char _name[SIGNAL_NAME_MAXSIZE];
        int _size = 0;

        real _wish[MAXIMUM_WISHFEED_SIGNAL_SIZE];
        real _feed[MAXIMUM_WISHFEED_SIGNAL_SIZE];

    public:
        int _pos = -1;
        wishfeed_node *left = nullptr;
        wishfeed_node *right = nullptr;

    public:
        wishfeed()
        {
            memset(_name, 0, sizeof(_name));
            memset(_wish, 0, sizeof(_wish));
            memset(_feed, 0, sizeof(_feed));
        }

        wishfeed(const char *name, int size) : _wish{}, _feed{}
        {
            _size = size;
            set_name(name);
            memset(_wish, 0, sizeof(_wish));
            memset(_feed, 0, sizeof(_feed));
        }

        void set_name(const char *name)
        {
            strncpy(_name, name, SIGNAL_NAME_MAXSIZE);
        }

        void set_size(int size) { _size = size; }

        int size() { return _size; }

        std::string_view name()
        {
            int len = strnlen(_name, SIGNAL_NAME_MAXSIZE);
            return std::string_view(_name, len);
        }

        real *wish() { return _wish; }

        real *feed() { return _feed; }

        template <class T> T &wish_() { return *(T *)_wish; }

        template <class T> T &feed_() { return *(T *)_feed; }
    };
}

#endif
