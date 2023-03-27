#ifndef RALGO_CNC_UTIL_H
#define RALGO_CNC_UTIL_H

#include <ctype.h>
#include <string.h>

namespace cnc
{
    static inline int symbol_to_index(char c)
    {
        switch (tolower(c))
        {
        case 'x':
            return 0;
        case 'y':
            return 1;
        case 'z':
            return 2;
        case 'a':
            return 3;
        case 'b':
            return 4;
        case 'c':
            return 5;
        case 'i':
            return 6;
        case 'j':
            return 7;
        case 'k':
            return 8;
        }
        return -1;
    }

    static inline char index_to_symbol(int i)
    {
        switch (i)
        {
        case 0:
            return 'x';
        case 1:
            return 'y';
        case 2:
            return 'z';
        case 3:
            return 'a';
        case 4:
            return 'b';
        case 5:
            return 'c';
        case 6:
            return 'i';
        case 7:
            return 'j';
        case 8:
            return 'k';
        }
        return '?';
    }
}

#endif