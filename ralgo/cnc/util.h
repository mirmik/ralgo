#ifndef RALGO_CNC_UTIL_H
#define RALGO_CNC_UTIL_H

#include <string.h>
#include <ctype.h>

namespace cnc {
static inline 
int symbol_to_index(char c) 
{
	switch (tolower(c))
	{
		case 'x': return 0;
		case 'y': return 1;
		case 'z': return 2;
		case 'a': return 3;
		case 'b': return 4;
		case 'c': return 5;
		case 'i': return 6;
		case 'j': return 7;
		case 'k': return 8;
	}
	return -1;
}
}

#endif