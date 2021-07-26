#ifndef RALGO_HEIMER_AXISCTR_APPROVAL_H
#define RALGO_HEIMER_AXISCTR_APPROVAL_H

#include <igris/compiler.h>
#include <ralgo/heimer2/heimer_types.h>

struct axisctr;

struct axisctr_approval 
{
	int (* check) (
		struct axisctr_approval * approval, 
		struct axisctr * ctr, 
		int dim,
		position_t * strt, 
		position_t * fini
	);
};

#endif