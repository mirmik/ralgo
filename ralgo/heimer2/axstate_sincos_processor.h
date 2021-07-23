/** @file */

#ifndef RALGO_HEIMER_AXSTATE_SINCOS_PROCESSOR_H
#define RALGO_HEIMER_AXSTATE_SINCOS_PROCESSOR_H

#include <igris/compiler.h>
#include <ralgo/heimer2/signal_processor.h>
#include <ralgo/heimer2/axis_state.h>

/**
Данный контроллер реализует управление
кинематической цепью из двух ортагональных актуаторов и
поворотного звена, перенося управляющие оси в точку выходного
звена.

\code
	Аппаратные оси (левые):
	
	^ yl  	  al           (направление оси al - на нас)
	|	 |-----O----x 
	|	 | 
	|	 |
	|	 |
	|	 |
		---
    ================= ---> xl

	Виртуальные оси (правые):

					^ yr
					|
					|
		 |-----O----x ----> xr
 		 |          ar           (направление оси ar - на нас) 	
	     |
		 |
		 |
	 	---
	=================
\endcode

xr = x_offset + xl + radius * cos(al)
yr = y_offset + yl + radius * sin(al)
ar = al

xl = xr - x_offset - radius * cos(ar)
yl = yr - y_offset - radius * sin(ar)
al = ar
*/
struct axstate_sincos_processor 
{
	struct signal_processor proc;

	struct axis_state ** leftside;   // order: x y a
	struct axis_state ** rightside;  // order: x y a

	position_t radius;

	position_t x_offset;
	position_t y_offset;
};

__BEGIN_DECLS

void axstate_sincos_processor_init(
	struct axstate_sincos_processor * scproc, 
	const char* name,
	struct axis_state ** leftside,
	struct axis_state ** rightside,
	position_t radius,
	position_t x_offset,
	position_t y_offset
); 

__END_DECLS

#endif