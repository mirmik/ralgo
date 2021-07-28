/** @file */

#ifndef RALGO_HEIMER_H
#define RALGO_HEIMER_H

#include <igris/compiler.h>
#include <ralgo/heimer/signal_processor.h>
#include <ralgo/heimer/axis_state.h>

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

Промежуточные координаты.
	Помимо основного преобразования контролер смещает сетку кооррдинат.
	Смещение осуществляется до и после преобразованияю.
	Смещение по осям xy линейно, поэтому выполняется один раз (Учитывается непосредственно в формуле).
	Смещение по осям a состоит из двух компонент - левого и правого смещения (Применяется до
	и после преобразования).

Прямое преобразование: physical <----- virtual

a = ar + aoff_right
xl = xr + xoff + radius * cos(a * ascale)
yl = yr + yoff + radius * sin(a * ascale)
al = a + aoff_left

dxl/dt = dxr/dt - radius * sin(a * ascale) * dar/dt * ascale
dyl/dt = dyr/dt + radius * cos(a * ascale) * dar/dt * ascale
dal/dt = dar/dt

Обратное преобразование: physical -----> virtual
a = al - aoff_left
xr = xl - xoff - radius * cos(a * ascale)
yr = yl - yoff - radius * sin(a * ascale)
ar = a - aoff_right

dxr/dt = dxl/dt + radius * sin(a * ascale) * dal/dt * ascale
dyr/dt = dyl/dt - radius * cos(a * ascale) * dal/dt * ascale
dar/dt = dal/dt

*/
class axstate_sincos_processor : public signal_processor
{
	struct axis_state ** leftside;   // order: x y a
	struct axis_state ** rightside;  // order: x y a

	position_t radius;

	position_t x_offset;
	position_t y_offset;
	position_t a_left_offset;
	position_t a_right_offset;

	float alpha_to_radian_scale;

public:
	int feedback(disctime_t time) override;
	int serve(disctime_t time) override;
	int command(int argc, char ** argv, char * output, int outmax) override;
	void deinit() override;
	struct signal_head * iterate_left(struct signal_head *) override;
	struct signal_head * iterate_right(struct signal_head *) override;

	void init(
	    const char* name,
	    struct axis_state ** leftside,
	    struct axis_state ** rightside,
	    position_t radius
	);

	void set_alpha_scale(float ascale);
	void set_offset(
	    position_t xoff,
	    position_t yoff,
	    position_t aloff,
	    position_t aroff);
	void set_x_offset(position_t xoff);
	void set_y_offset(position_t yoff);
	void set_a_left_offset(position_t aoff);
	void set_a_right_offset(position_t aoff);

};

#endif