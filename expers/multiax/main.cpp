#include <hal/board.h>
#include <hal/irq.h>

#include <systime/systime.h>

#include <genos/sched.h>
#include <genos/api.h>
#include <genos/schedee/autom.h>

#include <ralgo/planning/multiax.h>

void* blink(void * ptr, int * state) 
{
	board_led.toggle();
	msleep(500);
	return NULL;
}

genos::autom_schedee blink_schedee(blink);

int main() 
{
	board_init();
	schedee_manager_init();
	
	blink_schedee.run();

	rabbit::ellipse_curve2<float> ellipse 
	{ 
		rabbit::pnt2<float> { 0, 0 }, 
		rabbit::vec2<float> { 3, 0 },
		rabbit::vec2<float> { 0, 2 }
	};

	rabbit::trimmed_curve2<float> curve(&ellipse, 0, 1.5);

	irqs_enable();

	dprln("HelloWorld");
	dprln((float)ellipse.d0(0).x, (float)ellipse.d0(0).y);
	dprln((float)ellipse.d0(M_PI/2).x, (float)ellipse.d0(M_PI/2).y);
	dprln((float)ellipse.d0(M_PI).x, (float)ellipse.d0(M_PI).y);
	//dprln(ellipse.d0(0).y);

	ralgo::geom2d_trajectory<> traj(&curve, 10);
	
	while(1) 
		__schedule__();
}

void __schedule__() 
{
	while(1) 
	{
		ktimer_manager_step();
		schedee_manager_step();
	}
}