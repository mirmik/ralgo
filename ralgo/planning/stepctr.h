#ifndef RALGO_MSERVO_CONTROLLER_H
#define RALGO_MSERVO_CONTROLLER_H

#include <limits>

#include <ralgo/planning/speed_driver.h>

namespace ralgo
{
	class stepctr_server;

	class stepctr : public ralgo::phase_driver
	{
	public:
		int32_t accum = 0; // Текущее значение аккумулятора.
		volatile int32_t step = 0; // Величина инкремента.

		int32_t width = 10*1000*1000;

		float tick_per_timeunit = 1;
		float timeunit_per_tick = 1;		

		//stepctr_server * parent;

	public:
		stepctr()
		{}

		// Установить ширину импульса в единицах инкремента.
		void set_step(int _step) 
		{
			system_lock();
			step = _step;
			system_unlock();
		}

		void set_declared_serve_freq(float arg) 
		{
			tick_per_timeunit = arg;
			timeunit_per_tick = 1/arg;
		}

		void set_speed(float posunit_per_timeunit) 
		{
			dprln();
			set_step(width * timeunit_per_tick * posunit_per_timeunit);
		}

		virtual void inc() = 0;
		virtual void dec() = 0;

		void serve() 
		{
			accum += step;

			bool negative = accum < 0;

			if (negative) 
			{
				if (accum < -width) 
				{
					dec();
					accum += width;
					--target_impulse_position;
				}
			}

			else 
			{
				if (accum > width) 
				{
					inc();
					accum -= width; 
					++target_impulse_position;
				}
			}
		}
	};
}
/*	class stepctr_server
	{
	public:

		stepctr ** axes;
		int axtotal;

	public:
		stepctr_server(stepctr ** ptr, int total)
		{
			axes = ptr;
			axtotal = total;

			stepctr * ax;

			for (int i = 0; i < axtotal; ++i)
			{
				ax = axes[i];
				ax->parent = this;
			}
		}

		// Установить стоимость шага обслуживания.
		void set_tdelta(int32_t delta)
		{
			tdelta = delta;
		}

		void serve()
		{
			int diff;
			stepctr * ax;

			for (int i = 0; i < axtotal; ++i)
			{
				ax = axes[i];

				if ((diff = ax->imptarget - ax->impcurrent) != 0)
				{
					ax->stepaccum += tdelta;
					if (ax->stepaccum > ax->stepwidth)
					{
						ax->stepaccum -= ax->stepwidth;
						if (diff > 0)
						{
							ax->inc();
							++ax->impcurrent;
						}
						else
						{
							ax->dec();
							--ax->impcurrent;
						}

						if (ax->impcurrent == ax->imptarget)
							ax->stepaccum = 0;
					}
				}
			}
		}
	};
}


void ralgo::stepctr::set_stepwidth(int stepwidth)
{
	assert(stepwidth >= parent->tdelta);
	//if (parent->tdelta > stepwidth)
	//	BUG();

	this->stepwidth = stepwidth;
}

void ralgo::stepctr::serve() 
{

}*/

#endif