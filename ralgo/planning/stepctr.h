#ifndef RALGO_MSERVO_CONTROLLER_H
#define RALGO_MSERVO_CONTROLLER_H

#include <limits>

#include <ralgo/planning/phase_driver.h>

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

		bool emulate = false;		

		//stepctr_server * parent;

	public:
		stepctr()
		{}

		int32_t get_accum() override
		{
			return accum * tick_per_timeunit / width;
		}

		// Установить ширину импульса в единицах инкремента.
		void set_step(int32_t _step) 
		{
			system_lock();
			step = _step;
			system_unlock();
			//DPRINT(step);

			assert((step < width) && (step > -width));
		}

		void set_declared_serve_freq(float arg) 
		{
			tick_per_timeunit = arg / ralgo::discrete_time_frequency();
			timeunit_per_tick = 1/tick_per_timeunit;
		}

		void set_speed(float steps_per_timeunit) 
		{
			//DPRINT(steps_per_timeunit);
			set_step(width * timeunit_per_tick * steps_per_timeunit);
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
					if (!emulate)
						dec();

					accum += width;
					--control_steps_counter;
				}
			}

			else 
			{
				if (accum > width) 
				{
					if (!emulate)
						inc();
					
					accum -= width; 
					++control_steps_counter;
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