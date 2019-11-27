#ifndef RALGO_MSERVO_CONTROLLER_H
#define RALGO_MSERVO_CONTROLLER_H

#include <limits>
#include <igris/util/bug.h>
#include <igris/sync/syslock.h>
#include <ralgo/planning/phase_driver.h>

namespace ralgo
{
	class stepctr_server;

	class stepctr : public ralgo::phase_driver
	{
	public:
		volatile int64_t control_steps_counter = 0;

		int32_t accum = 0; // Текущее значение аккумулятора.
		volatile int32_t step = 0; // Величина инкремента.

		int32_t width = 10*1000*1000;

		// Позиционный мультипликатор учитывает 
		// отношение командного импульса к расчетному.
		float position_multiplier = 1;

		// Скоростной мультипликатор учитывает 
		// отношение командного импульса к расчетному 
		// + количество тиков алгоритма за расчетную единицу времени.
		float speed_multiplier = 1;
		
		bool emulate = false;		

		//stepctr_server * parent;

	public:
		void set_gear(float mul, float ticks_per_second/*ticks_per_timeunit*/) 
		{
			// Учитываем модификатор времени.
			position_multiplier = mul;
			speed_multiplier = mul 
				/ ticks_per_second 
				* ralgo::discrete_time_frequency();
		}

		stepctr(const char* drvname) : ralgo::phase_driver(drvname)
		{}

		int64_t drive_position() 
		{
			return control_steps_counter;
		}

		float get_accum_part()
		{
			return (float)accum / width;
		}

		void set_accum_part(float acc)
		{
			accum = acc * width;
			assert (accum > -width && accum < width);
		}

		// Установить ширину импульса в единицах инкремента.
		void set_step(int32_t _step) 
		{
			system_lock();
			step = _step;
			system_unlock();

			assert((step < width) && (step > -width));
		}

		/*void set_declared_serve_freq(float arg) 
		{
			tick_per_timeunit = arg / ralgo::discrete_time_frequency();
			timeunit_per_tick = 1/tick_per_timeunit;
		}*/

		/*void set_phases_speed(float steps_per_timeunit) 
		{
			//DPRINT(steps_per_timeunit);
			//PRINT(steps_per_timeunit);
			//PRINT(timeunit_per_tick);
			//PRINT(width);
			//set_step(width * timeunit_per_tick * steps_per_timeunit);
		}*/

		//float phases_speed() 
		//{
		//	PRINT((int32_t)step);
		//	return (float)step / timeunit_per_tick / width;
		//}

		float current_speed() override 
		{
			return (float)step / width / speed_multiplier; 
			//phases_speed() / multiplier;
		}

		float current_position() override 
		{
			igris::syslock_guard lock();
			return ((float)control_steps_counter + get_accum_part()) 
				/ position_multiplier;
		}

		void set_current_position(float pos) override 
		{
			igris::syslock_guard lock();

			float fcounter = pos * position_multiplier;

			int64_t counter = round(fcounter);

			set_accum_part(fcounter - counter);
			control_steps_counter = counter;
		}

		void set_speed(float spd) 
		{
			auto s = width * spd * speed_multiplier;
			if (!((s < width) && (s > -width))) 
			{
				DPRINT(width);
				DPRINT(s);
				DPRINT(spd);
				DPRINT(speed_multiplier);
			}

			//set_phases_speed(spd * speed_multiplier);
			//PRINT(spd);
			set_step(s);
		}

		void enable_power(bool en) override {} 

		virtual void inc() = 0;
		virtual void dec() = 0;

		void serve() 
		{
			accum += step;
			//PRINT((int32_t)step);
			//PRINT(accum);

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