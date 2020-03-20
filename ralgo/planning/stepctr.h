#ifndef RALGO_MSERVO_CONTROLLER_H
#define RALGO_MSERVO_CONTROLLER_H

#include <limits>
#include <igris/util/bug.h>
#include <igris/sync/syslock.h>
#include <ralgo/planning/phase_driver.h>

#define NODTRACE 0
#include <igris/dtrace.h>

namespace ralgo
{
	class stepctr_server;

	class stepctr : public ralgo::phase_driver
	{
	public:
		const char* name;

		volatile int64_t control_steps_counter = 0;

		int32_t accum = 0; // Текущее значение аккумулятора.
		volatile int32_t step = 0; // Величина инкремента.

		int32_t width = 10*1000*1000;
		float _gear = 1;

		// Позиционный мультипликатор учитывает 
		// отношение командного импульса к расчетному.
		//float position_multiplier = 1;

		// Скоростной мультипликатор учитывает 
		// отношение командного импульса к расчетному 
		// + количество тиков алгоритма за расчетную единицу времени.
		float _speed_multiplier = 1;
		float _ticks_per_second = 1;
		
		bool emulate = false;		

		//stepctr_server * parent;

	public:
		stepctr(const char* name) : name(name) {}

		void set_gear(float mul, float ticks_per_second/*ticks_per_timeunit*/) 
		{
			// Учитываем модификатор времени.
			_gear = mul;
			this->_ticks_per_second = ticks_per_second;
			_speed_multiplier = mul 
				/ _ticks_per_second 
				* ralgo::discrete_time_frequency();
		}

		void set_gear(float mul) 
		{
			set_gear(mul, _ticks_per_second);
		}

		stepctr(const char* drvname) : ralgo::phase_driver(drvname)
		{}

		int64_t drive_position() 
		{
			return control_steps_counter;
		}

		float accum_part()
		{
			return (float)accum / width;
		}

		void set_accum_part(float acc)
		{
			DTRACE();
			accum = acc * width;
			DPRINT(accum);
			assert (accum > -width && accum < width);
		}

		// Установить ширину импульса в единицах инкремента.
		void set_step(int32_t _step) 
		{
			//DTRACE();
			system_lock();
			step = _step;
			system_unlock();

			//DPRINT(step);
			assert((step < width) && (step > -width));
		}

		float current_speed() override 
		{
			return (float)step / width / _speed_multiplier; 
			//phases_speed() / multiplier;
		}

		int64_t current_position() override 
		{
			igris::syslock_guard lock();
			return control_steps_counter * _gear + accum_part() * _gear; 
		}

		void set_current_position(int64_t pos) override 
		{
			igris::syslock_guard lock();
			control_steps_counter = pos / _gear;
			set_accum_part( (float)(pos - control_steps_counter * _gear) / _gear );	

			// Попытка выполнения позиционирования при смене текущей
			// позиции может быть фатальна, так что состояние 
			// контроллера нужно пересчитать.
			if (controller) 
			{
				controller->update_control();
				controller->stop();
			}
		}

		void set_speed(float spd) 
		{
			//DTRACE();
			float s = (float)width * spd * _speed_multiplier;
			if (!((s < width) && (s > -width))) 
			{
				DPRINT(width);
				DPRINT(s);
				DPRINT(spd);
				DPRINT(_speed_multiplier);
				DPRINT(name);
			}

			set_step(s);
		}

		void enable_power(bool en) override {} 

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

#endif