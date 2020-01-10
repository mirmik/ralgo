#ifndef RALGO_PLANNING_SPEED_DRIVER_H
#define RALGO_PLANNING_SPEED_DRIVER_H

#include <math.h>
#include <ralgo/planning/signaler.h>
#include <igris/sync/syslock.h>
#include <assert.h>

#include  <ralgo/planning/disctime.h>

namespace ralgo
{
	// Данныый класс Устанавливает скорость
	// и интегрирует её с целью вычисления позиции.
	// Вычисление интеграла может быть как виртуальным,
	// так и физическим в зависимости от имлементации
	template < class Position, class Speed >
	class speed_phaser :
		public speed_signaler_store<Speed>,
		public position_signaler<Position>
	{
		using spdeed_store = speed_signaler_store<Speed>;

	public:
		speed_phaser() {}

		virtual Position current_position () = 0;

		// Имплементация помимо прочего должна
		// установить текущую позицию.
		virtual void set_speed_impl(Speed spd) = 0;

		void set_speed(Speed spd)
		{
			set_speed_impl(spd);
			spdeed_store::set_speed_store(spd);
		}

		Speed current_speed() 
		{
			return spdeed_store::speed();
		}
	};

	// Предыдущая версия содержала перевод шага интегрирования
	// в целочисленное пространство ширины width.
	// Идея была неплоха, но плохо отлаживалась.
	// TODO: Вернуться после отладки данной версии,
	// Поскольку это может оптимизировать быстродействие.
	template <class Position, class Speed = float, class TimeDelta = float>
	class stepctr_speed_phaser :
		public speed_phaser<Position, Speed>
	{
		using phaser = speed_phaser<Position, Speed>;

		Position curstep;
		
		Position pulsewidth;
		Position accum;

		Position virtual_pos = 0;
		Position control_pos = 0;

		float speed_multiplier = 1;

	public:
		Position position () { return virtual_pos; }
		Position current_position () { return virtual_pos; }
		void set_current_position(Position pos) { 
			virtual_pos = pos;
			control_pos = pos;
		}

		void set_gear(Position gear) 
		{
			pulsewidth = gear;
		}

		void set_ticks_per_second(int32_t ticks_per_second) 
		{
			speed_multiplier = ralgo::discrete_time() / ticks_per_second;			
		}

		virtual void inc() = 0;
		virtual void dec() = 0;

		void set_speed_impl(Speed spd) override
		{
			igris::syslock lock();
			curstep = spd * speed_multiplier;
			assert(ABS(curstep) < pulsewidth);
		}

		void serve()
		{
			virtual_pos += curstep;
			auto diffpos = virtual_pos - control_pos;

			bool positive = diffpos > 0;

			if (positive) 
			{
				if (diffpos > pulsewidth) 
				{
					inc();
					virtual_pos += pulsewidth;
				}
			}

			else 
			{
				if (diffpos < -pulsewidth) 
				{
					dec();
					virtual_pos -= pulsewidth;
				}
			}

			/*accum += curstep;

			bool negative = accum < 0;

			if (!negative)
			{
				if (accum > pulsewidth)
				{
					accum -= pulsewidth;
					inc();
				}
			}

			else
			{
				if (accum < -pulsewidth)
				{
					control_pos += step
					accum += pulsewidth;
					dec();
				}
			}*/
		}
	};
}

#endif