#ifndef RALGO_HEIMER_STEPCTR_H
#define RALGO_HEIMER_STEPCTR_H

#include <assert.h>
#include <igris/math.h>
#include <igris/sync/syslock.h>
#include <igris/dtrace.h>
#include <igris/dprint.h>
#include <ralgo/heimer/speed_phaser.h>

namespace ralgo
{
	namespace heimer
	{
		// Предыдущая версия содержала перевод шага интегрирования
		// в целочисленное пространство ширины width.
		// Идея была неплоха, но плохо отлаживалась.
		// TODO: Вернуться после отладки данной версии,
		// Поскольку это может оптимизировать быстродействие.
		template < class Position, class IntPos, class Speed >
		class stepctr : public speed_phaser<Position, IntPos, Speed>
		{
		protected:
			using phaser = speed_phaser<Position, IntPos, Speed>;
			using phaser::ext2int_pos;

			int32_t steps_total = 0;

			volatile double curstep = 0;

			IntPos pulsewidth = 0;
			IntPos pulsewidth_triggered = 0;
			IntPos accum = 0;

			int not_corrected_counter = 0;

			float triglevel = 0.7;
			volatile double virtual_pos = 0;
			volatile double control_pos = 0;

			//float speed_multiplier = 1;

		public:
			IntPos current_step() { return curstep; }
			void set_curstep(IntPos curstep) { this->curstep = curstep; }

			int step_counter() { return steps_total; }

			//Position position () { return virtual_pos; }
			//Position current_position () { return virtual_pos; }
			void set_current_position(Position pos)
			{
				virtual_pos = ext2int_pos(pos);
				control_pos = ext2int_pos(pos);
			}

			void set_gear(Position gear)
			{
				pulsewidth = gear;
				pulsewidth_triggered = gear * triglevel;
			}

			auto gear() { return pulsewidth; }

			virtual void inc() = 0;
			virtual void dec() = 0;

			void set_speed_internal_impl(Speed spd) override
			{
				not_corrected_counter = 0;

				igris::syslock lock();
				curstep = spd * phaser::_deltatime;

				if ( ABS(curstep) > pulsewidth )
				{
					DPRINT(pulsewidth);
					DPRINT(curstep);
					DPRINT(phaser::_deltatime);
					DPRINT(spd);
					DPRINT(phaser::int2ext_spd(spd));
					assert(ABS(curstep) <= pulsewidth);
					curstep = pulsewidth > 0 ? pulsewidth : -pulsewidth;	
				}
			}

			void serve()
			{
				not_corrected_counter++;
				if (not_corrected_counter > 10000) 
				{
					set_speed_internal_impl(0);
					return;
				}

				virtual_pos += curstep;
				auto diffpos = virtual_pos - control_pos;

				bool positive = diffpos > 0;

				if (positive)
				{
					if (diffpos > pulsewidth_triggered)
					{
						inc();
						steps_total++;
						control_pos += pulsewidth;
					}
				}

				else
				{
					if (diffpos < -pulsewidth_triggered)
					{
						dec();
						steps_total--;
						control_pos -= pulsewidth;
					}
				}
				phaser::_target_position = virtual_pos;
			}
		};

		template <class Position, class IntPos, class Speed>
		class stepctr_emulator : public stepctr<Position, IntPos, Speed>
		{
			using parent = stepctr<Position, IntPos, Speed>;
			void inc()
			{
				parent::_feedback_position += parent::pulsewidth;
			}

			void dec()
			{
				parent::_feedback_position -= parent::pulsewidth;
			}
		};
	}
}

#endif