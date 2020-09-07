#ifndef RALGO_HEIMER_SPEED_PHASER_AXIS_H
#define RALGO_HEIMER_SPEED_PHASER_AXIS_H

#include <ralgo/heimer/phaser.h>
#include <ralgo/heimer/axis.h>

namespace heimer
{
	template <class P, class IntPos, class V>
	class phaser_axis : public axis_node<P,V>
	{
		using parent = axis_node<P,V>;

	public:
		phaser<P,IntPos,V> * controlled = nullptr;
		V compspd = 0;
		float compkoeff = 0;
		P diff;

	public:
		constexpr 
		phaser_axis(const char* name) : axis_node<P,V>(name) {}
		
		constexpr 
		phaser_axis(const char* name, phaser<P,IntPos,V>* phaser)
			: axis_node<P,V>(name), controlled(phaser)
		{}

		void print_info() override 
		{
			nos::println("compkoeff:", compkoeff);
			nos::println("compspd:", compspd);
			nos::println("diff * 1000:", diff * 1000);
			parent::print_info();
		}

		void apply_speed(V spd)
		{
			controlled->set_speed(spd);
		}

		void feedback() 
		{
			parent::feedpos = controlled->feedback_position();
			parent::feedspd = controlled->feedback_speed();
		}

		control_node * iterate(control_node * it) 
		{
			if (it == nullptr)
				return controlled;
			return nullptr;
		}

		void serve_impl() override
		{
			// Ошибка по установленному значению.
			if (controlled->update_needed) 
			{
				controlled->invoke_update();
			}

			diff = parent::position_error();

			if (diff > 1) 
			{
				controlled->set_speed(0);	
				
				char str[56];
				sprintf(str, "position deviation error : mnemo:%s", parent::mnemo());
				ralgo::warn(str);

				force_stop_interrupt_args msg("position deviation error");
				parent::throw_interrupt(&msg);	
			}

			// Скорость вычисляется как
			// сумма уставной скорости на
			compspd = parent::ctrspd + compkoeff * diff;
			
			controlled->set_speed(compspd);
		}

		void set_compensate(float val) { compkoeff = val; }

		V compensated_speed()
		{
			return compspd;
		}

		void update_state()
		{
			parent::feedpos = controlled->target_position();
			parent::feedspd = controlled->feedback_speed();
		}

		P request_feedback_position() override
		{
			// Должна обновляться управляющим устройством
			return controlled->target_position();
		}
	};
}

#endif
