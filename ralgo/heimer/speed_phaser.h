#ifndef RALGO_HEIMER_SPEED_PHASER
#define RALGO_HEIMER_SPEED_PHASER

#include <ralgo/disctime.h>
#include <ralgo/heimer/device.h>

#include <igris/dprint.h>
#include <iostream>

namespace ralgo 
{
	namespace heimer 
	{
		template <class ExtPos, class IntPos, class Speed>
		class speed_phaser : public ralgo::heimer::device
		{
		protected:
			volatile IntPos _feedback_position = 0;
			volatile IntPos _target_position = 0;
			float _deltatime = 1;
			float _gain = 1;

			float _setted_speed = 0;

		public:
			ExtPos int2ext_pos(IntPos intpos) { return intpos / _gain; } 
			IntPos ext2int_pos(ExtPos extpos) { return extpos * _gain; }

			Speed int2ext_spd(Speed intspd) { return intspd / _gain; } 
			Speed ext2int_spd(Speed extspd) { return extspd * _gain; }

			virtual void set_speed_internal_impl(Speed spd) = 0;
			void set_speed_internal(Speed spd) { _setted_speed=spd; set_speed_internal_impl(spd); }
			Speed setted_speed_internal() { return _setted_speed; }

			void set_speed(Speed spd) { set_speed_internal(ext2int_spd(spd)); }
			Speed setted_speed() { return int2ext_spd(setted_speed_internal()); }
		
			IntPos target_position_internal() { return _target_position; }
			IntPos feedback_position_internal() { return _feedback_position; }

			ExtPos target_position() { return int2ext_pos(_target_position); } 
			ExtPos feedback_position() { return int2ext_pos(_feedback_position); }

			virtual void serve() = 0;
			
			void set_gain(float gain) { DTRACE(); DPRINT(gain); _gain = gain; } 
			void set_deltatime(int32_t ticks_per_second) 
			{	//_deltatime = ralgo::discrete_time_frequency() / ticks_per_second; 
				_deltatime = 1 / ticks_per_second; 
			}
		};

		template <class ExtPos, class IntPos, class Speed>
		class speed_phaser_emulator : public speed_phaser<ExtPos, IntPos, Speed>
		{
			using parent = speed_phaser<ExtPos, IntPos, Speed>;
			double integrator = 0;

		public:
			void serve() override
			{
				integrator +=
					parent::_setted_speed * parent::_deltatime;
				parent::_target_position = integrator;
				parent::_feedback_position = integrator;
			}

			void set_speed_internal_impl(Speed spd) override
			{}
		};
	}
}

#endif