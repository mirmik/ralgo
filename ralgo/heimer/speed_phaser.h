#ifndef RALGO_HEIMER_SPEED_PHASER
#define RALGO_HEIMER_SPEED_PHASER

#include <ralgo/disctime.h>
#include <ralgo/heimer/device.h>

namespace ralgo 
{
	namespace heimer 
	{
		template <class Position, class Speed>
		class speed_phaser : public ralgo::heimer::device
		{
		protected:
			Position _feedback_position = 0;
			Position _target_position = 0;
			float _deltatime = 1;
			float _gain = 1;

			Speed _speed = 0;

		public:
			virtual void set_speed(Speed spd) { _speed = spd * parent::_gain; }
			virtual void serve() = 0;
			Speed speed() { return _speed / parent::_gain; }
		
			Position target_position() { return _target_position; }
			Position feedback_position() { return _feedback_position; }

			void set_gain(float gain) { _gain = gain; } 
			void set_deltatime(int32_t ticks_per_second) {	_deltatime = 1. / ticks_per_second; }
		};

		template <class Position, class Speed>
		class speed_phaser_emulator : public speed_phaser<Position, Speed>
		{
			using parent = speed_phaser<Position, Speed>;

		public:
			void serve() 
			{
				parent::_feedback_position += _speed * parent::_deltatime;
			}
		};
	}
}

#endif