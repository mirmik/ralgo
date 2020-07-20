#ifndef RALGO_HEIMER_SPEED_PHASER
#define RALGO_HEIMER_SPEED_PHASER

#include <igris/sync/syslock.h>
#include <ralgo/disctime.h>
#include <ralgo/heimer/control.h>

namespace heimer
{
	template <class ExtPos, class IntPos, class Speed>
	class phaser
	{
	protected:
		volatile IntPos _feedback_position = 0;
		volatile IntPos _target_position = 0;
		float _gain = 1;

		float _setted_speed = 0;

	public:
		ExtPos int2ext_pos(IntPos intpos) { return intpos / _gain; }
		IntPos ext2int_pos(ExtPos extpos) { return extpos * _gain; }

		Speed int2ext_spd(Speed intspd) { return intspd / _gain; }
		Speed ext2int_spd(Speed extspd) { return extspd * _gain; }

		virtual void set_speed_internal_impl(Speed spd) = 0;
		void set_speed_internal(Speed spd) { _setted_speed = spd; set_speed_internal_impl(spd); }
		Speed setted_speed_internal() { return _setted_speed; }

		void set_speed(Speed spd) { set_speed_internal(ext2int_spd(spd)); }
		Speed setted_speed() { return int2ext_spd(setted_speed_internal()); }
		Speed feedback_speed() { return setted_speed(); }

		IntPos target_position_internal()
		{
			igris::syslock lock;
			return _target_position;
		}
		IntPos feedback_position_internal()
		{
			igris::syslock lock;
			return _feedback_position;
		}

		ExtPos target_position()
		{
			return int2ext_pos(target_position_internal());
		}

		ExtPos feedback_position()
		{
			return int2ext_pos(feedback_position_internal());
		}

		virtual void serve() = 0;

		void set_gain(float gain) { _gain = gain; }
		auto gain() { return _gain; }
	};

	template <class ExtPos, class Speed>
	class phaser_emulator : public phaser<ExtPos, ExtPos, Speed>
	{
		using parent = phaser<ExtPos, ExtPos, Speed>;
		double integrator = 0;

		int64_t lasttime;

	public:
		phaser_emulator<ExtPos, Speed>()
		{
			lasttime = ralgo::discrete_time();
		}

		void serve() override
		{
			int64_t delta = ralgo::discrete_time() - lasttime;

			integrator += parent::_setted_speed * (double)delta
				/ ralgo::discrete_time_frequency();
			parent::_target_position = integrator;
			parent::_feedback_position = integrator;

			lasttime = ralgo::discrete_time();
		}

		void set_speed_internal_impl(Speed spd) override
		{
			parent::_setted_speed = spd;
		}
	};
}

#endif