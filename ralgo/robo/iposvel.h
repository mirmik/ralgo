#ifndef RALGO_ROBO_POSVEL_INTERFACES_H
#define RALGO_ROBO_POSVEL_INTERFACES_H

namespace robo 
{
	class i_velocity_setter 
	{
	public:
		virtual void set_velocity(double vel) = 0;
	};

	class i_position_setter
	{
	public:
		virtual void set_position(double vel) = 0;
	};

	class i_velocity_feedback 
	{
	public:
		virtual double feedback_velocity() = 0;
	};

	class i_position_feedback
	{
	public:
		virtual double feedback_position() = 0;
	};

	class i_velocity_driver : public i_velocity_setter, public i_velocity_feedback 
	{};
}

#endif