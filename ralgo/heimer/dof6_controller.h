#ifndef RALGO_HEIMER_DOF6_CONTROLLER_H
#define RALGO_HEIMER_DOF6_CONTROLLER_H

#include <ralgo/heimer/signal_processor.h>
#include <ralgo/heimer/dof6_signal.h>

namespace heimer
{
	class dof6_controller : public signal_processor
	{
		dof6_signal * controlled = nullptr;

		velocity_t vel;
		acceleration_t acc;
		acceleration_t dcc;

	public:
		dof6_controller() = default;
		dof6_controller(const char * name);

		void set_controlled(dof6_signal * controlled);

		int feedback(disctime_t time) override;
		int serve(disctime_t time) override;
		int command(int argc, char ** argv, char * output, int outmax) override;
		void deinit() override;
		signal_head * iterate_left(signal_head *) override;
		signal_head * iterate_right(signal_head *) override;

		int leftsigtype(int i) override;
		signal_head * leftsig(int i) override;
		void set_leftsig(int i, signal_head * sig) override;
	

		void set_velocity(velocity_t v);
		void set_acceleration(velocity_t v);
		void set_decceleration(velocity_t v);
	};
}

#endif
