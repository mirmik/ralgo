#ifndef RALGO_HEIMER_AXSTATE_POSE3_PROCESSOR_H
#define RALGO_HEIMER_AXSTATE_POSE3_PROCESSOR_H

#include <ralgo/heimer/axstate_signal_processor.h>
#include <ralgo/heimer/dof6_signal.h>
#include <rabbit/space/pose3.h>
#include <rabbit/space/screw.h>

namespace heimer
{
	class axstate_pose3_chain_settings 
	{
		rabbit::pose3<double> constant_transform;
		rabbit::screw3<double> local_sensivities;
		rabbit::axis_state * state;
	};

	class axstate_pose3_chain_temporary 
	{
		// runtime
		rabbit::pose3<double> leftmatrix;
		rabbit::pose3<double> rightmatrix;
		rabbit::screw3<double> result_screw;
	};

	/**
		Порядок правых осей: x y z .
	*/
	class axstate_pose3_chain_processor : public signal_processor
	{		
		rabbit::pose3<double> C0_constant_transform;
		heimer::dof6_signal * rightside; 
		axstate_pose3_chain_settings * settings;
		axstate_pose3_chain_temporary * temporary;

		int _leftdim;

		rabbit::pose3<position_t> control_position;

	public:
		axstate_pose3_chain_processor(const char * name, int leftdim);
		void set_resources(axstate_pose3_chain_settings * settings, axstate_pose3_chain_temporary * tsettings);
		void set_resources(axstate_pose3_chain_settings * settings, axstate_pose3_chain_temporary * tsettings);
		
		rabbit::pose3<position_t> evaluate_current_position();
		
		rabbit::pose3<position_t> evaluate_target_position();
		rabbit::screw3<position_t> evaluate_target_velocity();
		rabbit::screw3<position_t> evaluate_position_error();

		int feedback(disctime_t time) override;
		int serve(disctime_t time) override;
		int command(int argc, char ** argv, char * output, int outmax) override;
		void deinit() override;
		void on_activate(disctime_t) override;

		void evaluate_error();
		void evaluate_output_sensivities(rabbit::screw3<double> * sensivities);
		void backpack(rabbit::screw3<double> * sensivities);
	};
}

#endif