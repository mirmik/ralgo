#ifndef RALGO_HEIMER_AXSTATE_POSE3_PROCESSOR_H
#define RALGO_HEIMER_AXSTATE_POSE3_PROCESSOR_H

#include <ralgo/heimer/axstate_signal_processor.h>
#include <ralgo/heimer/dof6_signal.h>
#include <ralgo/heimer/axis_state.h>

#include <ralgo/space/pose3.h>
#include <ralgo/space/screw.h>

namespace heimer
{
	class axstate_pose3_chain_settings 
	{
	public:
		ralgo::pose3<double> constant_transform;
		ralgo::screw3<double> local_sensivity;
		heimer::axis_state * controlled = nullptr;
	};

	// runtime
	class axstate_pose3_chain_temporary 
	{
	public:
		ralgo::pose3<double> leftmatrix;
		ralgo::pose3<double> rightmatrix;
		ralgo::screw3<double> result_screw;
	};

	/**
		Порядок правых осей: x y z .
	*/
	class axstate_pose3_chain_processor : public signal_processor
	{		
	public:
		ralgo::pose3<double> last_constant_transform;
		heimer::dof6_signal * rightside; 
		axstate_pose3_chain_settings * settings = nullptr;
		axstate_pose3_chain_temporary * temporary = nullptr;

		ralgo::pose3<position_t> control_position;

	public:
		axstate_pose3_chain_processor(const char * name, int leftdim);
		void set_resources(axstate_pose3_chain_settings * settings, axstate_pose3_chain_temporary * tsettings);
		
		ralgo::pose3<position_t> evaluate_feedback_position();
		
		ralgo::pose3<position_t> evaluate_target_position();
		ralgo::screw3<position_t> evaluate_target_velocity();
		ralgo::screw3<position_t> evaluate_position_error();

		int feedback(disctime_t time) override;
		int serve(disctime_t time) override;
		int command(int argc, char ** argv, char * output, int outmax) override;
		void deinit() override;
		void on_activate(disctime_t) override;

		axis_state * leftax(int i);
		void set_leftside(axis_state ** arr);
		void set_constant(int, float, float, float, float, float, float);
		void set_sensivity(int, float, float, float, float, float, float);

		void evaluate_error();
		void evaluate_output_sensivities(ralgo::screw3<double> * sensivities);
		void backpack(ralgo::screw3<double> * sensivities);

		void allocate_resources();

		int leftsigtype(int i) override;
		int rightsigtype(int i) override;
		signal_head * leftsig(int i) override;
		signal_head * rightsig(int i) override;
		void set_leftsig(int i, signal_head *) override;
		void set_rightsig(int i, signal_head *) override;
	};
}

#endif