#ifndef RALGO_HEIMER_AXSTATE_POSE3_PROCESSOR_H
#define RALGO_HEIMER_AXSTATE_POSE3_PROCESSOR_H

#include <ralgo/heimer/axstate_signal_processor.h>
#include <ralgo/heimer/dof6_signal.h>
#include <rabbit/space/pose3.h>
#include <rabbit/space/screw.h>

namespace heimer
{
	/**
		Порядок правых осей: x y z .
	*/
	class axstate_pose3_chain_processor : public signal_processor
	{
		heimer::axis_state ** leftside;
		heimer::dof6_signal * rightside; 

		rabbit::pose3<double> * constant_transforms; // links_count + 1
		rabbit::screw3<double> * local_sentivities; // links_count
		int links_count;

		rabbit::pose3<position_t> control_position;
		double compensation_coefficient;

	public:
		axstate_pose3_chain_processor(const char * name, int leftdim);

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