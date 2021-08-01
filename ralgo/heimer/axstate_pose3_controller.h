#ifndef RALGO_HEIMER_AXSTATE_POSE3_CONTROLLER_H
#define RALGO_HEIMER_AXSTATE_POSE3_CONTROLLER_H

#include <ralgo/heimer/axstate_signal_processor.h>

namespace heimer 
{
	class axstate_pose3_chain_controller : public axstate_signal_processor
	{
		rabbit::pose3<double> * constant_transforms; // links_count + 1
		rabbit::screw3<double> * local_sentivities; // links_count
		int links_count;

		pose3<position_t> control_position; 
		double compensation_coefficient;

		pose3<position_t> evaluate_target_position(); 
		screw3<position_t> evaluate_target_velocity();

		screw3<position_t> evaluate_position_error();



		void evaluate_error()

		void evaluate_output_sensivities(rabbit::screw3<double> * sensivities);
		void backpack(rabbit::screw3<double> * sensivities) {  }
	}
}

#endif