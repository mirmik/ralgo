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

		void evaluate_output_sensivities(rabbit::screw3<double> * sensivities);
	}
}

#endif