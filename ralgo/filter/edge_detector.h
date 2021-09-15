#ifndef RALGO_EDGE_DETECTOR_H
#define RALGO_EDGE_DETECTOR_H

#include <cmath>

namespace ralgo
{
	class rising_edge_detector
	{
		bool phase = false;
		bool last_direction = false;

		float trigger_level;

		float start;
		float last;

	public:
		rising_edge_detector(float trigger_level)
			: trigger_level(trigger_level)
		{}

		int serve(float signal)
		{
			int status = 0;
			bool direction = signal - last > 0;

			if (last_direction != direction)
			{
				start = signal;
			}

			if (direction == false)
			{
				if (fabs(signal - start) > trigger_level)
					phase = false;
			}

			if (direction == true)
			{
				if (fabs(signal - start) > trigger_level)
				{
					if (phase == false)
					{
						phase = true;

						if (start < 0)
							status = 1;
					}
				}
			}
			last_direction = direction;
			last = signal;

			return status;
		}
	};
}

#endif