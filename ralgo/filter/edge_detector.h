#ifndef RALGO_EDGE_DETECTOR_H
#define RALGO_EDGE_DETECTOR_H

#include <cmath>

namespace ralgo
{
	enum class EdgeDetectorStatus
	{
		UpdateFallingCandidate,
		UpdateRisingCandidate,
		FallingEvent,
		RisingEvent,
		None
	};

	class edge_detector
	{
		bool phase = false;
		bool last_direction = false;

		float trigger_level;

		float start;
		float last;

		bool prevent_halfspaces_area = true;

	public:
		edge_detector(float trigger_level)
			: trigger_level(trigger_level)
		{}

		EdgeDetectorStatus serve(float signal)
		{
			EdgeDetectorStatus status = EdgeDetectorStatus::None;
			bool direction = signal - last > 0;

			if (last_direction != direction)
			{
				start = signal;

				if (direction)
					status = EdgeDetectorStatus::UpdateRisingCandidate;
				else 
					status = EdgeDetectorStatus::UpdateFallingCandidate;
			}

			else
			{
				if (fabs(signal - start) > trigger_level)
				{
					if (direction == false)
					{
						if (phase == true)
						{
							phase = false;

							if (!prevent_halfspaces_area || start > 0)
								status = EdgeDetectorStatus::FallingEvent;
						}
					}

					if (direction == true)
					{
						if (phase == false)
						{
							phase = true;

							if (!prevent_halfspaces_area || start < 0)
								status = EdgeDetectorStatus::RisingEvent;
						}
					}
				}
			}

			last_direction = direction;
			last = signal;

			return status;
		}
	};

	using rising_edge_detector = edge_detector;
}

#endif