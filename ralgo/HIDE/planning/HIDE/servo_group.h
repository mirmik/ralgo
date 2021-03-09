#ifndef RALGO_PLANNING_SERVO_GROUP_H
#define RALGO_PLANNING_SERVO_GROUP_H

#include <ralgo/planning/servo.h>

namespace ralgo 
{
	class servo_group 
	{
		bool enabled;

		ralgo::servo * srvs;
		int total;
		
		ralgo::multiax_trajectory * current_trajectory;
		ralgo::servo_interpolation_line_traj line_traj;

		void enforce_squad() 
		{
			for (int i = 0; i < total; ++i) 
			{
				srvs[i].external_controller = this;
				srvs[i].current_status = ServoControlStatus::external;
			}
		}

		void release_squad() 
		{
			for (int i = 0; i < total; ++i) 
			{
				srvs[i].external_controller = nullptr;
				srvs[i].current_status = ServoControlStatus::stoped;
			}			
		}

		int prepare_operation()
		{
			bool allfree = check_can_operation();

			if (!allfree)
				return -1;

			enforce_squad();

			return 0;
		}

		int interpolate_move(
			const igris::array_view<int64_t>& startpos, 
			const igris::array_view<int64_t>& targets, 
			int64_t curtime, int64_t fintime) 
		{
			bool allfree = check_can_operation();

			if (!allfree)
				return -1;

			enforce_squad();

			line_traj.reset_speed_mode(targets.data(), );
		}

		int interpolate_absolute_move(
			const igris::array_view<int64_t>& targets, 
			float speed
		) 
		{
			int ans;

			if ((ans=prepare_operation())) 
				return ans;
		}

		void serve(int64_t time) 
		{
			if (!enabled)
				return;

			ralgo::phase<int64_t, float> phs[total];

			int ret = 
				current_trajectory->inloctime(time - opstart_tstamp, &phs);

			if (ret) 
			{
				release_squad();
			}
		}


	};
}

#endif