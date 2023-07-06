#ifndef RALGO_CNC_FEEDBACK_H
#define RALGO_CNC_FEEDBACK_H

#include <algorithm>
#include <igris/container/span.h>
#include <igris/container/static_vector.h>
#include <igris/event/delegate.h>
#include <ralgo/cnc/planner.h>

namespace cnc
{
    class feedback_guard
    {
    private:
        igris::delegate<void, igris::span<int64_t>>
            _set_feedback_position_callback;

        igris::static_vector<double, NMAX_AXES> feedback_to_drive = {};
        igris::static_vector<double, NMAX_AXES> control_to_drive =
            {}; //< этот массив равен gears

        // максимальное значение drop_pulses, после которого вызывается
        // planner->alarm_stop()
        igris::static_vector<double, NMAX_AXES> maximum_drop_pulses = {};

        cnc::planner *planner = nullptr;

    public:
        feedback_guard(cnc::planner *planner) : planner(planner) {}

        void set_feedback_to_drive_multiplier(igris::span<double> mult)
        {
            std::copy(mult.begin(), mult.end(), feedback_to_drive.begin());
        }

        void set_control_to_drive_multiplier(igris::span<double> mult)
        {
            std::copy(mult.begin(), mult.end(), control_to_drive.begin());
        }

        void set_feedback_position(igris::span<int64_t> pos)
        {
            std::array<int64_t, NMAX_AXES> fpos;
            for (size_t i = 0; i < pos.size(); ++i)
                fpos[i] = static_cast<int64_t>(pos[i] / feedback_to_drive[i]);
            _set_feedback_position_callback(igris::span(fpos));
        }

        void verify_position_and_alarm_if_needed(
            igris::span<int64_t> feedback_position,
            igris::span<int64_t> control_position)
        {
            for (size_t i = 0; i < planner->total_axes(); ++i)
            {
                int64_t drop_pulses =
                    feedback_position[i] * feedback_to_drive[i] -
                    control_position[i] * control_to_drive[i];

                if (std::abs(drop_pulses) > maximum_drop_pulses[i])
                {
                    planner->alarm_stop();
                    return;
                }
            }
        }
    };
}

#endif
