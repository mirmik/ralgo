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

        igris::delegate<void, size_t, int64_t>
            _set_feedback_position_by_axis_callback;

        igris::static_vector<double, NMAX_AXES> feedback_to_drive = {};
        igris::static_vector<double, NMAX_AXES> control_to_drive =
            {}; //< этот массив равен gears

        // максимальное значение drop_pulses, после которого вызывается
        // planner->alarm_stop()
        igris::static_vector<double, NMAX_AXES> maximum_drop_pulses = {};

        cnc::planner *planner = nullptr;

    public:
        feedback_guard(cnc::planner *planner) : planner(planner)
        {
            double max_drop = 6000000;
            for (size_t i = 0; i < NMAX_AXES; ++i)
            {
                maximum_drop_pulses[i] = max_drop;
                feedback_to_drive[i] = 1;
                control_to_drive[i] = 1;
            }
        }

        void set_set_feedback_position_by_axis_callback(
            igris::delegate<void, size_t, int64_t> dlg)
        {
            _set_feedback_position_by_axis_callback = dlg;
        }

        void set_feedback_to_drive_multiplier(igris::span<double> mult)
        {
            std::copy(mult.begin(), mult.end(), feedback_to_drive.begin());
        }

        void set_control_to_drive_multiplier(igris::span<double> mult)
        {
            std::copy(mult.begin(), mult.end(), control_to_drive.begin());
        }

        void set_feedback_to_drive_multiplier(int axno, double mult)
        {
            feedback_to_drive[axno] = mult;
        }

        void set_control_to_drive_multiplier(int axno, double mult)
        {
            control_to_drive[axno] = mult;
        }

        void set_maximum_drop_pulses(igris::span<double> max_drop)
        {
            std::copy(
                max_drop.begin(), max_drop.end(), maximum_drop_pulses.begin());
        }

        void set_feedback_position(igris::span<int64_t> pos)
        {
            std::array<int64_t, NMAX_AXES> fpos;
            for (size_t i = 0; i < pos.size(); ++i)
                fpos[i] = static_cast<int64_t>(pos[i] / feedback_to_drive[i]);
            _set_feedback_position_callback(igris::span(fpos));
        }

        void set_feedback_position(size_t axno, int64_t val)
        {
            int64_t fpos = static_cast<int64_t>(val / feedback_to_drive[axno]);
            _set_feedback_position_by_axis_callback(axno, fpos);
        }

        bool verify_position(igris::span<int64_t> feedback_position,
                             igris::span<int64_t> control_position)
        {
            for (size_t i = 0; i < planner->total_axes(); ++i)
            {
                int64_t drop_pulses =
                    feedback_position[i] * feedback_to_drive[i] -
                    control_position[i] * control_to_drive[i];

                if (std::abs(drop_pulses) > maximum_drop_pulses[i])
                {
                    // std::string f = nos::format("{}", feedback_to_drive[i]);
                    // std::string c = nos::format("{}", control_to_drive[i]);
                    // ralgo::warn("feedback_guard: drop_pulses overflow");
                    // ralgo::warn("feedback_guard: feedback_to_drive:",
                    //            f.c_str());
                    // ralgo::warn("feedback_guard: control_to_drive:",
                    // c.c_str());

                    return false;
                }
            }
            return true;
        }
    };
}

#endif
