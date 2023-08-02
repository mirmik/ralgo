#ifndef RALGO_CNC_FEEDBACK_H
#define RALGO_CNC_FEEDBACK_H

#include <algorithm>
#include <igris/container/span.h>
#include <igris/container/static_vector.h>
#include <igris/event/delegate.h>
#include <ralgo/cnc/planner.h>

namespace cnc
{
    class feedback_guard_tandem
    {
        std::vector<size_t> _nums;
        double _maximum_tandem_mistake = 4000;

    public:
        feedback_guard_tandem(std::vector<size_t> nums) : _nums(nums) {}
        void set_maximum_tandem_mistake(double mistake)
        {
            _maximum_tandem_mistake = mistake;
        }

        double maximum_tandem_mistake() const
        {
            return _maximum_tandem_mistake;
        }

        const std::vector<size_t> &nums() const
        {
            return _nums;
        }

        std::string info() const
        {
            return nos::format(
                "{}: max_mistake:{}", _nums, _maximum_tandem_mistake);
        }
    };

    class feedback_guard
    {
    private:
        igris::delegate<void, igris::span<int64_t>>
            _set_feedback_position_callback;

        igris::delegate<void, size_t, int64_t>
            _set_feedback_position_by_axis_callback;

        std::vector<feedback_guard_tandem> _tandems = {};

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
            double default_max_drop = 6000000;
            for (size_t i = 0; i < NMAX_AXES; ++i)
            {
                maximum_drop_pulses[i] = default_max_drop;
                feedback_to_drive[i] = 1;
                control_to_drive[i] = 1;
            }
        }

        const auto &tandems() const
        {
            return _tandems;
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

        int64_t drop_pulses_allowed(int no)
        {
            return maximum_drop_pulses[no];
        }

        void set_drop_pulses_allowed(int no, int64_t val)
        {
            maximum_drop_pulses[no] = val;
        }

        // int64_t tandem_allowed_diff()
        //{
        //    return maximum_tandem_mistake;
        //}

        // void set_tandem_allowed_diff(int64_t val)
        //{
        //    maximum_tandem_mistake = val;
        //}

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
                    return false;
                }
            }
            return true;
        }

        bool verify_tandems(igris::span<int64_t> feedback_position)
        {
            if (_tandems.size() == 0)
                return true;

            for (auto &tandem : _tandems)
            {
                size_t reference_index = tandem.nums()[0];
                double reference = feedback_position[reference_index] *
                                   feedback_to_drive[reference_index];
                for (size_t i = 1; i < _tandems.size(); ++i)
                {
                    size_t index = tandem.nums()[i];
                    double pos =
                        feedback_position[index] * feedback_to_drive[index];
                    double diff = pos - reference;
                    if (std::abs(diff) > tandems()[i].maximum_tandem_mistake())
                        return false;
                }
            }
            return true;
        }

        void add_tandem(const std::vector<size_t> &tandem)
        {
            _tandems.push_back(tandem);
        }

        void remove_tandem(size_t idx)
        {
            _tandems.erase(
                std::remove_if(_tandems.begin(),
                               _tandems.end(),
                               [idx](const cnc::feedback_guard_tandem &v) {
                                   return std::count(
                                       v.nums().begin(), v.nums().end(), idx);
                               }),
                _tandems.end());
        }
    };
}

#endif
