#ifndef RALGO_FREQ_INVOKE_IMITATION_H
#define RALGO_FREQ_INVOKE_IMITATION_H

#include <chrono>
#include <functional>
#include <initializer_list>
#include <ranges>
#include <vector>

namespace ralgo
{
    class freq_invoke_imitation
    {
        std::vector<std::pair<std::chrono::nanoseconds, std::function<void()>>>
            invokes = {};
        std::chrono::nanoseconds minimal_interval =
            std::chrono::nanoseconds(std::numeric_limits<int64_t>::max());

    public:
        freq_invoke_imitation(
            const std::initializer_list<std::pair<std::chrono::nanoseconds,
                                                  std::function<void()>>> &list)
            : invokes(list)
        {
            for (auto &a : invokes)
            {
                if (a.first < minimal_interval)
                    minimal_interval = a.first;
            }
        }

        void timeskip(std::chrono::nanoseconds ns)
        {
            std::vector<std::chrono::nanoseconds> awake;
            std::transform(invokes.begin(),
                           invokes.end(),
                           std::back_inserter(awake),
                           [](const auto &a) { return a.first; });

            while ((ns -= minimal_interval).count() >= 0)
            {
                for (size_t i = 0; i < awake.size(); ++i)
                {
                    awake[i] -= minimal_interval;

                    if (awake[i].count() <= 0)
                    {
                        awake[i] += invokes[i].first;
                        invokes[i].second();
                    }
                }
            }
        }
    };
}

#endif