#ifndef RALGO_LINE_TRAJECTORY_H
#define RALGO_LINE_TRAJECTORY_H

namespace ralgo
{
    template <class Time, class Position> class LinePositionFunction
    {
        Time start_time;
        Time finish_time;
        Position start_position;
        Position finish_position;

    public:
        LinePositionFunction(Time start_time,
                             Time finish_time,
                             Position start_position,
                             Position finish_position)
            : start_time(start_time), finish_time(finish_time),
              start_position(start_position), finish_position(finish_position)
        {
        }

        Position operator()(Time time)
        {
            auto k = (time - start_time) / (finish_time - start_time);
            auto result = start_position * (1 - k) + finish_position * (k);
            return result;
        }
    };
}

#endif