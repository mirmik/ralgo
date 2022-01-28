#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include <ralgo/cnc/defs.h>
#include <ralgo/linalg/vector_view.h>

namespace cnc 
{
	class control_task 
	{
	public:
        bool isok = false;
        std::array<double, NMAX_AXES> _poses = {};
        double feed = 0;
        double acc = 0;

        control_task() = default;

        ralgo::vector_view<double> poses(size_t total_axes) 
        {
        	ralgo::vector_view<double> ret(_poses.data(), total_axes);
        	std::copy(std::begin(_poses), std::end(_poses), ret.begin());
        	return ret;
        }

        const ralgo::vector_view<double> poses(size_t total_axes) const
        {
        	ralgo::vector_view<double> ret((double*)_poses.data(), total_axes);
        	std::copy(std::begin(_poses), std::end(_poses), ret.begin());
        	return ret;
        }

        void set_poses(igris::array_view<double> arr) 
        {
            std::copy(arr.begin(), arr.end(), _poses.begin());
        } 

        int parse(const nos::argv& argv)
        {
            for (unsigned int i = 0; i < argv.size(); ++i)
            {
                char symb = argv[i].data()[0];
                double val = atof(&argv[i].data()[1]);

                switch (tolower(symb))
                {
                case 'f':
                    feed = val;
                    continue;
                case 'm':
                    acc = val;
                    continue;
                case 'x':
                    _poses[0] = val;
                    continue;
                case 'y':
                    _poses[1] = val;
                    continue;
                case 'z':
                    _poses[2] = val;
                    continue;
                case 'a':
                    _poses[3] = val;
                    continue;
                case 'b':
                    _poses[4] = val;
                    continue;
                case 'c':
                    _poses[5] = val;
                    continue;
                case 'i':
                    _poses[6] = val;
                    continue;
                case 'j':
                    _poses[7] = val;
                    continue;
                case 'k':
                    _poses[8] = val;
                    continue;
                default:
                    return -1;
                }
            }
            return 0;
        }
	};
}

#endif