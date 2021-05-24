#ifndef RALGO_MAGNETOMETER_H
#define RALGO_MAGNETOMETER_H

#include <ralgo/linalg/linalg.h>

namespace ralgo
{
	class magnetometer_calibration
	{
		int yaw_total;
		int pitch_total; // Количество уровней без учёта вершин.

		float yaw_step;
		float pitch_step;

		float * points_array; // (size : yaw_total + pitch_total + 2)

	public:
		void init(int _yaw_total, int _pitch_total, float * _points_array)
		{
			yaw_total = _yaw_total;
			pitch_total = _pitch_total;
			points_array = _points_array;
		}

		float yaw_by_index(int y)
		{
			float k = y / yaw_total; // обход без endpoint
			return 2 * M_PI * k;
		}

		float pitch_by_index(int p)
		{
			float k = (p + 1) / (pitch_total + 2); // обход без endpoint
			return (-M_PI / 2) * (1.f - k) + (M_PI/2) * k;
		}

		linalg::vec<float, 2> angles_by_index(int idx)
		{
			if (idx == 0)
				return { 0, -M_PI / 2 };

			if (idx == yaw_total + pitch_total + 1)
				return { 0, M_PI / 2 };

			int p = (idx - 1) / yaw_total;
			int y = (idx - 1) % yaw_total;


		}

		int index_by_angles(linalg::vec<float, 2> vec)
		{

		}

	}
}

#endif