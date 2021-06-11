#ifndef RALGO_IMU_AMBER_H
#define RALGO_IMU_AMBER_H

namespace ralgo
{
	class amber
	{
	public:
		linalg::vec<float, 3> vel;
		linalg::vec<float, 3> pos;

		float velkoeff = 0.1;
		float poskoeff = 0.1;

	public:
		void set_pos(linalg::vec<float, 3> _pos)
		{
			pos = _pos;
		}

		void set_vel(linalg::vec<float, 3> _vel)
		{
			vel = _vel;
		}

		void step_acc(linalg::vec<float, 3> acc, float delta)
		{
			vel += acc * delta;
		}

		void step_vel(float delta)
		{
			vel += acc * delta;
		}

		void correct_vel(linalg::vec<float, 3> extvel, float koeff)
		{
			assert(koeff > 0 && koeff <= 1);
			vel += (extvel - vel) * koeff;
		}

		void correct_pos(linalg::vec<float, 3> extpos, float koeff)
		{
			assert(koeff > 0 && koeff <= 1);
			pos += (extpos - pos) * koeff;
		}
	}
}

#endif