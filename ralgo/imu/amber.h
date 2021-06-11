/**
	@file
*/

#ifndef RALGO_IMU_AMBER_H
#define RALGO_IMU_AMBER_H

#include <ralgo/linalg/linalg.h>

namespace ralgo
{
	class amber
	{
	public:
		linalg::vec<float, 3> vel = {};
		linalg::vec<float, 3> pos = {};

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
			pos += vel * delta;
		}

		/// @brief            Корректирует скорость по информации внешнего источника.
		/// @param[in] extvel Информация с внешнего источника в системе си.
		/// @param[in] koeff  Коэффициент доверия к источнику в диапазоне (0,1]. 
		///                   Чем выше шум, тем меньше доверие.   
		/// @param[in] delta  Время в секундах с прошлой итерации
		void correct_vel(linalg::vec<float, 3> extvel, float koeff, float delta)
		{
			assert(koeff > 0 && koeff <= 1);
			vel += (extvel - vel) * koeff * delta;
		}

		/// @brief            Корректирует положение по информации внешнего источника.
		/// @param[in] extpos Информация с внешнего источника в системе си.
		/// @param[in] koeff  Коэффициент доверия к источнику в диапазоне (0,1]. 
		///                   Чем выше шум, тем меньше доверие.   
		/// @param[in] delta  Время в секундах с прошлой итерации
		void correct_pos(linalg::vec<float, 3> extpos, float koeff, float delta)
		{
			assert(koeff > 0 && koeff <= 1);
			pos += (extpos - pos) * koeff * delta;
		}
	};
}

#endif