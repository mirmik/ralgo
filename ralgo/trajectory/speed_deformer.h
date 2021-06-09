#ifndef RALGO_SPEED_DEFORMATION_H
#define RALGO_SPEED_DEFORMATION_H

/**
	@file
*/

#include <math.h>
#include <assert.h>

#include <igris/dprint.h>

/**
	Деформаторы скорости нужны для того,
	чтобы наложить скоростной паттерн на
	траекторию описываемую уравнением s(t).

	Деформатор работает в условных единицах:
	x0 = 0
	x3 = 1
	t0 = 0

	Для режима с фиксированной скоростью: v1 = 1
	Для режима с фиксированной временем:  t3 = 1

	Входными параметрами для деформатора являются
	относительные входные, выходные скорости, относительные
	временные интервалы участков.
	Ускорения выражаются через входные параметры.

	Деформатор скорости D следует применять как:
	u = t / (t_fini-t_strt) - время в частях.
	x = s(D_pos(u) * t)
	v = D_spd(u) \dot{s}(D_pos(u) * t)

	Для траектории в виде отрезка с постоянной скоростью:
	\dot{s(t)} = const = V
	=>
	v = D_spd(u) * V

Основная форма (без учёта S паттерна):

	NOTE:
	График нарисован для случая, когда v0 = v3 = 0,
	хотя в алгоритме v0 и v3 могут не быть равны 0.


    t0    t1                       t2    t3
----|-----|-------------------------|-----|----
	                                          Ускорение
	|     |                         |     |
  a  _____
	|     |                         |     |
    |     |_________________________
                                    |     |
    |     |                         |_____| -d

    |     |                         |     |
--->--t01--<----------t12---------->--t23--<---
    |     |                         |     |
                                              Скорость
    |  v1 |_________________________| v2  |
         /                           \
    |   / |                         | \   |
       /                               \
    | /   |                         |   \ |
  v0 /                                   \ v3
----|-----|-------------------------|-----|---
                                              Положение
    |     |                         |     |

    |     |                         |    _| x3
                                 x2   .""
    |     |                         |"    |
                                   /
    |     |                       / |     |
                                 /
    |     |     . . . . . . . . /   |     |
              /
    |     |  /                      |     |
            /
    |     |/                        |     |
         .  x1
 x0 |_.." |                         |     |
-----------------------------------------------

	Уравнения движения без учёта S паттерна:
	                                a[i]*t[i:i+1]**2
	x[i+1] = x[i] + v[i]*t[i:i+1] + ----------------
	                                       2
	v[i+1] = v[i] + a[i]*t[i:i+1]


3. Учёт S паттерна.
   При учёте S паттерна основной расчёт, приведённый выше
   не изменяется, но меняются графики участков с ускорениями.
   S строится с тем, чтобы выходные скорость и положение
   не изменялись. При этом ускорение увеличивается до двух раз
   при полном S паттерне.

   График полного S паттерна:

    t0    t0h   t1
----|-----|-----|--
	 _____          Скорость изменения ускорения.
  b |     |     |
    |     |
          |     |
    |     |_____| -b

----|-----|-----|--
                    Ускорение.
    |           |
        a0h=2a
    |     .     |
         / \
    |   / | \   |
       /     \
    | /   |   \ |
a0=0 /         \  a1=0
----|-----|-----|--
                    Скорость.
    |     |     |
              __  v1
    |  v0h .""  |
          /
 v0 |__.."      |

----|-----|-----|
                    Положение.
    |     |     |

    |     |     |
               / x1
    |  x0h  _." |
  x0 ___..."
----|-----|-----|

*/

namespace ralgo
{
	class speed_deformer
	{
	public:
		static constexpr float t0 = 0;
		float t01 = 0;
		float t23 = 0;
		float t3 = 1;

		float v0 = 0;
		float v3 = 0;
		float v1 = 0;
		// v2 = v1;

		static constexpr float x0 = 0;
		float x1 = 0;
		float x2 = 0;
		static constexpr float x3 = 1;

		bool full_spattern = false;
		float t2 = 0;
		float x0h, x2h;
		float v0h, v2h;
		float a0h, a2h;
		float b, c;



	public:
		bool is_finished(float t)
		{
			return t >= t3;
		}

		// Вариант движения с увеличением максимальной скорости.
		void set_time_pattern(float t01, float t23, float v0 = 0, float v3 = 0)
		{
			this->full_spattern = false;
			this->t01 = t01;
			this->t23 = t23;
			this->t3 = 1;

			this->v0 = v0;
			this->v3 = v3;

			// Формула выводится из равенства площадей под идеальной и реальной кривыми.
			v1 =
			    (1 - v0 * t01 / 2 - v3 * t23 / 2) /
			    (1 - t01 / 2 - t23 / 2);

			//Вычисляем коэффициенты позиции в точках окончания участков.
			x1 = (v0 + v1) * t01 / 2;
			x2 = x1 + v1 * (t3 - t01 - t23);
		}

		// Второй вариант инициализации, когда вместо увеличения максимальной скорости
		// расширяется время довода изделия.
		void set_speed_pattern(
		    float t01,
		    float t23,
		    float v0 = 0,
		    float v3 = 0,
		    bool full_spattern = false)
		{
			this->full_spattern = full_spattern;
			this->t01 = t01;
			this->t23 = t23;

			this->v0 = v0;
			this->v3 = v3;
			this->v1 = 1;

			if (t01 + t23 < 2)
			{
				// Трапецидальный паттерн.
				t3 = 1
				     + (1 - v0) * t01 / 2
				     + (1 - v3) * t23 / 2;
			}
			else
			{
				// TODO
				this->full_spattern = false;

				// Учёт треугольного паттерна.
				v1 = sqrt(2 / (t01 + t23));
				this->t01 = t01 * v1;
				this->t23 = t23 * v1;
				t3 = this->t01 + this->t23;
			}

			//Вычисляем коэффициенты позиции в точках окончания участков.
			x1 = (v0 + v1) * this->t01 / 2;
			x2 = x1 + v1
			     * (t3 - this->t01 - this->t23);

			this->t2 = t3 - t23; 
			if (full_spattern)
			{
				x0h = t01 * (5 * v0 + v1) / 12;
				v0h = v0 / 2 + v1 / 2;
				a0h = 2 * (v1 - v0) / t01;
				b = 4 * (v1 - v0) / t01 / t01;

				x2h = -t23 * v1 / 12 - 5 * t23 * v3 / 12 + 1;
				v2h = v1 / 2 + v3 / 2;
				a2h = 2 * (v3 - v1) / t23;
				c = 4 * (v1 - v3) / t23 / t23;
			}
		}

		// Интеграл коэффициента деформации
		float posmod(float t)
		{
			if (t >= t3)
			{
				return 1;
			}

			else if (t < t01)
			{
				if (full_spattern == false)
				{
					return
					    t * v0
					    + t * t * (v1 - v0) / (2 * t01);
				}
				else
				{
					if (t < t01 / 2)
					{
						t = t;
						return v0 * t + b * t * t * t / 6;;
					}
					else
					{
						t = t - t01 / 2;
						return x0h + v0h * t + a0h * t * t / 2 - b * t * t * t / 6;
					}
				}
			}

			else if (t < t3 - t23)
			{
				return
				    x1
				    + v1 * (t - t01);
			}

			else
			{
				if (full_spattern == false)
				{
					auto loct = t - t3 + t23;
					return x2
					       + (loct) * v1
					       - (loct) * ((loct) / t23 * (v1 - v3)) / 2;
				}
				else
				{
					if (t < t2 + t23/2) 
					{
						t = t - t2;
						return x2 + v1 * t + - c*t*t*t/6;
					}

					else 
					{
						t = t - t2 - t23/2;
						return x2h + v2h * t + a2h*t*t/2 + c*t*t*t/6;
					}
				}
			}
		}

		// Коэффициент деформации
		float spdmod(float t)
		{
			if (t >= t3)
			{
				assert(!isnan(v3));
				return v3;
			}

			if (t < t01)
			{
				if (full_spattern == false)
				{
					float k = t / t01;
					return v0 * (1 - k) + v1 * k;
				}
				else
				{
					if (t < t01 / 2)
					{
						t = t;
						return b * t * t / 2;
					}
					else
					{
						t = t - t01 / 2;
						return v0h + a0h * t - b * t * t / 2;
					}
				}
			}

			else if (t < t3 - t23)
			{
				return v1;
			}

			else
			{
				if (full_spattern == false)
				{
					float k = (t3 - t) / t23;
					return v3 * (1 - k) + v1 * k;
				}
				else
				{
					if (t < t2 + t23/2) 
					{
						t = t - t2;
						return v1 - c*t*t/2;
					}

					else 
					{
						t = t - t2 - t23/2;
						return v2h + a2h*t + c*t*t/2;
					}
				}
			}
		}

		// Сбросить состояние.
		void nullify()
		{
			v0 = v3 = v1 = 0;
			t3 = 1;
			full_spattern = false;
		}

		// Паттерн остановки с линейным убыванием скорости.
		void set_stop_pattern()
		{
			full_spattern = false;
			t01 = 0;
			t23 = 2;
			t3 = 2;

			v0 = 1;
			v3 = 0;
			v1 = 1;

			x1 = 0;
			x2 = 0;
		}
	};
}

#endif