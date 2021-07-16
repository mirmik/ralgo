#include <ralgo/trajectory/tsdeform.h>

int tsdeform_is_finished(tsdeform_t * tsd, float t)
{
	return t >= tsd->t3;
}

/// Вариант движения с увеличением максимальной скорости.
void tsdeform_set_time_pattern(
	tsdeform_t * tsd, 
	float t01, 
	float t23, 
	float v0, 
	float v3)
{
	tsd->full_spattern = 0;
	tsd->t01 = t01;
	tsd->t23 = t23;
	tsd->t3 = 1;

	tsd->v0 = v0;
	tsd->v3 = v3;

	// Формула выводится из равенства площадей под идеальной и реальной кривыми.
	tsd->v1 =
	    (1 - v0 * t01 / 2 - v3 * t23 / 2) /
	    (1 - t01 / 2 - t23 / 2);

	//Вычисляем коэффициенты позиции в точках окончания участков.
	tsd->x1 = (v0 + tsd->v1) * t01 / 2;
	tsd->x2 = tsd->x1 + tsd->v1 * (tsd->t3 - t01 - t23);
}

/// Второй вариант инициализации, когда вместо увеличения максимальной скорости
/// расширяется время довода изделия.
void tsdeform_set_speed_pattern(
	tsdeform_t * tsd,
    float t01,
    float t23,
    float v0,
    float v3,
    int full_spattern)
{
	float v1 = 1;

	tsd->full_spattern = full_spattern;
	tsd->t01 = t01;
	tsd->t23 = t23;

	tsd->v0 = v0;
	tsd->v3 = v3;
	tsd->v1 = v1;

	if (t01 + t23 < 2)
	{
		// Трапецидальный паттерн.
		tsd->t3 = 1
		     + (1 - v0) * t01 / 2
		     + (1 - v3) * t23 / 2;
	}
	else
	{
		// TODO
		tsd->full_spattern = 0;

		// Учёт треугольного паттерна.
		tsd->v1 = sqrt(2 / (t01 + t23));
		tsd->t01 = tsd->t01 * tsd->v1;
		tsd->t23 = tsd->t23 * tsd->v1;
		tsd->t3 = tsd->t01 + tsd->t23;
	}

	//Вычисляем коэффициенты позиции в точках окончания участков.
	tsd->x1 = (v0 + tsd->v1) * tsd->t01 / 2;
	tsd->x2 = tsd->x1 + tsd->v1
	     * (tsd->t3 - tsd->t01 - tsd->t23);

	tsd->t2 = tsd->t3 - t23;
	if (tsd->full_spattern)
	{
		tsd->x0h = t01 * (5 * v0 + v1) / 12;
		tsd->v0h = v0 / 2 + v1 / 2;
		tsd->a0h = 2 * (v1 - v0) / t01;
		tsd->b = 4 * (v1 - v0) / t01 / t01;

		tsd->x2h = -t23 * v1 / 12 - 5 * t23 * v3 / 12 + 1;
		tsd->v2h = v1 / 2 + v3 / 2;
		tsd->a2h = 2 * (v3 - v1) / t23;
		tsd->c = 4 * (v1 - v3) / t23 / t23;
	}
}

/// Интеграл коэффициента деформации
float tsdeform_posmod(tsdeform_t * tsd, float t)
{
	int full_spattern = tsd->full_spattern;

	float v0 = tsd->v0;
	float v1 = tsd->v1;
	float v3 = tsd->v3;
	float v0h = tsd->v0h;
	float v2h = tsd->v2h;
	
	float x1 = tsd->x1;
	float x2 = tsd->x2;
	float x0h = tsd->x0h;
	float x2h = tsd->x2h;

	float t2 = tsd->t2;
	float t3 = tsd->t3;
	float t01 = tsd->t01;
	float t23 = tsd->t23;

	float c = tsd->c;
	float b = tsd->b;
	float a0h = tsd->a0h;
	float a2h = tsd->a2h;

	
	if (t >= t3)
	{
		return 1;
	}

	else if (t < t01)
	{
		if (full_spattern == 0)
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
		if (full_spattern == 0)
		{
			float loct = t - t3 + t23;
			return x2
			       + (loct) * v1
			       - (loct) * ((loct) / t23 * (v1 - v3)) / 2;
		}
		else
		{
			if (t < t2 + t23 / 2)
			{
				t = t - t2;
				return x2 + v1 * t + - c * t * t * t / 6;
			}

			else
			{
				t = t - t2 - t23 / 2;
				return x2h + v2h * t + a2h * t * t / 2 + c * t * t * t / 6;
			}
		}
	}
}

/// Коэффициент деформации
float tsdeform_spdmod(tsdeform_t * tsd, float t)
{
	int full_spattern = tsd->full_spattern;

	float t3 = tsd->t3;
	float v3 = tsd->v3;
	float t01 = tsd->t01;
	float b = tsd->b;
	float c = tsd->c;

	float v0 = tsd->v0;
	float v1 = tsd->v1;
	float t2 = tsd->t2;
	float t23 = tsd->t23;

	float v0h = tsd->v0h;
	float v2h = tsd->v2h;
	float a0h = tsd->a0h;
	float a2h = tsd->a2h;

	if (t >= t3)
	{
		assert(!isnan(v3));
		return v3;
	}

	if (t < t01)
	{
		if (full_spattern == 0)
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
		if (full_spattern == 0)
		{
			float k = (t3 - t) / t23;
			return v3 * (1 - k) + v1 * k;
		}
		else
		{
			if (t < t2 + t23 / 2)
			{
				t = t - t2;
				return v1 - c * t * t / 2;
			}

			else
			{
				t = t - t2 - t23 / 2;
				return v2h + a2h * t + c * t * t / 2;
			}
		}
	}
}

/// Сбросить состояние.
void tsdeform_nullify(tsdeform_t * tsd)
{
	tsd->v0 = tsd->v3 = tsd->v1 = 0;
	tsd->t3 = 1;
	tsd->full_spattern = 0;
}

/// Паттерн остановки с линейным убыванием скорости.
void tsdeform_set_stop_pattern(tsdeform_t * tsd)
{
	tsd->full_spattern = 0;
	tsd->t01 = 0;
	tsd->t23 = 2;
	tsd->t3 = 2;

	tsd->v0 = 1;
	tsd->v3 = 0;
	tsd->v1 = 1;

	tsd->x1 = 0;
	tsd->x2 = 0;
}