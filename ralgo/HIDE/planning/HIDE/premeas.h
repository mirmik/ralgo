#ifndef RALGO_PREMEAS_H
#define RALGO_PREMEAS_H

namespace ralgo
{
	//@acc - time
	//@dcc - time
	//@dist - distance
	//@spd - distance / time
	template <class T = double>
	T timepredict_trapecidal(T dist, T spd, T acc, T dcc)
	{
		T time_accdcc = acc + dcc;
		T dist_accdcc =
			spd * time_accdcc /
			2; //< Полное расстояние на разгонном и тормозном участках

		if (dist_accdcc < dist)
		{
			//Изделие полностью выполнит разгонные участки.
			//Эпюра имеет форму трапеции.
			T dist_line = dist - dist_accdcc;
			T time_line = dist_line / spd;
			return time_line + time_accdcc;
		}
		else
		{
			//Изделие не успеет полностью разогнаться и эпюра
			//примет форму треугольника.
			T koeff = dist / dist_accdcc;
			return time_accdcc * sqrt(koeff);
		}
	}
} // namespace ralgo

#endif