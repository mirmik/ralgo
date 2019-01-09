#ifndef AUTOCONTROL_PI_H
#define AUTOCONTROL_PI_H

#include <cstdlib>

namespace ralgo
{
	template <class T>
	struct regulator_const_delta
	{
		virtual T operator()(T error) const = 0;
	};

	template <class T, class Koeff = float>
	struct pi_regulator_const_delta : public regulator_const_delta<T>
	{
		Koeff kp;
		Koeff ki_discr;

		T u;
		T e;

		pi_regulator_const_delta(Koeff _kp, Koeff _kip) : kp(_kp), ki_discr(_kip) {}

		T operator()(T error) const override
		{
			u = u + kp * (error - e) + ki_discr * error;
			e = error;
			return u;
		}
	};

	template <class T, class Time, class Koeff = float>
	struct pi_regulator
	{
		Koeff kp;
		Koeff ki_discr_delta;

		T u;
		T e;

		pi_regulator(Koeff _kp, Koeff _ki) : kp(_kp), ki_discr_delta(_kp * _ki) {}

		T operator()(T error, Time delta) const override
		{
			u = u + kp * (error - e) + ki_discr_delta * delta * error;
			e = error;
			return u;
		}
	};

	template <class T, class Time, class Koeff> T ki_discr(Koeff ki, Time delta)
	{
		return ki / delta;
	}

	template <class T, class Time, class Koeff> T ki_discr(Koeff kp, Koeff kip, Time delta)
	{
		return kp * kip / delta;
	}

	template <class T, class Koeff> void step_responce(const regulator_const_delta<T>& regul, T* responce, size_t n, T input = 1) 
	{
		for (unsigned int i = 0; i < n; ++i) 
		{
			responce[i] = regul(input);
		}
	} 

	template <class T, class Koeff> std::vector<T> step_responce(const regulator_const_delta<T>& regul, size_t n, T input = 1) 
	{
		std::vector<T> responce(n);
		return step_responce(regul, &responce[0], n, input);
	} 
}

#endif