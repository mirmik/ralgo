#ifndef AUTOCONTROL_PI_H
#define AUTOCONTROL_PI_H

#include <cstdlib>
#include <nos/print.h>

#include <ralgo/lintrans.h>

namespace ralgo {
	template <class T> struct regulator_const_delta {
		virtual T operator()(T error) = 0;

		T operator()(T current, T target) {
			return operator()(target - current);
		}
	};

	template <class T, class Koeff = float>
	struct pi_regulator_const_delta : public regulator_const_delta<T> {
		using parent = regulator_const_delta<T>;
		Koeff kp;
		Koeff ki_discr;

		// T u = 0;
		// T e = 0;
		T integral = 0;

		pi_regulator_const_delta(Koeff _kp, Koeff _ki_discr)
			: kp(_kp), ki_discr(_ki_discr) {}

		using parent::operator();
		T operator()(T error) override {
			integral += error;
			return kp * error + ki_discr * integral;
			// u = u + kp * (error - e) + ki_discr * error;
			// e = error;
			// return u;
		}
	};

	template <class T, class Time, class Koeff = float> struct pi_regulator {
		Koeff kp;
		Koeff ki_discr_delta;

		T u;
		T e;

		pi_regulator(Koeff _kp, Koeff _ki)
			: kp(_kp), ki_discr_delta(_kp * _ki) {}

		T operator()(T error, Time delta) override {
			u = u + kp * (error - e) + ki_discr_delta * delta * error;
			e = error;
			return u;
		}
	};

	// template <class Time, class Koeff=float> Koeff ki_discr(Koeff ki, Time
	// delta)
	//{
	//	return ki * delta;
	//}

	template <class Time, class Koeff = float>
	Koeff ki_discr(Koeff kp, Koeff kip, Time delta) {
		return kp * kip * delta;
	}

	template <class T, class Koeff>
	void step_responce(const regulator_const_delta<T> &regul, T *responce,
					   size_t n, T input = 1) {
		for (unsigned int i = 0; i < n; ++i) {
			responce[i] = regul(input);
		}
	}

	template <class T, class Koeff>
	std::vector<T> step_responce(const regulator_const_delta<T> &regul,
								 size_t n, T input = 1) {
		std::vector<T> responce(n);
		return step_responce(regul, &responce[0], n, input);
	}
} // namespace ralgo

#endif