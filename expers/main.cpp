#include <ralgo/regulator/pi.h>
#include <nos/print.h>

int main() {
	auto kp 		= 1;
	auto kip 		= 0.5;
	auto delta 		= 0.01;
	auto ki_discr 	= ralgo::ki_discr<double,double>(kp, kip, delta);
	auto reg 		= ralgo::pi_regulator_const_delta<double>(kp, ki_discr);

	double r=0;
	r = reg(r, 1); nos::println(r);
	r = reg(r, 1); nos::println(r);
	r = reg(r, 1); nos::println(r);
	r = reg(r, 1); nos::println(r);
	r = reg(r, 1); nos::println(r);
	r = reg(r, 1); nos::println(r);
	r = reg(r, 1); nos::println(r);
	r = reg(r, 1); nos::println(r);
	r = reg(r, 1); nos::println(r);
	r = reg(r, 1); nos::println(r);
	r = reg(r, 1); nos::println(r);
	r = reg(r, 1); nos::println(r);
	r = reg(r, 1); nos::println(r);
	r = reg(r, 1); nos::println(r);
	r = reg(r, 1); nos::println(r);
	r = reg(r, 1); nos::println(r);
	r = reg(r, 1); nos::println(r);
}