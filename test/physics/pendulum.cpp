#include <doctest/doctest.h>
#include <ralgo/filter/moment_servo_filter.h>

double acceleration(double phi, double omega) 
{
	double I = 1;
	double D = 0.1;
	double m = 1;
	double g = 9.81;
	double R = 1;

	return - D/I*omega - m*g*R*sin(phi);
}

TEST_CASE("moment_servo_filter") 
{
	double phi = 1;
	double omega = 0;

	for(int i = 0; i < 100; ++i) 
	{

	}

	
}
