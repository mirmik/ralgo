#include <nos/print.h>
#include <ralgo/lintrans.h>
#include <ralgo/planning/premeas.h>
#include <ralgo/integrate/integrate.h>

using namespace ralgo;

int main() 
{
/*	auto T = 1;
	auto ksi = 0;

	//lintrans::zv2<double, double> ap(10, 0, 0.05);
	lintrans::integrator<double> ap;
	lintrans::pi<double,double> pi(0.2, 0.1, 0.05);

	for(int i=0;i<60;i++) 
	{
		double target = 20;

		auto sig = pi(target - ap.output());
		//PRINT(sig);
		nos::println(ap(sig));
	}
*/

	PRINT(ralgo::timepredict_trapecidal<double>(100, 10, 0, 0));
	PRINT(ralgo::timepredict_trapecidal<double>(100, 10, 1, 1));
	PRINT(ralgo::timepredict_trapecidal<double>(100, 10, 1, 2));
	PRINT(ralgo::timepredict_trapecidal<double>(100, 10, 2, 2));
	PRINT(ralgo::timepredict_trapecidal<double>(100, 10, 5, 5));
	PRINT(ralgo::timepredict_trapecidal<double>(100, 10, 7.5, 7.5));
	PRINT(ralgo::timepredict_trapecidal<double>(100, 10, 10, 10));
	PRINT(ralgo::timepredict_trapecidal<double>(100, 10, 11, 11));
	PRINT(ralgo::timepredict_trapecidal<double>(100, 10, 12, 12));
	PRINT(ralgo::timepredict_trapecidal<double>(100, 10, 13, 13));
	PRINT(ralgo::timepredict_trapecidal<double>(100, 10, 14, 14));
	PRINT(ralgo::timepredict_trapecidal<double>(100, 10, 14, 20));

	PRINT(ralgo::integrate_parabolic<double>([](double x){ return x*x*x; }, 0, 1, 10));
	PRINT(ralgo::integrate_trapecidal<double>([](double x){ return x*x*x; }, 0, 1, 100));
	PRINT(ralgo::integrate_rectangle<double>([](double x){ return x*x*x; }, 0, 1, 1000));

}