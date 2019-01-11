#include <nos/print.h>
#include <ralgo/lintrans.h>

using namespace ralgo;

int main() 
{
	auto T = 1;
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

}