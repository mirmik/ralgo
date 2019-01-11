#include <ralgo/regulator/pi.h>
#include <nos/print.h>
#include <ralgo/lintrans.h>

using namespace ralgo;

int main() 
{
	auto T = 1;
	auto ksi = 0;

	lintrans::zv2<double, double> ap(1, 0, 0.1);
	//lintrans::colleb<double, double> ap2(1, 0, 0.1);
	//lintrans::zv1<float> ap(1, 0.1);

	//for(int i=0;i<10;i++)nos::println(ap(1));

}