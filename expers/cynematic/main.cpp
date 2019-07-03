#include <ralgo/signal/fft.h>
#include <ralgo/signal/voice.h>
#include <nos/print.h>

#include <math.h>

#include <ralgo/vecops.h>
#include <ralgo/signal/convolution.h>
#include <ralgo/signal/signal.h>

#include <ralgo/kinematics/link.h>

#include <igris/math/deg.h>

using namespace linalg::ostream_overloads;

int main() 
{
	cynematic::rotation_link<double> rot0({0,0,1});
	cynematic::rotation_link<double> rot1({1,0,0});

	cynematic::chain<double> chain{&rot0, &rot1};

	malgo::vector<double> reference{0,0};
	
	auto tgtq = linalg::rotation_quat<double>({0,0,1},deg(10)) * linalg::rotation_quat<double>({1,0,0},deg(20));
	//nos::println(linalg::quat<double>({0,0,1},deg(10)));
	//nos::println(tgtq);
	//nos::println(length(tgtq));

	linalg::mtrans<double> target(tgtq, {0,0,0});

	chain.solve_inverse_cynematic(target, reference);
}