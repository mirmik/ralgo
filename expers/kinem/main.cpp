#include <ralgo/signal/fft.h>
#include <ralgo/signal/voice.h>
#include <nos/print.h>

#include <math.h>

#include <ralgo/kinematics/link.h>
#include <igris/math/deg.h>

using namespace linalg::ostream_overloads;

int main() 
{
	cynematic::rotation_link<double> rot0({0,0,1});
	cynematic::rotation_link<double> rot1({1,0,0});

	cynematic::chain<double> chain{&rot0,&rot1};

	malgo::vector<double> cur = {0,0};
	chain.solve_inverse_cynematic(linalg::mtrans<double>(
		linalg::rotation_quat<double>(linalg::vec<double,3>{1,1,0}, deg(10)), {0,0,0}), cur);
}