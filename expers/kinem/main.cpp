#include <ralgo/signal/fft.h>
#include <ralgo/signal/voice.h>
#include <nos/print.h>

#include <math.h>

#include <ralgo/linalg/htrans.h>
#include <ralgo/linalg/extension.h>
#include <ralgo/cynematic/link.h>
#include <ralgo/cynematic/chain.h>
#include <igris/math/deg.h>

using namespace ralgo::ostream_overloads;
using namespace linalg::ostream_overloads;

int main() 
{
	ralgo::cynematic::rotation_link<double> rot0({0,0,1});
	ralgo::cynematic::rotation_link<double> rot1({1,0,0});

	ralgo::cynematic::chain<double> chain{&rot0,&rot1};

	std::vector<double> cur = {0,0};
	chain.solve_inverse_cynematic(ralgo::htrans<double>(
		linalg::rotation_quat<double>(linalg::vec<double,3>{1,1,0}, deg(10)), {0,0,0}), cur);

	auto a = ralgo::htrans<double>(linalg::quat<double>{0,0,0,1},linalg::vec<double,3>{});
	auto b = ralgo::htrans<double>(linalg::quat<double>{0,0,0,1},linalg::vec<double,3>{0,0,1});
	auto c = ralgo::htrans<double>(linalg::rotation_quat<double>({0,0,1},0.1),linalg::vec<double,3>{0,0,0});

	nos::println(a);
	nos::println(b);
	nos::println(c);
	nos::println(a.vector6());
	nos::println(b.vector6());
	nos::println(c.vector6());
	nos::println(a.vector6_to(b));
	nos::println(a.vector6_to(c));
	nos::println(b.vector6_to(a));
	nos::println(c.vector6_to(a));
	nos::println(b.vector6_to(c));
	nos::println(c.vector6_to(b));
}
