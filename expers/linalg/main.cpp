#include <linalg-v3/linalg.h>
#include <linalg-v3/linalg-ext.h>
#include <igris/math/deg.h>
#include <nos/print.h>

#include <ralgo/linalg/uvec.h>

using namespace linalg::ostream_overloads;

int main() 
{
	auto q = linalg::rotation_quat(linalg::uvec<double,3>{1,0,0}, deg(20));
	auto qq = linalg::rotation_quat(linalg::uvec<double,3>{1,0,0}, deg(40));
	
	linalg::vec<double,3> v(0,0,1);
	
	auto b = q * linalg::quat<double>{v,0} * conjugate(q);
	auto bb = qq * linalg::quat<double>{v,0};

	PRINT(b);
	PRINT(bb);
}