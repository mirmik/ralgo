#include <nos/print.h>
#include <ralgo/planning/geom.h>

using namespace ralgo;

int main()
{
	ralgo::line_traj traj({0,0,1}, {1,1,2});

	nos::println(traj.tang(traj.tdist));
}
