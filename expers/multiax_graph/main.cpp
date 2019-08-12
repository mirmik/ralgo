#include <ralgraph/show.h>
#include <ralgraph/chart.h>
#include <ralgo/planning/multiax.h>

#include <rabbit/geom/curve2.h>
#include <ralgo/vecops.h>

int main() 
{
	rabbit::pnt2<float> cp = rabbit::pnt2<float>(8,3);
	auto xp = rabbit::vec2<float>(1,4);
	auto yp = rabbit::vec2<float>(0,5); 

	PRINT(cp);
	PRINT(xp);
	PRINT(yp);

	rabbit::ellipse_curve2<float> basecrv(cp,xp,yp);
	rabbit::trimmed_curve2<float> crv{ &basecrv, 0, 1 };
	ralgo::acc_speed_deformer deformer {0.1};

	ralgo::geom2d_trajectory<> traj(&crv, &deformer, 1000);

	PRINT(basecrv.c);
	PRINT(basecrv.x);
	PRINT(basecrv.y);

	exit(0);

	auto t = ralgo::vecops::linspace<std::vector<float>>(0,1000,100);
	std::vector<float> x(t.size());
	std::vector<float> y(t.size());

	for (int i = 0; i < t.size(); ++i) 
	{
		ralgo::phase<> phs[2];

		traj.inloctime_placed(t[i],phs);

		x[i] = phs[0].d0;
		y[i] = phs[1].d0;
	}

	ralgraph::init_qt_application();

	auto chart = ralgraph::chart();

	chart.set_data(x,y);
	chart.autoscale();

	ralgraph::show(chart);
}