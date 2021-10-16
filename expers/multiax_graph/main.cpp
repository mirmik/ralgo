#include <ralgraph/show.h>
#include <ralgraph/chart.h>
#include <ralgo/planning/multiax.h>

#include <rabbit/geom/curve2.h>
#include <ralgo/vecops.h>

int main() 
{
	rabbit::pnt2<float> cp = rabbit::pnt2<float>(0,0);
	auto xp = rabbit::vec2<float>(1,0);
	auto yp = rabbit::vec2<float>(0,1); 

	rabbit::ellipse_curve2<float> basecrv(cp,xp,yp);
	rabbit::trimmed_curve2<float> crv{ &basecrv, 0, M_PI * 2 };
	ralgo::accdcc_speed_deformer deformer {0.8,0.2};
	//ralgo::acc_speed_deformer deformer {0.1, 0.3};

	ralgo::geom2d_trajectory<> traj(&crv, &deformer, 1);

	auto t = ralgo::vecops::linspace<std::vector<float>>(0,1,100);
	std::vector<float> x(t.size());
	std::vector<float> y(t.size());
	std::vector<float> x1(t.size());
	std::vector<float> y1(t.size());

	for (int i = 0; i < t.size(); ++i) 
	{
		ralgo::phase<> phs[2];

		traj.inloctime_placed(t[i],phs);

		x[i] = phs[0].d0;
		y[i] = phs[1].d0;
		x1[i] = phs[0].d1;
		y1[i] = phs[1].d1;
	}

	ralgraph::init_qt_application();

	auto chart = ralgraph::chart();

	auto m = ralgo::vecops::elementwise2<std::vector<float>>(
		[](float x, float y) ->float {
			return sqrt(x * x + y * y);
		}, x1, y1);

	//chart.set_data(t,m);
	chart.set_data(t,x);
	//chart.set_data(t,y);
	//chart.set_data(x,y);
	chart.autoscale();

	ralgraph::show(chart);
}
