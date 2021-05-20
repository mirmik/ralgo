#include <doctest/doctest.h>
#include <ralgo/madgwick.h>
#include <nos/print.h>
#include <linalg/linalg.h>

double deg(double x) { return x / 180. * M_PI; }

TEST_CASE("madgwick")
{
	ralgo::madgwick madgwick;
	madgwick.setKoeff(100, 0.005);

	madgwick.reset(linalg::rotation_quat<float>({0., 0., 1.}, 0));
	madgwick.update(0, 0, 0, 0, 0, -1, 1, 0, 0);
	CHECK_EQ(madgwick.hx, doctest::Approx(1).epsilon(0.001));
	CHECK_EQ(madgwick.hy, doctest::Approx(0).epsilon(0.001));

	madgwick.reset(linalg::rotation_quat<float>({0., 0., 1.}, deg(90)));
	madgwick.update(0, 0, 0, 0, 0, -1, 1, 0, 0);
	CHECK_EQ(madgwick.hx, doctest::Approx(0).epsilon(0.001));
	CHECK_EQ(madgwick.hy, doctest::Approx(1).epsilon(0.001));

	madgwick.reset(linalg::rotation_quat<float>({0., 0., 1.}, deg(180)));
	madgwick.update(0, 0, 0, 0, 0, -1, 1, 0, 0);
	CHECK_EQ(madgwick.hx, doctest::Approx(-1).epsilon(0.001));
	CHECK_EQ(madgwick.hy, doctest::Approx(0).epsilon(0.001));

	madgwick.reset(linalg::rotation_quat<float>({0., 0., 1.}, deg(270)));
	madgwick.update(0, 0, 0, 0, 0, -1, 1, 0, 0);
	CHECK_EQ(madgwick.hx, doctest::Approx(0).epsilon(0.001));
	CHECK_EQ(madgwick.hy, doctest::Approx(-1).epsilon(0.001));

	madgwick.reset(linalg::rotation_quat<float>({0., 0., 1.}, deg(90)));
	madgwick.apply(0, 0, 0, 0, 0, -1, 0, 0, 0);
	nos::println(madgwick.quat());

	madgwick.reset(linalg::rotation_quat<float>({0., 0., 1.}, deg(90)));
	madgwick.apply(0, 0, 0, 0, 0, -1);
	nos::println(madgwick.quat());



	nos::println("A");
	for (int i = 0; i < 100000; ++i)
	{
		madgwick.update(
		    0, 0, 0,
		    0, 0, -1,
		    0, 1, 0
		);
	}
	nos::println(madgwick.quat());
	CHECK_EQ(madgwick.quat()[3], doctest::Approx(0.70710).epsilon(0.001));
	CHECK_EQ(madgwick.quat()[2], doctest::Approx(0.70710).epsilon(0.001));


	nos::println("B");
	for (int i = 0; i < 100000; ++i)
	{
		madgwick.update(
		    0, 0, 0,
		    0, 0, -1,
		    1, 0, 0
		);
	}
	nos::println(madgwick.quat());
	CHECK_EQ(madgwick.quat()[3], doctest::Approx(1).epsilon(0.001));
	CHECK_EQ(madgwick.quat()[2], doctest::Approx(0).epsilon(0.001));

	nos::println("C");
		for (int i = 0; i < 100000; ++i)
	{
		madgwick.update(
		    0, 0, 0,
		    0, 0, -1,
		    0, -1, 0
		);
	}
	nos::println(madgwick.quat());
	CHECK_EQ(madgwick.quat()[3], doctest::Approx(0.70710).epsilon(0.001));
	CHECK_EQ(madgwick.quat()[2], doctest::Approx(0.70710).epsilon(0.001));


	nos::println("C");
	for (int i = 0; i < 100000; ++i)
	{
		madgwick.update(
		    0, 0, 0,
		    0, 0, -1,
		    -1, 0, 0
		);
	}
	nos::println(madgwick.quat());
	CHECK_EQ(madgwick.quat()[3], doctest::Approx(0).epsilon(0.001));
	CHECK_EQ(madgwick.quat()[2], doctest::Approx(1).epsilon(0.001));




}