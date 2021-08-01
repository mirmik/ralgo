#include <doctest/doctest.h>
#include <ralgo/kinematic/kinchain.h>

#include <iostream>
#include <igris/dprint.h>

namespace linalg { using linalg::ostream_overloads::operator <<; }

TEST_CASE("kinchain.1")
{
	rabbit::pose3<double> constants[3] = { {}, {}, {} };
	rabbit::screw3<double> locsenses[2] = { {{0,0,0}, {0,0,1}}, {{0,0,0}, {0,0,0}} };
	double coords[2] = { 0, 0 };
	rabbit::screw3<double> outsenses[2];

	ralgo::kinematic_chain_sensivities(
	    constants,
	    locsenses,
	    coords,
	    2,
	    outsenses
	);

	CHECK_EQ(outsenses[0].lin, linalg::vec<double,3>{0,0,1});
	CHECK_EQ(outsenses[0].ang, linalg::vec<double,3>{0,0,0});
	CHECK_EQ(outsenses[1].lin, linalg::vec<double,3>{0,0,0});
	CHECK_EQ(outsenses[1].ang, linalg::vec<double,3>{0,0,0});
}

TEST_CASE("kinchain.2")
{
	rabbit::pose3<double> constants[3] = { rabbit::mov3<double>({1,1,1}), {}, {} };
	rabbit::screw3<double> locsenses[2] = { {{0,0,0}, {0,0,1}}, {{0,0,0}, {0,0,0}} };
	double coords[2] = { 0, 0 };
	rabbit::screw3<double> outsenses[2];

	ralgo::kinematic_chain_sensivities(
	    constants,
	    locsenses,
	    coords,
	    2,
	    outsenses
	);

	CHECK_EQ(outsenses[0].lin, linalg::vec<double,3>{0,0,1});
	CHECK_EQ(outsenses[0].ang, linalg::vec<double,3>{0,0,0});
	CHECK_EQ(outsenses[1].lin, linalg::vec<double,3>{0,0,0});
	CHECK_EQ(outsenses[1].ang, linalg::vec<double,3>{0,0,0});
}

TEST_CASE("kinchain.3")
{
	rabbit::pose3<double> constants[2] = { rabbit::mov3<double>({0,0,0}), rabbit::mov3<double>({1,0,0}) };
	rabbit::screw3<double> locsenses[1] = { {{0,0,1}, {0,0,0}} };
	double coords[1] = { 0 };
	rabbit::screw3<double> outsenses[1];

	ralgo::kinematic_chain_sensivities(
	    constants,
	    locsenses,
	    coords,
	    1,
	    outsenses
	);

	CHECK_EQ(outsenses[0].lin, linalg::vec<double,3>{0,1,0});
	CHECK_EQ(outsenses[0].ang, linalg::vec<double,3>{0,0,1});
}

TEST_CASE("kinchain.4")
{
	rabbit::pose3<double> constants[2] = { rabbit::mov3<double>({0,0,0}), rabbit::mov3<double>({1,0,0}) };
	rabbit::screw3<double> locsenses[1] = { {{0,0,1}, {0,0,0}} };
	double coords[1] = { M_PI/2 };
	rabbit::screw3<double> outsenses[1];

	ralgo::kinematic_chain_sensivities(
	    constants,
	    locsenses,
	    coords,
	    1,
	    outsenses
	);

	CHECK_EQ(outsenses[0].lin[0], doctest::Approx(-1));
	CHECK_EQ(outsenses[0].lin[1], doctest::Approx(0));
	CHECK_EQ(outsenses[0].lin[2], doctest::Approx(0));
	CHECK_EQ(outsenses[0].ang, linalg::vec<double,3>{0,0,1});
}