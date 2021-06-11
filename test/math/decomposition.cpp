#include <doctest/doctest.h>

#include <ralgo/linalg/svd.h>
#include <ralgo/linalg/plud.h>
#include <ralgo/linalg/qrd.h>

#include <ralgo/linalg/linalg.h>

TEST_CASE("svd") 
{
	double arr[12];
	ralgo::matrix_view<double> mat(arr,4,3);
	ralgo::matops::copy_from_cols(mat, std::vector<std::vector<double>>
		{ {1,0,0,1}, {0,1,0,-1}, {0,0,1,0} });

	auto svd = ralgo::svd(mat);

	CHECK_EQ(svd.u.size(), 12);
	CHECK_EQ(svd.v.size(), 9);
	CHECK_EQ(svd.w.size(), 3);
}

TEST_CASE("plud") 
{
	ralgo::matrix<double> mat {
		{4,3},
		{6,3}
	};

	auto lu = ralgo::plud(mat);

	CHECK_EQ(lu.l.at(0,0), doctest::Approx(1));
	CHECK_EQ(lu.l.at(0,1), doctest::Approx(0));
	CHECK_EQ(lu.l.at(1,0), doctest::Approx(0.666666667));
	CHECK_EQ(lu.l.at(1,1), doctest::Approx(1));

	CHECK_EQ(lu.u.at(0,0), doctest::Approx(6));
	CHECK_EQ(lu.u.at(0,1), doctest::Approx(3));
	CHECK_EQ(lu.u.at(1,0), doctest::Approx(0));
	CHECK_EQ(lu.u.at(1,1), doctest::Approx(1));

	//auto res = lu.solve(std::vector<double>{7,9});
	//nos::print_list(res);
	//nos::print_matrix(lu.l);
	//nos::print_matrix(lu.u);
	//nos::print_list(ralgo::matops::multiply(lu.l, res));


	//auto mmul = ralgo::matops::mul_mv(lu.p, ralgo::matops::multiply(lu.l, lu.u));
	//CHECK_EQ(mmul, mat);
}

TEST_CASE("qrd") 
{
	ralgo::matrix<double> mat {
		{12,-51, 4},
		{6, 167, -68},
		{-4, 24, -41}
	};

	ralgo::matrix<double> res_r {
		{14,21, -14},
		{0, 175, -70},
		{0, 0, 35}
	};

	ralgo::matrix<double> res_q {
		{6./7.,-69./175., -58./175.},
		{3./7., 158./175., 6./175.},
		{-2./7., 6./35., -33./35.}
	};

	auto qr = ralgo::qrd(mat);

	CHECK(ralgo::matops::equal(qr.r, res_r, 1e-10));
	CHECK(ralgo::matops::equal(qr.q, res_q, 1e-10));

	auto mmul = ralgo::matops::multiply(qr.q, qr.r);
	CHECK_EQ(mmul, mat);
}