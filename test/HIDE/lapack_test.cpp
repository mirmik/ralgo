#include <doctest/doctest.h>
#include <clapack.h>
#include <string.h>

TEST_CASE("blas")
{
	float x[] = { 1, 2, 3 };
	float y[] = { 2, 4, 6 };
	float r[3];


	//y[i] = a*x[i] + y[i];
	long int n = 3;
	long int incx = 1;
	long int incy = 1;

	float a = 4.0;
	saxpy_(&n, &a, x, &incx, y, &incy);

	CHECK_EQ(y[0], 6);
	CHECK_EQ(y[1], 12);
	CHECK_EQ(y[2], 18);
}

TEST_CASE("lapack")
{
	SUBCASE("1")
	{
		long int n = 1;
		long int lda = 1;
		float a[] = { 2 };
		long int ipiv[1];
		float work[1];
		long int lwork = 1;
		long int info;

		memset(ipiv, 0, sizeof(long int) * 1);
		memset(work, 0, sizeof(float) * 1);


		sgetri_(&n, a, &lda, ipiv, work, &lwork, &info);
		CHECK_EQ(a[0], 0.5);
	}

	SUBCASE("2")
	{
		long int n = 2;
		long int lda = 2;
		float a[] = { 2, 0.0f, 0.0f, 2 };
		long int ipiv[2];
		float work[100];
		long int lwork = 100;
		long int info;

		memset(ipiv, 0, sizeof(long int) * 2);
		memset(work, 0, sizeof(float) * 100);


		sgetri_(&n, a, &lda, ipiv, work, &lwork, &info);
		CHECK_EQ(a[0], 0.5);
		CHECK_EQ(a[1], doctest::Approx(0));
		CHECK_EQ(a[2], doctest::Approx(0));
		CHECK_EQ(a[3], 0.5);
	}

	/*	CHECK_EQ(ipiv[0], 1);
		CHECK_EQ(ipiv[1], 2);
		CHECK_EQ(ipiv[2], 3);
		CHECK_EQ(info, 0);
		CHECK_EQ(lda, 3);
		CHECK_EQ(work[0], 3);
		CHECK_EQ(a[0*3 + 0], 0.5);
		CHECK_EQ(a[0*3 + 1], 0);
		CHECK_EQ(a[0*3 + 2], 0);
		CHECK_EQ(a[1*3 + 0], 0);
		CHECK_EQ(a[1*3 + 1], 1./3);
		CHECK_EQ(a[1*3 + 2], 0);
		CHECK_EQ(a[2*3 + 0], 0);
		CHECK_EQ(a[2*3 + 1], 0);
		CHECK_EQ(a[2*3 + 2], 1./4);*/
}