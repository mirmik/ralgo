#include <doctest/doctest.h>
#include <clapack.h>

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