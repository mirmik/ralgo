#include <ralgo/clinalg/matops.h>
#include <ralgo/linalg/matops.h>
#include <ralgo/linalg/matrix_view.h>

int matops_square_inverse_f(float * data,  int n, int stride, float * out) 
{
	ralgo::matrix_view_ro<float> src(data, n, n, stride);
	ralgo::matrix_view_ro<float> dst(out, n, n, stride);
	
	return ralgo::matops::square_matrix_inverse(src, dst);
}

int matops_square_inverse_d(double * data, int n, int stride, double * out) 
{
	ralgo::matrix_view_ro<double> src(data, n, n, stride);
	ralgo::matrix_view_ro<double> dst(out, n, n, stride);
	
	return ralgo::matops::square_matrix_inverse(src, dst);
} 