#include <ralgo/matrix.h>
#include <ralgo/util/svd.h>
#include <ralgo/vecops.h>

#include <nos/print.h>
#include <vector>

#include <malgo/nrecipes/svd.h>

int main() 
{	
	double mma[3*3] = { 1,1,1, 2,3,2, 3,2,3 };
	double mmb[3*3];
	double mmc[3*3];

	ralgo::matrix_view mat_a(mma, 3, 3);
	ralgo::matrix_view mat_b(mmb, 3, 3);
	ralgo::matrix_view mat_c(mmc, 3, 3);

	double need[3] = {7,9,6};

	ralgo::vector_view<double> b(need, 3);


	malgo::vector<double> bb {7,9,6};		
	malgo::vector<double> x(3);		
	std::vector<double> v(3);

	malgo::matrix<double> mat{{1,1,1}, {2,3,2}, {3,2,3}};
	malgo::SVD sss(mat);
	sss.solve(bb, x);

	nos::print_list(x);

	auto svd = ralgo::make_SVD(mat_a, mat_b, mat_c, v);
	svd.solve(b, x);


	nos::print_list(x);
}