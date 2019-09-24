#include <ralgo/matrix.h>
#include <ralgo/util/svd.h>
#include <ralgo/vecops.h>

#include <nos/print.h>

int main() 
{	
	double mma[3*2] = { 1,1,3, 3,3,1 };
	double mmb[3*2] = { 3,2,1, 1,2,3 };

	ralgo::matrix_view mat_a(mma, 3, 2);
	ralgo::matrix_view mat_b(mmb, 3, 2);

	ralgo::vecops::add_vv_to(mat_a, mat_a, mat_b);

	for (int i = 0; i < 6; ++i) 
	{
		nos::println(mma[i]);
	}


}