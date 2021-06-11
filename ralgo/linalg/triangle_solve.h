#ifndef RALGO_LINALG_TRIANGLE_SOLVE_H
#define RALGO_LINALG_TRIANGLE_SOLVE_H

namespace ralgo
{
	// B = MX -> X
	template<class X, class M, class B>
	void L_triangle_solve(const M & a, const B & b, X&& x) 
	{
		nos::println("L_triangle_solve");
		nos::println("a:");
		nos::print_matrix(a);
		nos::println("b:");
		nos::print_list(b);

		typename M::value_type acc;

		for (int i = 0; i < a.rows(); ++i) 
		{
			acc = b[i];

			for (int j = 0; j < i; j++) 
			{
				acc -= x[j] * a.at(i,j);
				nos::println(acc);
			}

			x[i] = acc / a.at(i, i);
		}

		nos::println("\nx:");
		nos::print_list(x);
		nos::println();
	}

	/*template<class X=void, class M, class B>
	defvec_t<X> L_triangle_solve(M&& a, B&& b) 
	{
		defvec_t<X> x;
		L_triangle_solve(a, b, x);
		return x;	
	}*/	
}

#endif