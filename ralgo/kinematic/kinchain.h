#ifndef RALGO_HEIMER_KINEMATIC_CHAIN_H
#define RALGO_HEIMER_KINEMATIC_CHAIN_H

#include <rabbit/space/pose3.h>
#include <rabbit/space/screw.h>

namespace ralgo 
{
	/** @brief Расчёт выходных чуствительностей кинематической цепи.
	 	@detail
	 		Кинематическая цепь задана выражением
	 		P_1^n = prod(i=[1,n], C_i W_i(x_i)) C_{n+1}

			P_1^5 = C1 W1(x1) C2 W2(x2) C3 W3(x3) C4 W4(x4) C5
			P_1^5 = C1 W1(x1) C2 W2(x2) C3 W3(x3) C4 W4(x4) C5

			P_1^5 = P_1^2 W2(x2) P_3^5
			P_1^5 = P_1^2 S_x2 W2 P_3^5

			P_1^5 = P_1^2 (P_5^2* S_x2^-1)^-1
			P_1^5 = P_1^2 P_5^2* S_x2
			h_2 = P_1^2 P_5^2* w_2

			dP_1^n/dt = summ P_1^i dW_i(x_i)/dx_i P_{i+1}^n
			dP_1^n/dt = summ P_1^i S_xi W_i P_{i+1}^n
			dP_1^n/dt = summ P_1^i P_n^{i+1}* S_xi

			|R1 r1||w v||R2 r2|
			| 0  1||0 0|| 0  1|

			|R1 r1||w*R2 w*r2 + v|  --- оператор кинематического переноса
			| 0  1||   0        0|

			|R1*R2*w R1*r2*w+R1*v|  --- поворот винта
			|      0            0|
			
		@param constants - вектор матриц C - размер dim + 1.
	    @param locsenses - вектор винтов чуствительности W - размер dim.
		@param dim - количество степеней свободы кинематической цепи.
	*/
	void kinematic_chain_sensivities(
		rabbit::pose3<double> * constants,
		rabbit::screw3<double> * locsenses,
    	double * coords,
		int dim,
		rabbit::screw3<double> * outsenses
	);
}

#endif