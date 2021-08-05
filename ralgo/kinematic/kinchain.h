#ifndef RALGO_HEIMER_KINEMATIC_CHAIN_H
#define RALGO_HEIMER_KINEMATIC_CHAIN_H

#include <ralgo/space/pose3.h>
#include <ralgo/space/screw.h>

namespace ralgo 
{
	/** @brief Расчёт выходных чуствительностей кинематической цепи.
	 	@detail
	 		Кинематическая цепь задана выражением
	 		P_1^n = prod(i=[1,n], C_i W_i(x_i)) C_{n+1}

			P_1^5 = C1 W1(x1) C2 W2(x2) C3 W3(x3) C4 W4(x4) C5
			P_1^5 = P_1^2 W2(x2) P_3^5

			Угловая скорость дифференцируется по формуле Жилина,
			а скорость по формуле Бура.

			d(|R r|)     = |dR/dx dr/dx| = |S*R S*r+d'r/dx| = |S v||R r|                 / dr/dx = S*r+v
			 (|0 1|)/dx    |    0     0|   |  0          0|   |0 0||0 1|                 /

            dd(|R r|)       = |S v| d(|R r|)    = |S v||S v||R r| = |S*S S*v||R r|
			  (|0 1|)/dxdx    |0 0|  (|0 1|)/dx   |0 0||0 0||0 1|   |  0   0||0 1|

			 
			dP_1^n/dt = summ P_1^i dW_i(x_i)/dx_i P_{i+1}^n
			dP_1^n/dt = summ P_1^i S_xi W_i P_{i+1}^n
			dP_1^n/dt = summ P_1^i P_n^{i+1}* S_xi

			|R1 r1||S v||R2 r2|
			| 0  1||0 0|| 0  1|

			|R1 r1||S*R2 S*r2 + v|  --- оператор кинематического переноса
			| 0  1||   0        0|

			|R1*S*R2 R1*S*r2+R1*v|  --- поворот винта
			|      0            0|
			
		@param constants - вектор матриц C - размер dim + 1.
	    @param locsenses - вектор винтов чуствительности W - размер dim.
		@param dim - количество степеней свободы кинематической цепи.
	*/
	void kinematic_chain_sensivities(
		ralgo::pose3<double> * constants,
		ralgo::screw3<double> * locsenses,
    	double * coords,
		int dim,
		ralgo::screw3<double> * outsenses
	);
}

#endif