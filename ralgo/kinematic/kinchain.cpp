#include <ralgo/kinematic/kinchain.h>
#include <nos/print.h>
#include <igris/dprint.h>

void ralgo::kinematic_chain_sensivities(
    rabbit::pose3<double> * constants,
    rabbit::screw3<double> * locsenses,
    double * coords,
    int dim,
    rabbit::screw3<double> * outsenses
)
{
	rabbit::pose3<double> temp;
	for (int i = dim-1; i >= 0; --i) 
	{
		auto W = rabbit::pose3<double>::from_screw(locsenses[i] * coords[i]);
		auto C = constants[i+1];
		temp = temp * W * C;

		outsenses[i] = locsenses[i].kinematic_carry(temp);
	}	

	temp = constants[0];
	for (int i = 0; i < dim; ++i) 
	{
		outsenses[i] = temp.rotate_screw(outsenses[i]);
	
		if (i == dim-1)
			break;

		auto W = rabbit::pose3<double>::from_screw(locsenses[i] * coords[i]);
		auto C = constants[i+1];
		temp = temp * W * C;
	}
}