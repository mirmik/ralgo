#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include <ralgo/regulator/pi.h>

using namespace ralgo;
namespace py = pybind11;

PYBIND11_MODULE(libralgo, m)
{
	auto regcdelta = py::class_<regulator_const_delta<double>>(m, "regulator_const_delta");
	py::class_<pi_regulator_const_delta<double>>(m, "pi_regulator_const_delta", regcdelta)
		.def(py::init<double, double>())
	;
}