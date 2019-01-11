#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include <ralgo/regulator/pi.h>

using namespace ralgo;
namespace py = pybind11;

PYBIND11_MODULE(libralgo, m)
{
	/*auto regcdelta = py::class_<regulator_const_delta<double>>(m, "regulator_const_delta");
	py::class_<pi_regulator_const_delta<double>>(m, "pi_regulator_const_delta", regcdelta)
		.def(py::init<double, double>())
		.def("__call__",(double(pi_regulator_const_delta<double>::*)(double))&pi_regulator_const_delta<double>::operator())
		.def("__call__",(double(regulator_const_delta<double>::*)(double,double))&regulator_const_delta<double>::operator())
	;

	m.def("ki_discr", &ki_discr<double>, py::arg("kp"), py::arg("kip"), py::arg("delta"));
*/

	auto inout = py::class_<lintrans::inout<double>>(m, "inout")
		.def("__call__", &lintrans::inout<double>::operator())
		.def("print_internal", &lintrans::inout<double>::print_internal)
	;
	auto inout_state = py::class_<lintrans::inout_state<double>>(m, "inout_state", inout)
		.def("output", &lintrans::inout_state<double>::output)
	;

	py::class_<lintrans::aperiodic<double,double>>(m, "aperiodic", inout_state)
		.def(py::init<double,double>(), py::arg("T"), py::arg("delta"))
		.def("__call__", &lintrans::aperiodic<double,double>::operator())
		.def("output", &lintrans::aperiodic<double,double>::output)
		.def("a", &lintrans::aperiodic<double,double>::a)
		.def("print_internal", &lintrans::aperiodic<double,double>::print_internal)
	;	
	py::class_<lintrans::oscilator<double,double>>(m, "oscilator", inout_state)
		.def(py::init<double,double,double>(), py::arg("a"), py::arg("b"), py::arg("delta"))
		.def("__call__", &lintrans::oscilator<double,double>::operator())
		.def("output", &lintrans::oscilator<double,double>::output)
		.def("a", &lintrans::oscilator<double,double>::a)
		.def("b", &lintrans::oscilator<double,double>::b)
		.def("print_internal", &lintrans::oscilator<double,double>::print_internal)
	;
	py::class_<lintrans::pi <double,double>>(m, "pi",  inout)
		.def(py::init<double,double,double>())
		.def("__call__", &lintrans::pi<double,double>::operator())
		.def("kp", &lintrans::pi<double,double>::kp)
		.def("ki", &lintrans::pi<double,double>::ki)
		.def("print_internal", &lintrans::pi<double,double>::print_internal)
	;
}