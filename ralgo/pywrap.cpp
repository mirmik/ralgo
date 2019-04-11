#include <pybind11/functional.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

//#include <ralgo/regulator/pi.h>
#include <ralgo/lintrans.h>
#include <ralgo/planning/traj.h>

using namespace ralgo;
namespace py = pybind11;

#define DOUBLE4 double, double, double, double

PYBIND11_MODULE(libralgo, m)
{
	/*auto regcdelta = py::class_<regulator_const_delta<double>>(m,
	"regulator_const_delta"); py::class_<pi_regulator_const_delta<double>>(m,
	"pi_regulator_const_delta", regcdelta) .def(py::init<double, double>())
		.def("__call__",(double(pi_regulator_const_delta<double>::*)(double))&pi_regulator_const_delta<double>::operator())
		.def("__call__",(double(regulator_const_delta<double>::*)(double,double))&regulator_const_delta<double>::operator())
	;

	m.def("ki_discr", &ki_discr<double>, py::arg("kp"), py::arg("kip"),
	py::arg("delta"));
*/

	auto inout =
		py::class_<lintrans::inout<double>>(m, "inout")
			.def("__call__", &lintrans::inout<double>::operator())
			.def("print_internal", &lintrans::inout<double>::print_internal);
	auto inout_state =
		py::class_<lintrans::inout_state<double>>(m, "inout_state", inout)
			.def("output", &lintrans::inout_state<double>::output);

	py::class_<lintrans::aperiodic<double, double>>(m, "aperiodic", inout_state)
		.def(py::init<double, double>(), py::arg("T"), py::arg("delta"))
		.def("__call__", &lintrans::aperiodic<double, double>::operator())
		.def("output", &lintrans::aperiodic<double, double>::output)
		.def("a", &lintrans::aperiodic<double, double>::a)
		.def("print_internal",
			 &lintrans::aperiodic<double, double>::print_internal);
	py::class_<lintrans::oscilator<double, double>>(m, "oscilator", inout_state)
		.def(py::init<double, double, double>(), py::arg("a"), py::arg("b"),
			 py::arg("delta"))
		.def("__call__", &lintrans::oscilator<double, double>::operator())
		.def("output", &lintrans::oscilator<double, double>::output)
		.def("a", &lintrans::oscilator<double, double>::a)
		.def("b", &lintrans::oscilator<double, double>::b)
		.def("print_internal",
			 &lintrans::oscilator<double, double>::print_internal);
	py::class_<lintrans::pi<double, double>>(m, "pi", inout)
		.def(py::init<double, double, double>())
		.def("__call__", &lintrans::pi<double, double>::operator())
		.def("kp", &lintrans::pi<double, double>::kp)
		.def("ki", &lintrans::pi<double, double>::ki)
		.def("print_internal", &lintrans::pi<double, double>::print_internal);

	py::class_<phase<double, double, double>>(m, "phase")
		.def_readwrite("d0", &phase<double, double, double>::d0)
		.def_readwrite("d1", &phase<double, double, double>::d1)
		.def_readwrite("d2", &phase<double, double, double>::d2);

	auto traj = py::class_<trajectory<DOUBLE4>>(m, "trajectory")
					.def("inloctime", &trajectory<DOUBLE4>::inloctime);
	py::class_<keep_trajectory<DOUBLE4>>(m, "keep_trajectory", traj)
		.def(py::init<double>())
		.def("inloctime_placed", &keep_trajectory<DOUBLE4>::inloctime_placed);
	py::class_<accdcc_by_time_trajectory<DOUBLE4>>(
		m, "accdcc_by_time_trajectory", traj)
		.def(py::init<double, double, double, double, double>(), py::arg("x0"),
			 py::arg("x1"), py::arg("tacc"), py::arg("tlin"), py::arg("tdcc"))
		.def("inloctime_placed",
			 &accdcc_by_time_trajectory<DOUBLE4>::inloctime_placed);
}