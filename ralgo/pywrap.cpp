#include <pybind11/functional.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/complex.h>
#include <pybind11/stl.h>
#include <pybind11/chrono.h>

#include <nos/print.h>
#include <nos/fprint.h>

#include <ralgo/linalg/htrans.h>

//#include <ralgo/regulator/pi.h>
#include <ralgo/lintrans.h>
#include <ralgo/planning/traj.h>

#include <ralgo/cynematic/link.h>
#include <ralgo/cynematic/chain.h>

#include <igris/math/deg.h>

using namespace ralgo;
using namespace linalg;
using namespace linalg::ostream_overloads;
using namespace ralgo::ostream_overloads;
namespace py = pybind11;

#define DOUBLE4 double, double, double, double
#define DOUBLE3 double, double, double

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


	py::class_<linalg::vec<double,3>>(m, "vec3")
		.def(py::init<DOUBLE3>())
		.def(py::init<const vec<double,3>&>());

	py::class_<linalg::mat<double,3,3>>(m, "mat33")
		.def(py::init<const mat<double,3,3>&>());

	py::class_<linalg::quat<double>>(m, "quat")
		.def(py::init<DOUBLE4>())
		.def(py::init<const quat<double>&>());

	// RALGO::LINALG
	py::class_<htrans<double>>(m, "htrans")
	    .def(py::init<>())
	    .def(py::init<const quat<double>&, const vec<double,3>&>())
		.def("__mul__", &htrans<double>::operator*)
		.def("inverse", &htrans<double>::inverse)
		.def("transform_vector", &htrans<double>::transform_vector)
		.def("transform_point", &htrans<double>::transform_point)
		.def("__str__", [](const htrans<double>& self){ return nos::format("{}", self); });

	// LINALG
	m.def("rotation_quat", (quat<double>(*)(const vec<double,3> &, double))&rotation_quat<double> );

	// CYNEMATIC
	auto alink = py::class_<cynematic::abstract_link<double>>(m, "abstract_link");

	py::class_<cynematic::rotation_link<double>>(m, "rotation_link", alink)
		.def(py::init<vec<double,3>>());

	py::class_<cynematic::translation_link<double>>(m, "translation_link", alink)
		.def(py::init<vec<double,3>>());

	py::class_<cynematic::chain<double>>(m, "chain")
		//.def(py::init<std::initializer_list<cynematic::abstract_link<double>*>>());
		.def(py::init<std::vector<cynematic::abstract_link<double>*>>())
		.def("solve_inverse_cynematic", &cynematic::chain<double>::solve_inverse_cynematic)
		.def("get", (htrans<double>(cynematic::chain<double>::*)(const std::vector<double> &coords))&cynematic::chain<double>::get);


	// IGRIS
	m.def("deg", (double(*)(double))&deg );
}