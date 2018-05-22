#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "Attractor.h"

namespace py = pybind11;

PYBIND11_MODULE(WrappToPy) 
{
	py::module m("lorentz_attractor", "wrapped_py");
	py::class_<Trajectory>(m, "Trajectory")
		.def(py::init<double, double>())
		.def(py::init<Trajectory>())
		.def("append_pos", (void (Trajectory::*)(double, double)) &Trajectory::AppendPos)
		.def("append_pos", (void (Trajectory::*)(std::tuple<double, double>)) &Trajectory::AppendPos)
		.def("first_position", &Trajectory::FirstPosition)
		.def("now_position", &Trajectory::NowPosition)
		.def("get_trajectry", &Trajectory::GetTrajectory)
		.def("get_time_record", &Trajectory::GetTimeRecord);

	py::class_<RungeKutta>(m, "RungeKutta")
		.def(py::init<std::function<double(double, double)>, double, double>())
		.def("replace_func", &RungeKutta::ReplaceFunc)
		.def("run", (void (RungeKutta::*)(double, double)) &RungeKutta::Run)
		.def("run", (void (RungeKutta::*)(std::tuple<double, double>)) &RungeKutta::Run)
		.def("replace_and_run", &RungeKutta::ReplaceAndRun)
		.def("get_record", &RungeKutta::GetRecord)
		.def("fit", &RungeKutta::Fit);

	return m;
}

