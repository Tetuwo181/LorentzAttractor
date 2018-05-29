#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include "Attractor.h"

LorentsTrajectory::LorentsTrajectory(Trajectory resultX, Trajectory resultY, Trajectory resultZ)
{
	x = resultX.GetTrajectory();
	y = resultY.GetTrajectory();
	z = resultZ.GetTrajectory();
	time = resultX.GetTimeRecord();
}

LorentsTrajectory::~LorentsTrajectory()
{
}

std::vector<double> LorentsTrajectory::X()
{
	return x;
}

std::vector<double> LorentsTrajectory::Y()
{
	return y;
}

std::vector<double> LorentsTrajectory::Z()
{
	return z;
}

std::vector<double>LorentsTrajectory::Time()
{
	return time;
}

Attractor::Attractor(const double pBase, const double bBase, const double rBase, const double timeWidth):p(pBase), b(bBase), r(rBase), width(timeWidth)
{
}

Attractor::~Attractor()
{
}

double Attractor::DxDtBase(const double t, const double x, const double y, const double z)
{
	return p*(-x+y);
}

double Attractor::DyDtBase(const double t, const double x, const double y, const double z)
{
	return x*(-z+r)-y;
}

double Attractor::DzDtBase(const double t, const double x, const double y, const double z)
{
	return x*y-b*z;
}

std::function<double(double, double)> Attractor::DxDt(const double y, const double z)
{
	return [=](double t, double x)  {return DxDtBase(t, x, y, z);};
}

std::function<double(double, double)> Attractor::DyDt(const double x, const double z)
{
	return [=](double t, double y)  {return DyDtBase(t, x, y, z);};
}

std::function<double(double, double)> Attractor::DzDt(const double x, const double y)
{
	return [=](double t, double z) {return DzDtBase(t, x, y, z);};
}

LorentsTrajectory Attractor::Fit(const double initXBase, const double initYBase, const double initZBase, const int iterateNum)
{

	RungeKutta xFunc(DxDt(initYBase, initZBase), initXBase, width);
	RungeKutta yFunc(DyDt(initXBase, initZBase), initYBase, width);
	RungeKutta zFunc(DzDt(initXBase, initYBase), initYBase, width);
	auto nowXValue = xFunc.Run(0, initXBase);
	auto nowYValue = yFunc.Run(0, initYBase);
	auto nowZValue = zFunc.Run(0, initZBase);

	for (auto iteration = 0; iteration < iterateNum; iteration++) {
		auto nextXValue = xFunc.ReplaceAndRun(DxDt(std::get<1>(nowYValue), std::get<1>(nowZValue)), nowXValue);
		auto nextYValue = yFunc.ReplaceAndRun(DyDt(std::get<1>(nowXValue), std::get<1>(nowZValue)), nowYValue);
		auto nextZValue = zFunc.ReplaceAndRun(DzDt(std::get<1>(nowXValue), std::get<1>(nowYValue)), nowZValue);
		nowXValue = nextXValue;
		nowYValue = nextYValue;
		nowZValue = nextZValue;
	}
	return LorentsTrajectory(xFunc.GetRecord(), yFunc.GetRecord(), zFunc.GetRecord());
}


namespace py = pybind11;
PYBIND11_MODULE(LorenzAttractor, m)
{
	m.doc() = "record trajectory";
	py::class_<Trajectory>(m, "Trajectory")
		.def(py::init<double, double>())
		.def("append_pos", (void (Trajectory::*)(double, double)) &Trajectory::AppendPos)
		.def("append_pos", (void (Trajectory::*)(std::tuple<double, double>)) &Trajectory::AppendPos)
		.def("first_position", &Trajectory::FirstPosition)
		.def("now_position", &Trajectory::NowPosition)
		.def("search_position_by_time", &Trajectory::SearchPositionByTime)
		.def("get_trajectory", &Trajectory::GetTrajectory)
		.def("get_time", &Trajectory::GetTimeRecord);

	py::class_<RungeKutta>(m, "RungeKutta")
		.def(py::init<std::function<double(double, double)>, const double, const double, const double>())
		.def("replace_func", &RungeKutta::ReplaceFunc)
		.def("run", (std::tuple<double, double>(RungeKutta::*)(double, double)) &RungeKutta::Run)
		.def("run", (std::tuple<double, double>(RungeKutta::*)(std::tuple<double, double>)) &RungeKutta::Run)
		.def("replace_and_run", &RungeKutta::ReplaceAndRun)
		.def("get_record", &RungeKutta::GetRecord)
		.def("fit", &RungeKutta::Fit);

	py::class_<LorentsTrajectory>(m, "LorentsTrajectory")
		.def(py::init<Trajectory, Trajectory, Trajectory>())
		.def("x", &LorentsTrajectory::X)
		.def("y", &LorentsTrajectory::Y)
		.def("z", &LorentsTrajectory::Z)
		.def("time", &LorentsTrajectory::Time);

	py::class_<Attractor>(m, "Attractor")
		.def(py::init<const double, const double, const double, const double>())
		.def("dxdt_base", &Attractor::DxDtBase)
		.def("dydt_base", &Attractor::DyDtBase)
		.def("dzdt_base", &Attractor::DzDtBase)
		.def("dxdt", &Attractor::DxDt)
		.def("dydt", &Attractor::DyDt)
		.def("dzdt", &Attractor::DzDt)
		.def("fit", &Attractor::Fit);
}
