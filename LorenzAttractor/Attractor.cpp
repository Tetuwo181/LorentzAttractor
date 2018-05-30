#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include "Attractor.h"

template<typename T>
std::vector<T> DecimateVector(std::vector<T> baseVector, const uint64_t space = 0) {
	if (space == 0) {
		return baseVector;
	}
	std::vector<T> decimated{ baseVector[0] };
	for (auto index = 1; index < baseVector.size(); index ++ ) {
		if ((index%space) == 0) {
			decimated.push_back(baseVector[index]);
		}
	}
	return decimated;
}

std::function<uint64_t(uint64_t)> CheckSpace(const uint64_t space) {
	if (space == 0) {
		return [](uint64_t iteration) {return true;};
	}
	else {
		return [space](uint64_t iteration) {return (iteration%space) == 0;};
	}
}

LorenzTrajectory::LorenzTrajectory(Trajectory resultX, Trajectory resultY, Trajectory resultZ)
{
	x = resultX.GetTrajectory();
	y = resultY.GetTrajectory();
	z = resultZ.GetTrajectory();
	time = resultX.GetTimeRecord();
}

LorenzTrajectory::LorenzTrajectory(std::vector<double> resultX, std::vector<double> resultY, std::vector<double> resultZ, std::vector<double> resultTime)
{
	x = resultX;
	y = resultY;
	z = resultZ;
	time = resultTime;
}

LorenzTrajectory::~LorenzTrajectory()
{
}

std::vector<double> LorenzTrajectory::X(const uint64_t space)
{
	return DecimateVector(x, space);
}

std::vector<double> LorenzTrajectory::Y(const uint64_t space)
{
	return DecimateVector(y, space);
}

std::vector<double> LorenzTrajectory::Z(const uint64_t space)
{
	return DecimateVector(z, space);
}

std::vector<double>LorenzTrajectory::Time(const uint64_t space)
{
	return DecimateVector(time, space);
}

LorenzTrajectory LorenzTrajectory::GetDecimatedTrajectory(const uint64_t space)
{
	auto decimatedX = X(space);
	auto decimatedY = Y(space);
	auto decimatedZ = Z(space);
	auto decimatedTime = Time(space);
	return LorenzTrajectory(decimatedX, decimatedY, decimatedZ, decimatedTime);
}

Attractor::Attractor(const double pBase, const double rBase, const double bBase, const double timeWidth):p(pBase), r(rBase), b(bBase), width(timeWidth)
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


LorenzTrajectory Attractor::Fit(const double initXBase, const double initYBase, const double initZBase, const uint64_t iterateNum, const uint64_t recordSpace)
{
	RungeKutta xFunc(DxDt(initYBase, initZBase), initXBase, width);
	RungeKutta yFunc(DyDt(initXBase, initZBase), initYBase, width);
	RungeKutta zFunc(DzDt(initXBase, initYBase), initZBase, width);
	auto nowXValue = xFunc.Run(0, initXBase);
	auto nowYValue = yFunc.Run(0, initYBase);
	auto nowZValue = zFunc.Run(0, initZBase);
	auto recordChecker = CheckSpace(recordSpace);
	for (auto iteration = 0; iteration < iterateNum; iteration++) {
		auto willRecord = recordChecker(iteration);
		auto nextXValue = xFunc.ReplaceAndRun(DxDt(std::get<1>(nowYValue), std::get<1>(nowZValue)), nowXValue, willRecord);
		auto nextYValue = yFunc.ReplaceAndRun(DyDt(std::get<1>(nowXValue), std::get<1>(nowZValue)), nowYValue, willRecord);
		auto nextZValue = zFunc.ReplaceAndRun(DzDt(std::get<1>(nowXValue), std::get<1>(nowYValue)), nowZValue, willRecord);
		nowXValue = nextXValue;
		nowYValue = nextYValue;
		nowZValue = nextZValue;
	}
	return LorenzTrajectory(xFunc.GetRecord(), yFunc.GetRecord(), zFunc.GetRecord());
}


namespace py = pybind11;
PYBIND11_MODULE(LorenzAttractor, m)
{
	m.doc() = "record trajectory of lorenz attractor";

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
		.def("run", (std::tuple<double, double>(RungeKutta::*)(double, double, bool)) &RungeKutta::Run)
		.def("run", (std::tuple<double, double>(RungeKutta::*)(std::tuple<double, double>, bool)) &RungeKutta::Run)
		.def("replace_and_run", &RungeKutta::ReplaceAndRun)
		.def("get_record", &RungeKutta::GetRecord)
		.def("fit", &RungeKutta::Fit);


	py::class_<LorenzTrajectory>(m, "LorentsTrajectory")
		.def(py::init<Trajectory, Trajectory, Trajectory>())
		.def(py::init <std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>> ())
		.def("x", &LorenzTrajectory::X)
		.def("y", &LorenzTrajectory::Y)
		.def("z", &LorenzTrajectory::Z)
		.def("time", &LorenzTrajectory::Time)
		.def("get_decimated_trajectory", &LorenzTrajectory::GetDecimatedTrajectory);



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
