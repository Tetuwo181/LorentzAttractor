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

Attractor::Attractor(const double pBase, const double bBase, const double rBase, const double timeWidth)
{
	p = pBase;
	b = bBase;
	r = rBase;
	width = timeWidth;
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
	return [=](double t, double x) {return DxDtBase(t, x, y, z);};
}

std::function<double(double, double)> Attractor::DyDt(const double x, const double z)
{
	return [=](double t, double y) {return DyDtBase(t, x, y, z);};
}

std::function<double(double, double)> Attractor::DzDt(const double x, const double y)
{
	return [=](double t, double z) {return DzDtBase(t, x, y, z);};
}

LorentsTrajectory Attractor::Fit(const double initXBase, const double initYBase, const double initZBase, const int iterateNum)
{

	RungeKutta xFunc(DxDt(initYBase, initZBase), initXBase, width);
	RungeKutta yFunc(DxDt(initXBase, initZBase), initYBase, width);
	RungeKutta zFunc(DxDt(initYBase, initYBase), initYBase, width);
	auto nowXValue = xFunc.Run(0, initXBase);
	auto nowYValue = yFunc.Run(0, initYBase);
	auto nowZValue = zFunc.Run(0, initZBase);

	for (auto iteration = 0; iteration < iterateNum; iteration++) {
		auto nextXValue = xFunc.ReplaceAndRun(DxDt(std::get<1>(nowYValue), std::get<1>(nowZValue)), nowXValue);
		auto nextYValue = xFunc.ReplaceAndRun(DxDt(std::get<1>(nowXValue), std::get<1>(nowZValue)), nowYValue);
		auto nextZValue = xFunc.ReplaceAndRun(DxDt(std::get<1>(nowXValue), std::get<1>(nowYValue)), nowZValue);
		nowXValue = nextXValue;
		nowYValue = nextYValue;
		nowZValue = nextZValue;
	}
	return LorentsTrajectory(xFunc.GetRecord(), yFunc.GetRecord(), zFunc.GetRecord());
}


