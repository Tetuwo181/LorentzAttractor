#include "Attractor.h"

using namespace std::placeholders;



LorentsTrajectory::LorentsTrajectory(const Trajectory resultX, const Trajectory resultY, const Trajectory resultZ)
{
	x = resultX;
	y = resultY;
	z = resultZ;
}

Trajectory LorentsTrajectory::X()
{
	return Trajectory();
}

Trajectory LorentsTrajectory::Y()
{
	return Trajectory();
}

Trajectory LorentsTrajectory::Z()
{
	return Trajectory();
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
	auto xFunc = RungeKutta(DxDt(initYBase, initZBase), initXBase, width);
	auto yFunc = RungeKutta(DxDt(initXBase, initZBase), initYBase, width);
	auto zFunc = RungeKutta(DxDt(initYBase, initYBase), initYBase, width);
}


