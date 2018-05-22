#pragma once
#include<vector>
#include<functional>
#include "RungeKutta.h"

class LorentsTrajectory{
private:
	std::vector<double> x;
	std::vector<double> y;
	std::vector<double> z;
	std::vector<double> time;
public:
	LorentsTrajectory(Trajectory resultX, Trajectory resultY, Trajectory resultZ);
	~LorentsTrajectory();
	std::vector<double> X();
	std::vector<double> Y();
	std::vector<double> Z();
	std::vector<double> Time();
};

class Attractor
{
private:
	double p;
	double b;
	double r;
	double width;
public:
	Attractor(const double pBase = 0.0, const double bBase = 0.0, const double rBase = 0.0, const double timeWidth = 0.0001);
	~Attractor();
	double DxDtBase(const double t, const double x, const double y, const double z);
	double DyDtBase(const double t, const double x, const double y, const double z);
	double DzDtBase(const double t, const double x, const double y, const double z);
	//ここから先はRungeKuttaクラスで実行するためのインターフェース統一のため
	std::function<double(double, double)> DxDt(const double y, const double z);
	std::function<double(double, double)> DyDt(const double x, const double z);
	std::function<double(double, double)> DzDt(const double x, const double y);
	LorentsTrajectory Fit(const double initXBase = 0.0, const double initYBase = 0.0, const double initZBase = 0.0,  const int iterateNum = 100000);
};

