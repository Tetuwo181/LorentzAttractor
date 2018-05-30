#pragma once
#include<vector>
#include<functional>
#include "RungeKutta.h"


class LorenzTrajectory{
private:
	std::vector<double> x;
	std::vector<double> y;
	std::vector<double> z;
	std::vector<double> time;
public:
	LorenzTrajectory(Trajectory resultX, Trajectory resultY, Trajectory resultZ);
	LorenzTrajectory(std::vector<double> resultX, std::vector<double> resultY, std::vector<double> resultZ, std::vector<double> resultTime);
	~LorenzTrajectory();
	std::vector<double> X(const uint64_t space = 0);
	std::vector<double> Y(const uint64_t space = 0);
	std::vector<double> Z(const uint64_t space = 0);
	std::vector<double> Time(const uint64_t space = 0);
	LorenzTrajectory GetDecimatedTrajectory(const uint64_t space);
};

class Attractor
{
private:
	const double p;
	const double b;
	const double r;
	const double width;
public:
	Attractor(const double pBase = 10.0, const double rBase = 28.0, const double bBase = 8.0/3.0, const double timeWidth = 0.0001);
	~Attractor();
	double DxDtBase(const double t, const double x, const double y, const double z);
	double DyDtBase(const double t, const double x, const double y, const double z);
	double DzDtBase(const double t, const double x, const double y, const double z);
	//ここから先はRungeKuttaクラスで実行するためのインターフェース統一のため
	std::function<double(double, double)> DxDt(const double y, const double z);
	std::function<double(double, double)> DyDt(const double x, const double z);
	std::function<double(double, double)> DzDt(const double x, const double y);
	//ここから先はアルゴリズムを回す
	LorenzTrajectory Fit(const double initXBase = 0.0, const double initYBase = 0.0, const double initZBase = 0.0,  const uint64_t iterateNum = 100000, const uint64_t recordSpace = 0);
};

