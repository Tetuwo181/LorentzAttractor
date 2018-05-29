#define NOMINMAX
#define _CRT_SECURE_NOWARNINGS

#include "RungeKutta.h"


RungeKutta::RungeKutta(const std::function<double(double, double)> base,
					   const double initialPosition,
	                   const double initialIime,
	                   const double baseWidth):width(baseWidth)
{
	baseFunc = base;
	recorder = Trajectory(initialPosition, initialIime);
}

RungeKutta::~RungeKutta()
{
}

void RungeKutta::ReplaceFunc(std::function<double(double, double)> newFunc)
{
	baseFunc = newFunc;
}

/*
現在の時間の入力から次の時間の入力の値を近似して算出する
古典的なやり方、4次精度の近似値を出す
*/
std::tuple<double, double> RungeKutta::Run(const double time, const double position)
{
	auto k1 = baseFunc(time, position);
	auto k2 = baseFunc(time + (width/2.0), position + (width/2.0*k1));
	auto k3 = baseFunc(time + (width/2.0), position + (width/2.0*k2));
	auto k4 = baseFunc(time + width, position + (width * k3));
	auto nextPosition = position + (k1 + 2.0*k2 + 2.0*k3 + k4)*(width / 6.0);
	auto nextTime = time + width;
	auto newValue = std::make_tuple(nextTime, nextPosition);
	recorder.AppendPos(newValue);
	return newValue;
}

std::tuple<double, double> RungeKutta::Run(std::tuple<double, double> nowValue)
{
	auto nowTime = std::get<0>(nowValue);
	auto nowPos = std::get<1>(nowValue);
	return Run(nowTime, nowPos);
}

std::tuple<double, double> RungeKutta::ReplaceAndRun(std::function<double(double, double)> newFunc, std::tuple<double, double> nowValue)
{
	ReplaceFunc(newFunc);
	return Run(nowValue);
}

Trajectory RungeKutta::GetRecord()
{
	return recorder;
}


Trajectory RungeKutta::Fit(const int iterateNum)
{
	for (auto iteration = 0; iteration < iterateNum; iteration++) {
		Run(recorder.NowPosition());
	}
	return recorder;
}

