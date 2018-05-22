#include "RungeKutta.h"



RungeKutta::RungeKutta(const std::function<double(double, double)> base, double initialPosition, const double baseWidth)
{
	baseFunc = base;
	width = baseWidth;
	initPos = initialPosition;
	recorder = Trajectory(initialPosition);
}


RungeKutta::~RungeKutta()
{
}

void RungeKutta::ReplaceFunc(std::function<double(double, double)> newFunc)
{
	baseFunc = newFunc;
}


std::tuple<double, double> RungeKutta::Run(const double time, const double position)
{
	/*
	現在の時間の入力から次の時間の入力の値を近似して算出する
	古典的なやり方、4次精度の近似値を出す
	*/
	auto k1 = baseFunc(time, position);
	auto k2 = baseFunc(time + (width/2.0), position + (width/2.0*k1));
	auto k3 = baseFunc(time + (width/2.0), position + (width/2.0*k2));
	auto k4 = baseFunc(time + width, position + (width + k3));
	auto nextPosition = position + (k1 + k2 + k3 + k4)*(width / 6.0);
	auto nextTime = time + width;
	auto newValue = std::make_tuple(nextTime, nextPosition);
	recorder.appendPos(newValue);
	return newValue;
}

std::tuple<double, double> RungeKutta::Run(std::tuple<double, double> nowValue)
{
	auto nowTime = std::get<0>(nowValue);
	auto nowPos = std::get<1>(nowValue);
	return Run(nowTime, nowPos);
}

Trajectory RungeKutta::GetRecord()
{
	return recorder;
}

Trajectory RungeKutta::Fit(const double initialPosition, const int iterateNum)
{
	for (auto iteration = 0; iteration < iterateNum; iteration++) {
		auto nextValue = Run(recorder.NowPosition());
		recorder.appendPos(nextValue);
	}
	return recorder;
}