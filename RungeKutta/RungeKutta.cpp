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
std::tuple<double, double> RungeKutta::Run(const double time, const double position, const bool willRecord)
{
	auto k1 = baseFunc(time, position);
	auto k2 = baseFunc(time + (width/2.0), position + (width/2.0*k1));
	auto k3 = baseFunc(time + (width/2.0), position + (width/2.0*k2));
	auto k4 = baseFunc(time + width, position + (width * k3));
	auto nextPosition = position + (k1 + 2.0*k2 + 2.0*k3 + k4)*(width / 6.0);
	auto nextTime = time + width;
	auto newValue = std::make_tuple(nextTime, nextPosition);
	if (willRecord) {
		recorder.AppendPos(newValue);
	}
	return newValue;
}

std::tuple<double, double> RungeKutta::Run(std::tuple<double, double> nowValue, const bool willRecord)
{
	auto nowTime = std::get<0>(nowValue);
	auto nowPos = std::get<1>(nowValue);
	return Run(nowTime, nowPos, willRecord);
}

std::tuple<double, double> RungeKutta::ReplaceAndRun(std::function<double(double, double)> newFunc, std::tuple<double, double> nowValue, bool willRecord)
{
	ReplaceFunc(newFunc);
	return Run(nowValue, willRecord);
}

Trajectory RungeKutta::GetRecord()
{
	return recorder;
}


Trajectory RungeKutta::Fit(const uint64_t iterateNum, const uint64_t recordSpace)
{
	if (recordSpace == 0) {
		//記録する間隔が0ならすべて記録
		for (auto iteration = 0; iteration < iterateNum; iteration++) {
			Run(recorder.NowPosition());
		}
	}else{
		//記録する間隔が1以上なら指定した間隔分のみ記録（メモリ対策のため）
		auto willRecord = true;
		auto nowValue = recorder.NowPosition();
		for (auto iteration = 0; iteration < iterateNum;iteration++) {
			if ((iteration%recordSpace) == 0) {
				willRecord = true;
			}
			else {
				willRecord = false;
			}
			nowValue = Run(nowValue, willRecord);
		}
	}
	return recorder;
}

