#pragma once
#include<vector>
#include<tuple>


/*
ルンゲクッタ法を用いて解析した
軌道を記録するクラス
trajectory: 時間における位置の座標を記録
time: 時間を記録
*/
class Trajectory{
private:
	std::vector<double> trajectory;
	std::vector<double> time;
public:
	Trajectory();
	Trajectory(const double initPosition, const double initTime = 0);
	Trajectory(const Trajectory& base);
	~Trajectory();
	void AppendPos(const double nextTime, const double nextPos);
	void AppendPos(std::tuple<double, double> nowValue);
	std::tuple<double, double> FirstPosition();
	std::tuple<double, double> NowPosition();
	std::tuple<double, double> SearchPositionByTime(const double searchTime);
	std::vector<double> GetTrajectory();
	std::vector<double> GetTimeRecord();
};

