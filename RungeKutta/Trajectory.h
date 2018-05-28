#pragma once
#include<vector>
#include<tuple>

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
	std::vector<double> GetTrajectory();
	std::vector<double> GetTimeRecord();
};

