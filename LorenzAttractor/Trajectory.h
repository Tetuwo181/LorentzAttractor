#pragma once
#include<vector>
#include<tuple>
class Trajectory{
private:
	std::vector<double> trajectory;
	std::vector<double> time;
public:
	Trajectory(const double initPosition, const double initTime = 0);
	~Trajectory();
	void appendPos(const double nextTime, const double nextPos);
	void appendPos(std::tuple<double, double> nowValue);
	std::tuple<double, double> FirstPosition();
	std::tuple<double, double> NowPosition();
	std::vector<double> GetTrajectory();
	std::vector<double> GetTimeRecord();
};

