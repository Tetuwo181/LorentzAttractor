#include "Trajectory.h"



Trajectory::Trajectory(const double initPosition, const double initTime)
{
	time.push_back(initTime);
	trajectory.push_back(initPosition);
}


Trajectory::~Trajectory()
{
}


void Trajectory::appendPos(const double nextTime, const double nextPos)
{
	time.push_back(nextTime);
	trajectory.push_back(nextPos);
}

void Trajectory::appendPos(std::tuple<double, double> nowValue)
{
	const auto nextTime = std::get<0>(nowValue);
	const auto nextPos = std::get<1>(nowValue);
	appendPos(nextTime, nextPos);
}


std::tuple<double, double> Trajectory::FirstPosition()
{
	return std::make_tuple(time[0], trajectory[0]);
}

std::tuple<double, double> Trajectory::NowPosition()
{
	return std::make_tuple(time[time.size], trajectory[trajectory.size]);
}

std::vector<double> Trajectory::GetTrajectory()
{
	return trajectory;
}

std::vector<double> Trajectory::GetTimeRecord()
{
	return time;
}

