#define NOMINMAX
#define _CRT_SECURE_NOWARNINGS

#include "Trajectory.h"


Trajectory::Trajectory()
{
}

Trajectory::Trajectory(const double initPosition, const double initTime)
{
	time.push_back(initTime);
	trajectory.push_back(initPosition);
}

Trajectory::Trajectory(const Trajectory &base)
{
	trajectory = base.trajectory;
	time = base.time;
}


Trajectory::~Trajectory()
{
}


void Trajectory::AppendPos(const double nextTime, const double nextPos)
{
	time.push_back(nextTime);
	trajectory.push_back(nextPos);
}

void Trajectory::AppendPos(std::tuple<double, double> nowValue)
{
	const auto nextTime = std::get<0>(nowValue);
	const auto nextPos = std::get<1>(nowValue);
	AppendPos(nextTime, nextPos);
}


std::tuple<double, double> Trajectory::FirstPosition()
{
	return std::make_tuple(time[0], trajectory[0]);
}

std::tuple<double, double> Trajectory::NowPosition()
{
	return std::make_tuple(time.back(), trajectory.back());
}

std::vector<double> Trajectory::GetTrajectory()
{
	return trajectory;
}

std::vector<double> Trajectory::GetTimeRecord()
{
	return time;
}

