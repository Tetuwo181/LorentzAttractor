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


/*
ì¸óÕÇµÇΩéûä‘Ç©ÇÁàÍî‘ãﬂÇ¢éûä‘ÇÃ
ç¿ïWÇíTÇ∑
*/
std::tuple<double, double> Trajectory::SearchPositionByTime(const double searchTime)
{
	auto position = trajectory[0];
	auto dtime = abs(searchTime - time[0]);
	auto nearstTime = time[0];
	for (auto index = 0; index < time.size(); index++) {
		auto nowDtime = abs(searchTime - time[index]);
		if (nowDtime < dtime) {
			dtime = nowDtime;
			position = trajectory[index];
			nearstTime = time[index];
		}
	}
	return std::make_tuple(nearstTime, position);
}

std::vector<double> Trajectory::GetTrajectory()
{
	return trajectory;
}

std::vector<double> Trajectory::GetTimeRecord()
{
	return time;
}

