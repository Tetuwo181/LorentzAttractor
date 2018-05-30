#pragma once
#include<functional>
#include<Tuple>
#include<vector>
#include "Trajectory.h"


/*
�����Q�N�b�^�@���s���N���X
baseFunc:��͂������֐�
width:���̎��Ԃ܂ł̊Ԋu
recorder:��͌��ʂ��L�^�������
*/
class RungeKutta
{
private:
	std::function<double(double, double)> baseFunc;
	const double width;
	Trajectory recorder;
public:
	RungeKutta(const std::function<double(double, double)> base, const double initialPosition, const double initialIime, const double baseWidth = 0.0000001);
	~RungeKutta();
	void ReplaceFunc(std::function<double(double, double)> newFunc);
	std::tuple<double, double> Run(const double time, const double position, const bool willRecord = true);
	std::tuple<double, double> Run(std::tuple<double, double> nowValue, const bool willRecord = true);
	std::tuple<double, double> ReplaceAndRun(std::function<double(double, double)> newFunc, std::tuple<double, double> nowValue, const bool willRecord = true);
	Trajectory GetRecord();
	Trajectory Fit(const uint64_t iterateNum = 100000000, const uint64_t recordSpace = 0);
};

