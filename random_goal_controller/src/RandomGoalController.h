#pragma once

#include <mc_control/mc_controller.h>
#include <vector>
#include <string>

#include "api.h"

struct RandomGoalController_DLLAPI RandomGoalController : public mc_control::MCController
{
	RandomGoalController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

	bool run() override;

	void reset(const mc_control::ControllerResetData & reset_data) override;

	void switch_target();
private:
	mc_rtc::Configuration config_;
	std::vector<int> mbcJointIdx;
	std::vector<int> refJointIdx;
	std::vector<double> rand_multiplier;
	std::vector<std::string> jointNames;// = {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"};
	std::vector<double> upper;// = {M_PI_2, -0.01, M_PI_2, 1.5, 1.8};
	std::vector<double> lower;// = {-M_PI_2, -1.5, -M_PI_2, 0.1, -1.8};
	std::map<std::string, std::vector<double>> targets;
	rbd::ForwardDynamics fd;
	double errorTolerance = 2.5;
};