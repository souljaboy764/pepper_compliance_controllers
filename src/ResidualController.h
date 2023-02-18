#pragma once

#include <mc_control/mc_controller.h>
#include <vector>
#include <string>
#include <chrono>

#include "api.h"

struct MyFirstController_DLLAPI ResidualController : public mc_control::MCController
{
	ResidualController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

	bool run() override;

	void reset(const mc_control::ControllerResetData & reset_data) override;
private:
	mc_rtc::Configuration config_;
	std::vector<int> jointIdx;
	bool handsOpen = true;
	std::vector<std::string> jointNames = {"RWristYaw"};
	std::map<std::string, std::vector<double>> targets;
	std::chrono::time_point<std::chrono::high_resolution_clock> start;
};