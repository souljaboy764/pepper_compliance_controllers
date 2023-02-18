#include "ResidualController.h"

ResidualController::ResidualController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt)
{
	config_.load(config);
	for(auto name: jointNames)
		jointIdx.push_back(robot().jointIndexByName(name));
	solver().addTask(postureTask);


	mc_rtc::log::success("ResidualController init done ");
	start = std::chrono::high_resolution_clock::now();
}

bool ResidualController::run()
{
	float resuidual = 0;
	for(int i=0; i < jointIdx.size(); i++)
		resuidual += std::abs(postureTask->posture()[jointIdx[i]][0] - robot().mbc().q[jointIdx[i]][0]);
	return mc_control::MCController::run();
}

void ResidualController::reset(const mc_control::ControllerResetData & reset_data)
{
	mc_control::MCController::reset(reset_data);
}

CONTROLLER_CONSTRUCTOR("ResidualController", ResidualController)
