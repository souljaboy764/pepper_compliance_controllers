#pragma once

#include <mc_control/mc_controller.h>
#include <vector>
#include <string>

#include "api.h"
#include "Model.hpp"

struct ResidualLearningController_DLLAPI ResidualLearningController : public mc_control::MCController
{
	ResidualLearningController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

	bool run() override;

	void reset(const mc_control::ControllerResetData & reset_data) override;

private:
	void update_target(Eigen::VectorXf residual_error, std::vector<std::vector<double>> q, std::vector<std::vector<double>> q_dot);
	void switch_target();
	void addPlot();
	void removePlot();

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
	double stateTime_ = 0;
	Eigen::VectorXf residual;

	Eigen::Vector4f Kv = {0.08, 0.02, 0.02, 0.02};
	Eigen::Vector4f Kc = {17, 5, 5, 17};

	// torch::Tensor tensor
	Model model;
};