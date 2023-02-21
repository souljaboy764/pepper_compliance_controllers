#include <mc_rtc/gui/plot.h>
#include <ros/ros.h>
#include <check_selfcollision/SelfCollosion.h>
#include <check_selfcollision/SelfCollosionRequest.h>
#include <check_selfcollision/SelfCollosionResponse.h>

#include "ResidualLearningController.h"

#include <cstdlib>
#include <cmath>
#include <chrono>
#include <thread>
#include <fstream>

ResidualLearningController::ResidualLearningController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt)
{
	config_.load(config);
	int seed = 1;
	config_("ControllerParams")("RandomSeed", seed);
	std::srand(seed);
	config_("ControllerParams")("JointNames", jointNames);
	config_("ControllerParams")("UpperLimits", upper);
	config_("ControllerParams")("LowerLimits", lower);
	config_("ControllerParams")("AngleErrorTolerance", errorTolerance);
	std::vector<std::string> refJointNames = robot().refJointOrder();
	for(int i=0; i <jointNames.size();i++)
	{
		std::string name = jointNames[i];
		int idx = robot().jointIndexByName(name);
		mbcJointIdx.push_back(idx);
		auto refIdx = std::find(refJointNames.begin(), refJointNames.end(), name);
		if(refIdx == refJointNames.end())
			mc_rtc::log::error_and_throw<std::runtime_error>("ResidualLearningController:: wrong joint nameL {}", name);
		refJointIdx.push_back(refIdx - refJointNames.begin());
		rand_multiplier.push_back((upper[i] - lower[i])/RAND_MAX);
		mc_rtc::log::info("ResidualLearningController: Joint[{}] name: \"{}\", mbc index: {}, ref index {}", i, name, idx, refIdx - refJointNames.begin());
	}
	residual.resize(mbcJointIdx.size());
	int argc = 0;
	ros::init(argc, std::vector<char*>({""}).data(), "ResidualLearningController", ros::init_options::NoSigintHandler);
	mc_rtc::log::info("ResidualLearningController: ROS {}", ros::ok());
  	
	solver().addConstraintSet(selfCollisionConstraint);
	switch_target();
	solver().addTask(postureTask);
	
	fd = rbd::ForwardDynamics(robot().mb());
	addPlot();
	mc_rtc::log::success("ResidualLearningController: init done");
}

bool ResidualLearningController::run()
{
	std::vector<std::vector<double>> target = postureTask->posture();
	std::vector<double> encoderValues = robot().encoderValues();	
	fd.computeH(robot().mb(), robot().mbc());
	fd.computeC(robot().mb(), robot().mbc());
	std::vector<std::vector<double>> q_ddot = robot().mbc().alphaD, q_dot = robot().mbc().alpha, q = robot().mbc().q;
	Eigen::VectorXf tau = (fd.H() * rbd::dofToVector(robot().mb(), q_ddot) + fd.C()).cast<float>();

	float dist = 0;
	Eigen::VectorXf model_input(20);
	for(int i=0; i < mbcJointIdx.size(); i++)
	{
		dist += std::abs(target[mbcJointIdx[i]][0] - encoderValues[refJointIdx[i]]);
		/* model_input[i] = encoderValues[refJointIdx[i]];
		model_input[4+i] = q[mbcJointIdx[i]][0];
		model_input[8+i] = q_dot[mbcJointIdx[i]][0];
		model_input[12+i] = q_ddot[mbcJointIdx[i]][0];
		model_input[16+i] = tau[mbcJointIdx[i]]; */
		residual[i] = q[mbcJointIdx[i]][0] - encoderValues[refJointIdx[i]];
	}
	/* dist /= mbcJointIdx.size();
	if(dist < errorTolerance*M_PI/180.)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(2000));
		// switch_target();
	} */
	// Eigen::VectorXf pred = model(model_input);
	// Eigen::VectorXf residual_error = residual - pred;
	update_target(residual, q, q_dot);
	mc_rtc::log::info("ResidualLearningController Residual: {:4f}", float(residual[0]));
	stateTime_ = stateTime_ + solver().dt();
	return mc_control::MCController::run();
}

void ResidualLearningController::update_target(Eigen::VectorXf residual_error, std::vector<std::vector<double>> q, std::vector<std::vector<double>> q_dot)
{
	std::string modified_idx("");
	for(int i=0; i < jointNames.size(); i++)
	{
		if(std::abs(residual_error[i]) >= M_PI/180.)
		{
			targets[jointNames[i]] = {q[mbcJointIdx[i]][0] - Kc[i]*residual_error[i] + Kv[i]*q_dot[mbcJointIdx[i]][0]};
			modified_idx += jointNames[i];
			modified_idx += " ";
		}
	}
	postureTask->target(targets);
	mc_rtc::log::info("ResidualLearningController: Modified Posture Targets for " + modified_idx);
}

void ResidualLearningController::reset(const mc_control::ControllerResetData & reset_data)
{
	mc_control::MCController::reset(reset_data);
}

void ResidualLearningController::switch_target()
{
	bool is_colliding = true;
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<check_selfcollision::SelfCollosion>("check_selfcollision_service");
	check_selfcollision::SelfCollosionRequest request;
	check_selfcollision::SelfCollosionResponse response;
	// std::vector<double> target {M_PI_2, -0.15, M_PI_2, 0.009};
	std::vector<double> target {0.01, 0.};
	while(is_colliding)
	{
		request.robot_state.joint_state.position.clear();
		request.robot_state.joint_state.name.clear();
		for(int i=0; i < jointNames.size(); i++)
		{
			targets[jointNames[i]] = {target[i]};//{lower[i] + rand_multiplier[i]*((double) rand())};
			request.robot_state.joint_state.name.push_back(jointNames[i]);
			request.robot_state.joint_state.position.push_back(targets[jointNames[i]][0]);
		}
		if(client.call(request, response))
			is_colliding = response.is_colliding;
	}
	postureTask->target(targets);
	mc_rtc::log::success("ResidualLearningController: Switch Target");
}


void ResidualLearningController::addPlot()
{
  // Plot filtered residual
  gui()->addPlot("Contact detection",
    mc_rtc::gui::plot::X("Time (s)", [this]() { return stateTime_; }),
    mc_rtc::gui::plot::Y("Residual", [this]() { return residual[0]; }, mc_rtc::gui::Color::Blue)
    // mc_rtc::gui::plot::Y("+Threshold", [this]() { return residualThreshold_; }, Color::Red),
    // mc_rtc::gui::plot::Y("-Threshold", [this]() { return -residualThreshold_; }, Color::Red)
  );

  // Plot measured and predicted errors
  /* gui()->addPlot("Joint position tracking error",
    mc_rtc::gui::plot::X("Time (s)", [this]() { return stateTime_; }),
    mc_rtc::gui::plot::Y("err", [this]() { return err_; }, mc_rtc::gui::Color::Black),
    mc_rtc::gui::plot::Y("expErr", [this]() { return errExp_; }, mc_rtc::gui::Color::Green)
  ); */
}

void ResidualLearningController::removePlot()
{
  gui()->removePlot("Contact detection");
  gui()->removePlot("Joint position tracking error");
}

CONTROLLER_CONSTRUCTOR("ResidualLearningController", ResidualLearningController)
