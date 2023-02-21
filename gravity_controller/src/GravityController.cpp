#include <ros/ros.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include "GravityController.h"

#include <cstdlib>
#include <cmath>
#include <chrono>
#include <thread>
#include <fstream>

GravityController::GravityController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
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
			mc_rtc::log::error_and_throw<std::runtime_error>("GravityController:: wrong joint nameL {}", name);
		refJointIdx.push_back(refIdx - refJointNames.begin());
		rand_multiplier.push_back((upper[i] - lower[i])/RAND_MAX);
		mc_rtc::log::info("GravityController: Joint[{}] name: \"{}\", mbc index: {}, ref index {}", i, name, idx, refIdx - refJointNames.begin());
	}

	int argc = 0;
	ros::init(argc, std::vector<char*>({""}).data(), "GravityController", ros::init_options::NoSigintHandler);
	mc_rtc::log::info("GravityController: ROS {}", ros::ok());
  	
	solver().addConstraintSet(selfCollisionConstraint);
	switch_target();
	solver().addTask(postureTask);
	
	fd = rbd::ForwardDynamics(robot().mb());
	mc_rtc::log::success("GravityController: init done");
}

bool GravityController::run()
{
	std::vector<std::vector<double>> target = postureTask->posture();
	std::vector<double> encoderValues = robot().encoderValues();	
	fd.computeH(robot().mb(), robot().mbc());
	fd.computeC(robot().mb(), robot().mbc());
	std::vector<std::vector<double>> alphaD = robot().mbc().alphaD, alpha = robot().mbc().alpha, q = robot().mbc().q;
	Eigen::VectorXd tau = fd.H() * rbd::dofToVector(robot().mb(), alphaD) + fd.C();

	float dist = 0;
	for(int i=0; i < mbcJointIdx.size(); i++)
		dist += std::abs(target[mbcJointIdx[i]][0] - encoderValues[refJointIdx[i]]);
	dist /= mbcJointIdx.size();
	if(dist < errorTolerance*M_PI/180.)
	{
		mc_rtc::log::info("GravityController: Difference: {:4e} {:4e}", dist, errorTolerance*M_PI/180.);
		mc_rtc::log::info("GravityController: Encoder Values: {:4f} {:4f} {:4f} {:4f} {:4f}", encoderValues[6], encoderValues[7], encoderValues[8], encoderValues[9], encoderValues[10]);
		mc_rtc::log::info("GravityController: Robot Angles: {:4f} {:4f} {:4f} {:4f} {:4f}", q[29][0], q[30][0], q[31][0], q[32][0], q[33][0]);
		mc_rtc::log::info("GravityController: Posture Target: {:4f} {:4f} {:4f} {:4f} {:4f}", target[29][0], target[30][0], target[31][0], target[32][0], target[33][0]);
		std::this_thread::sleep_for(std::chrono::milliseconds(2000));
		switch_target();			
	}
	return mc_control::MCController::run();
}

void GravityController::reset(const mc_control::ControllerResetData & reset_data)
{
	mc_control::MCController::reset(reset_data);
}

void GravityController::switch_target()
{
	bool is_colliding = true;
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<check_selfcollision::SelfCollosion>("check_selfcollision_service");
	check_selfcollision::SelfCollosionRequest request;
	check_selfcollision::SelfCollosionResponse response;
	while(is_colliding)
	{
		request.robot_state.joint_state.position.clear();
		request.robot_state.joint_state.name.clear();
		for(int i=0; i < jointNames.size(); i++)
		{
			targets[jointNames[i]] = {lower[i] + rand_multiplier[i]*((double) rand())};
			request.robot_state.joint_state.name.push_back(jointNames[i]);
			request.robot_state.joint_state.position.push_back(targets[jointNames[i]][0]);
		}
		if(client.call(request, response))
		{
			is_colliding = response.is_colliding;
			mc_rtc::log::info("GravityController: Collision Check Result: {}, {:4f} {:4f} {:4f} {:4f}", is_colliding, targets[jointNames[0]][0], targets[jointNames[1]][0], targets[jointNames[2]][0], targets[jointNames[3]][0]);
		}
		else
		{
			mc_rtc::log::error("GravityController: Collision Check Failed {} {}", client.isValid(), strcmp(ros::service_traits::md5sum(request), ros::service_traits::md5sum(response)));
		}
	}
	postureTask->target(targets);
	mc_rtc::log::info("GravityController: New Posture Target: {:4f} {:4f} {:4f} {:4f}", targets[jointNames[0]][0], targets[jointNames[1]][0], targets[jointNames[2]][0], targets[jointNames[3]][0]);
	mc_rtc::log::success("GravityController: Switch Target");
}

CONTROLLER_CONSTRUCTOR("GravityController", GravityController)
