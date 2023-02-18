#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/RobotState.h>

#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>

#include "RandomGoalController.h"

#include <cstdlib>
#include <cmath>
#include <chrono>
#include <thread>
#include <fstream>

RandomGoalController::RandomGoalController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
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
			mc_rtc::log::error_and_throw<std::runtime_error>("RandomGoalController: wrong joint nameL {}", name);
		refJointIdx.push_back(refIdx - refJointNames.begin());
		rand_multiplier.push_back((upper[i] - lower[i])/RAND_MAX);
		mc_rtc::log::info("RandomGoalController Joint[{}] name: \"{}\", mbc index: {}, ref index {}", i, name, idx, refIdx - refJointNames.begin());
	}
	solver().addConstraintSet(selfCollisionConstraint);
	switch_target();
	solver().addTask(postureTask);
	
	std::string urdf_path = config_("ControllerParams")("URDFPath"), srdf_path = config_("ControllerParams")("SRDFPath");
	/* std::ifstream urdf(urdf_path), srdf(srdf_path);
	std::stringstream urdf_buffer, srdf_buffer;
	urdf_buffer << urdf.rdbuf();
	srdf_buffer << srdf.rdbuf(); */

	/* urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDFFile(urdf_path);
	srdf::Model srdf;//(new srdf::Model);// = std::make_shared<srdf::Model>();
	srdf.initFile(*urdf, srdf_path);
	const srdf::ModelConstSharedPtr& srdf_ptr = std::make_shared<srdf::Model>(srdf);
	moveit::core::RobotModel robot_model(urdf, srdf_ptr);
	const moveit::core::RobotModelPtr& kinematic_model = std::make_shared<moveit::core::RobotModel>(urdf, srdf_ptr); */
	/* robot_model_loader::RobotModelLoader::Options opt(urdf_buffer.str(), srdf_buffer.str());
	robot_model_loader::RobotModelLoader model_loader(opt);
	const moveit::core::RobotModelPtr& kinematic_model = model_loader.getModel(); */
	// scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);

	fd = rbd::ForwardDynamics(robot().mb());
	mc_rtc::log::success("RandomGoalController init done");
}

bool RandomGoalController::run()
{
	float dist = 0;
	std::vector<std::vector<double>> target = postureTask->posture();
	std::vector<double> encoderValues = robot().encoderValues();
	
	for(int i=0; i < mbcJointIdx.size(); i++)
		dist += std::abs(target[mbcJointIdx[i]][0] - encoderValues[refJointIdx[i]]);
	dist /= mbcJointIdx.size();
	
	fd.computeH(robot().mb(), robot().mbc());
	fd.computeC(robot().mb(), robot().mbc());
	std::vector<std::vector<double>> alphaD = robot().mbc().alphaD, alpha = robot().mbc().alpha, q = robot().mbc().q;
	Eigen::VectorXd tau = fd.H() * rbd::dofToVector(robot().mb(), alphaD) + fd.C();
	// int pidx = 0;
	// if(send(0).robots_state.size()>0)
	// 	mc_rtc::log::info("RandomGoalController:\tTau {:.4e}\tAlphaD {:.4f}\tAlpha {:.4f}\tQ_d {:.4f}\tQ {:.4f}\t\u03F5 {:.4f}\tdist {:4f}", tau[mbcJointIdx[pidx]], alphaD[mbcJointIdx[pidx]][0], alpha[mbcJointIdx[pidx]][0], q[mbcJointIdx[pidx]][0], robot().encoderValues()[refJointIdx[pidx]], robot().encoderValues()[refJointIdx[pidx]] - q[mbcJointIdx[pidx]][0], dist);
	if(dist < errorTolerance*M_PI/180.)
	{
		mc_rtc::log::info("RandomGoalController Difference: {:4e} {:4e}", dist, errorTolerance*M_PI/180.);
		mc_rtc::log::info("RandomGoalController Encoder Values: {:4f} {:4f} {:4f} {:4f} {:4f}", encoderValues[6], encoderValues[7], encoderValues[8], encoderValues[9], encoderValues[10]);
		mc_rtc::log::info("RandomGoalController Robot Angles: {:4f} {:4f} {:4f} {:4f} {:4f}", q[29][0], q[30][0], q[31][0], q[32][0], q[33][0]);
		mc_rtc::log::info("RandomGoalController Posture Target: {:4f} {:4f} {:4f} {:4f} {:4f}", target[29][0], target[30][0], target[31][0], target[32][0], target[33][0]);
		std::this_thread::sleep_for(std::chrono::milliseconds(2000));
		switch_target();
					
	}
	return mc_control::MCController::run();
}

void RandomGoalController::reset(const mc_control::ControllerResetData & reset_data)
{
	mc_control::MCController::reset(reset_data);
}

void RandomGoalController::switch_target()
{
	collision_detection::CollisionRequest collision_request;
	collision_detection::CollisionResult collision_result;
	for(int i=0; i < jointNames.size(); i++)
		targets[jointNames[i]] = {lower[i] + rand_multiplier[i]*((double) rand())};
	postureTask->target(targets);
	mc_rtc::log::info("RandomGoalController New Posture Target: {:4f} {:4f} {:4f} {:4f}", targets[jointNames[0]][0], targets[jointNames[1]][0], targets[jointNames[2]][0], targets[jointNames[3]][0]);
	mc_rtc::log::success("RandomGoalController Switch Target");
	/* if(handsOpen)
	{
		for(int i=0; i < jointNames.size(); i++)
			targets[jointNames[i]] = robot().qu()[mbcJointIdx[i]];
		postureTask->target(targets);
	}
	else
	{
		for(int i=0; i < jointNames.size(); i++)
			targets[jointNames[i]] = robot().ql()[mbcJointIdx[i]];
		postureTask->target(targets);
	}
	handsOpen = !handsOpen; */
}


CONTROLLER_CONSTRUCTOR("RandomGoalController", RandomGoalController)
