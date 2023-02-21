#pragma once

#include <mc_control/mc_controller.h>
#include <vector>
#include <string>


#include <rbdl/Dynamics.h>

#include "api.h"

struct GravityController_DLLAPI GravityController : public mc_control::MCController
{
	GravityController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

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

	RigidBodyDynamics::Model rbdl_model_;  /*!< Robot model from RBDL */

	Eigen::VectorXd q_zero_;    /*!< Zero vector with joint_names size */
	Eigen::VectorXd tau_cmd_;   /*!< Vector with the necessary torque to maintain gravity */
	Eigen::VectorXd q_act_;     /*!< Vector with the current position of the joint states */


};