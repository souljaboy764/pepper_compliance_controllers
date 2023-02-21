#include <ros/ros.h>
#include <check_selfcollision/SelfCollosion.h>
#include <check_selfcollision/SelfCollosionRequest.h>
#include <check_selfcollision/SelfCollosionResponse.h>

#include "BolotnikovaController.h"

#include <cstdlib>
#include <cmath>
#include <chrono>
#include <thread>
#include <fstream>

BolotnikovaController::BolotnikovaController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt)
{
	// Load state config
	config_.load(config);

	if(!config("ControllerParams").has("inContactDuration"))
		mc_rtc::log::error_and_throw<std::runtime_error>("MakeContactBack start | inContactDuration config entry missing");
	config("ControllerParams")("inContactDuration", inContactDuration_);

	// Read state configuration
	if(!config("ControllerParams").has("features"))
		mc_rtc::log::error_and_throw<std::runtime_error>("BolotnikovaController start | features config entry missing");
	config("ControllerParams")("features", features_);
	
	if(!config("ControllerParams").has("residualThreshold"))
		mc_rtc::log::error_and_throw<std::runtime_error>("BolotnikovaController start | residualThreshold config entry missing");
	config("ControllerParams")("residualThreshold", residualThreshold_);
	
	if(!config_("ControllerParams").has("monitoredJoint"))
		mc_rtc::log::error_and_throw<std::runtime_error>("BolotnikovaController start | monitoredJoint config entry missing");
	config_("ControllerParams")("monitoredJoint", monitoredJointName_);
	monitoredJointIndex_ = robot().jointIndexByName(monitoredJointName_);
	auto it = std::find(robot().refJointOrder().begin(), robot().refJointOrder().end(), monitoredJointName_);
	monitoredJointRefOrder_ = std::distance(robot().refJointOrder().begin(), it);

	// Path to model file
	if(!config_("ControllerParams").has("pathToModel"))
		mc_rtc::log::error_and_throw<std::runtime_error>("BolotnikovaController start | pathToModel config entry missing");
	std::string pathToModel;
	config_("ControllerParams")("pathToModel", pathToModel);
	// Load trained predictor model
	XGBoosterCreate(0, 0, &boosterHandle_);
	if(XGBoosterLoadModel(boosterHandle_, pathToModel.c_str()) == -1)
		mc_rtc::log::error_and_throw<std::runtime_error>("BolotnikovaController start | XGBoost model cannot be loaded from {}, {}", pathToModel,
																				XGBoosterLoadModel(boosterHandle_, pathToModel.c_str()));
  
	// Validate model feature vector size
	XGBoosterGetNumFeature(boosterHandle_, &numF_);
	if(numF_ != features_.size())// FIX removing this check causes prediction failure
		mc_rtc::log::error_and_throw<std::runtime_error>("BolotnikovaController start | Feature vector size mismatch. Expect {}, got {}", features_.size(), numF_);
	
	// Load posture goal
	if(!config_("ControllerParams").has("armPostureGoal"))
		mc_rtc::log::error_and_throw<std::runtime_error>("BolotnikovaController start | armPostureGoal config entry missing");
	config_("ControllerParams")("armPostureGoal", armPostureGoal_);
	
	solver().addConstraintSet(selfCollisionConstraint);
	// postureTask->reset();
	postureTask->target(armPostureGoal_);
	solver().addTask(postureTask);
	mc_rtc::log::success("BolotnikovaController: init done");
}

void BolotnikovaController::updateInputVector(std::vector<std::pair<std::string, std::string>> &features)
{
	float featureArray[1][numF_];
	for (size_t i = 0; i < features.size(); i++)
	{
		unsigned int jointIndex = robot().jointIndexByName(features[i].first);
		if(features[i].second == "alpha")
			featureArray[0][i] = float(robot().mbc().alpha[jointIndex][0]);
		else if(features[i].second == "alphaD")
			featureArray[0][i] = float(robot().mbc().alphaD[jointIndex][0]);
		else if(features[i].second == "torque")
			featureArray[0][i] = float(robot().mbc().jointTorque[jointIndex][0]);
		else
			mc_rtc::log::error_and_throw<std::runtime_error>("BolotnikovaController updateInputVector | cannot add feature {} of joint {}", 
				features[i].second, features[i].first
			);
	}
	// Update input feature matrix handle
	if(XGDMatrixCreateFromMat((float *)featureArray, 1, numF_, -1, &inputVec_) == -1)
		mc_rtc::log::error_and_throw<std::runtime_error>("BolotnikovaController updateInputVector | XGBoost cannot update input feature matrix handle");
}

bool BolotnikovaController::run()
{
	if(first) // For some reason, despite setting the target in the constructor, it gets reset by the time this gets called
	{
		first = false;
		mc_rtc::log::success("BolotnikovaController: Resetting Target Pose");
		postureTask->target(armPostureGoal_);
	}
	// Update feature vector
	updateInputVector(features_);

	// Predict expected position tracking error
	if(XGBoosterPredict(boosterHandle_, inputVec_, 0, 0, 0, &outLen_, &outVec_) == -1)
		mc_rtc::log::error_and_throw<std::runtime_error>("BolotnikovaController run | XGBoosterPredict failure");

	if(outLen_!=1)
		mc_rtc::log::error_and_throw<std::runtime_error>("BolotnikovaController run | Wrong prediction result size {}", outLen_);

	// Compute discrepancy
	err_ = robot().q()[monitoredJointIndex_][0] - robot().encoderValues()[monitoredJointRefOrder_];
	errExp_ = outVec_[0];
	jointResidual_ = err_ - errExp_;

	// Add residual signal to the filter window
	medianFilter_.addSample(jointResidual_);
	// Wait for the filter window to be filled
	if(stateTime_/solver().dt() > filterWindowSize_)
		filterWindowFilled_ = true;
	
	if(filterWindowFilled_)
	{
		// Compute residual signal filter window median
		jointResidualFiltered_ = medianFilter_.getMedian();
		// Check if contact is detected
		if(std::abs(jointResidualFiltered_) > residualThreshold_)
			contactDetected_ = true;
	}
	mc_rtc::log::success("BolotnikovaController: {:3e}\t{:3e}\t{:3e}\t{:3e}", err_, errExp_, jointResidual_, jointResidualFiltered_);
	if(contactDetected_)
	{
		/* if(!postureTaskReset_)
		{
			postureTask->reset();
			postureTaskReset_ = true;
		} */
		// In contact period countdown
		inContactDuration_ -= solver().dt();
		// State termination criteria
		if(inContactDuration_ <= 0)
		{
			mc_rtc::log::success("BolotnikovaController: Contact Period Done, changing goal");
		}
	}
	
	stateTime_ = stateTime_ + solver().dt();
	return mc_control::MCController::run();
}

void BolotnikovaController::reset(const mc_control::ControllerResetData & reset_data)
{
	mc_control::MCController::reset(reset_data);
}

CONTROLLER_CONSTRUCTOR("BolotnikovaController", BolotnikovaController)
