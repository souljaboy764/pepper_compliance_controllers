#pragma once

#include <mc_control/mc_controller.h>
#include <vector>
#include <string>

#include "api.h"
#include "MedianFilter.h"

#include <xgboost/c_api.h>

struct BolotnikovaController_DLLAPI BolotnikovaController : public mc_control::MCController
{
	BolotnikovaController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

	bool run() override;

	void reset(const mc_control::ControllerResetData & reset_data) override;

private:
	rbd::ForwardDynamics fd;
	bool first = true;

	// Taken from https://github.com/anastasiabolotnikova/autonomous_phri_init/blob/master/src/states/MakeContactBack.h
	void updateInputVector(std::vector<std::pair<std::string, std::string>> &features);

	// State configuration
	mc_rtc::Configuration config_;
	// State time
	double stateTime_ = 0.0;

	// Posture target
	std::map<std::string, std::vector<double>> armPostureGoal_;

	// Monitor if contact is detected
	bool contactDetected_ = false;
	// How long to stay in contact after it has been detected
	double inContactDuration_;
	// Reset posture task target after contact is detected
	bool postureTaskReset_ = false;

	// XGBoost predictor
	BoosterHandle boosterHandle_;
	// Monitored joint info
	std::string monitoredJointName_;
	unsigned int monitoredJointIndex_;
	unsigned int monitoredJointRefOrder_;
	// Names of the model input features
	std::vector<std::pair<std::string, std::string>> features_;
	// Number of features
	bst_ulong numF_;
	// Input feature vector handle
	DMatrixHandle inputVec_;
	// Prediction vector length
	bst_ulong outLen_;
	// Prediction output vector
	const float *outVec_;
	// Expected position tracking error
	double errExp_ = 0.0;

	// Measured position tracking error
	double err_ = 0.0;
	// Residual value
	double jointResidual_ = 0.0;
	// Median filter
	const static int filterWindowSize_ = 20;
	MedianFilter<double, filterWindowSize_> medianFilter_;
	bool filterWindowFilled_ = false;
	// Filtered contact residual
	double jointResidualFiltered_ = 0.0;
	// Residual threshold
	double residualThreshold_;
};