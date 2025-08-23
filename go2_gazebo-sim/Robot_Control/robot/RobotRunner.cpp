#include "RobotRunner.h"
#include <unistd.h>
#include <iostream>
#include "../common/QuadRobotDynamics.h"

//void RobotRunner::RobotRunner(){ }

void QuadRobotRunner::init() {

  lowCmd = new LowlevelCmd();
  lowState = new LowlevelState();

  _QuadRobot = buildQuadRobotModel();
  _legController = new LegController(lowCmd, lowState);
  _stateEstimate = new StateEstimate<float>();
  _stateEstimator = new StateEstimatorContainer<float>( imuData, _legController->datas, _stateEstimate, lowState);
  initializeStateEstimator();
  ct_ctrl = new CT_Controller();

  ct_ctrl->_humanoid = &_QuadRobot;
  ct_ctrl->_legController = _legController;
  ct_ctrl->_stateEstimator = _stateEstimator;
  ct_ctrl->_stateEstimate = _stateEstimate;
  ct_ctrl->initializeController();

}

void QuadRobotRunner::run() {
  _stateEstimator->run();

  _legController->updateData();
  _legController->zeroCommand();
  _legController->setEnabled(true);

  ct_ctrl->runController();

  _legController->updateCommand();

  ioInter->sendRecv(lowCmd, lowState);

}

void QuadRobotRunner::initializeStateEstimator() {

	_stateEstimator->removeAllEstimators();
	Vec2<float> contactDefault;
	contactDefault << 0.5, 0.5;

	_stateEstimator->setContactPhase(contactDefault);
	_stateEstimator->addEstimator<IMUOrientationEstimator<float>>();
}

QuadRobotRunner::~QuadRobotRunner() {
	delete _legController;
	delete ct_ctrl;
	delete _stateEstimator;
	delete imuData;
	delete lowCmd;
	delete lowState;
	delete ioInter;

}
//
//void RobotRunner::cleanup() {}