#include "ControlFSM.h"

//总的FSM程序，这部分是通过ControlFSMData管理的
template <typename T>

ControlFSM<T>::ControlFSM(QuadRobot* _humanoid,
                          StateEstimatorContainer<T>* _stateEstimator,
                          LegController* _legController)
			  {
  std::cout << "-------------------------------------------" << std::endl;
  std::cout << std::endl;
  //控制相关的数据
  data._humanoid = _humanoid;
  data._stateEstimator = _stateEstimator;
  data._legController = _legController;

  // Initialize and add all of the FSM States to the state list
  statesList.invalid = nullptr;
  statesList.passive = new FSM_State_Passive<T>(&data);
  statesList.rl = new FSM_State_RL<T>(&data);
  // safetyChecker = new SafetyChecker<T>(&data);
  // Initialize the FSM with the Passive FSM State
  initialize();
}

template <typename T>
void ControlFSM<T>::initialize() {
  // Initialize a new FSM State with the control data
  currentState = statesList.rl;

  // Enter the new current state cleanly
  currentState->onEnter();

  // Initialize to not be in transition
  nextState = currentState;

  // Initialize FSM mode to normal operation
  operatingMode = FSM_OperatingMode::NORMAL;
}


template <typename T>
void ControlFSM<T>::runFSM() {
  // Check the robot state for safe operation
  operatingMode = safetyPreCheck();
  
  currentState = statesList.rl;

  currentState->run();
}

/**
 */
template <typename T>
FSM_OperatingMode ControlFSM<T>::safetyPreCheck() {
  return operatingMode;
}

/**
 */
template <typename T>
FSM_OperatingMode ControlFSM<T>::safetyPostCheck() {
  // Default is to return the current operating mode
  return operatingMode;
}


template <typename T>
FSM_State<T>* ControlFSM<T>::getNextState(FSM_StateName stateName) {
  // Choose the correct FSM State by enumerated state name
  switch (stateName) {
    case FSM_StateName::INVALID:
      return statesList.invalid;

    case FSM_StateName::RL:
      return statesList.rl;

    default:
      return statesList.invalid;
  }
}

template class ControlFSM<float>;