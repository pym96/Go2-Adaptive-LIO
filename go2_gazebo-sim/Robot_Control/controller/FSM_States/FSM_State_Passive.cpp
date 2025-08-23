#include "FSM_State_Passive.h"

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_Passive<T>::FSM_State_Passive(ControlFSMData* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::PASSIVE, "PASSIVE") {
  
  this->checkSafeOrientation = false;
  this->checkPDesFoot = false;
  this->checkForceFeedForward = false;
}

template <typename T>
void FSM_State_Passive<T>::onEnter() {
  this->nextStateName = this->stateName;
  // Reset the transition data
  this->transitionData.zero();
}

template <typename T>
void FSM_State_Passive<T>::run() {
  testTransition();
}

template <typename T>
TransitionData<T> FSM_State_Passive<T>::testTransition() {
  this->transitionData.done = true;
  return this->transitionData;
}

template <typename T>
FSM_StateName FSM_State_Passive<T>::checkTransition() {
  this->nextStateName = this->stateName;
  iter++;

  // Switch FSM control mode
  switch (this->control_mode) {
    case K_PASSIVE:  // normal c (0)
      // Normal operation for state based transitions
      break;

    case K_JOINT_PD:
      // Requested switch to joint PD control
      this->nextStateName = FSM_StateName::JOINT_PD;
      break;

    case K_STAND_UP:
      // Requested switch to joint PD control
      this->nextStateName = FSM_StateName::STAND_UP;
      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                << K_PASSIVE << " to "
                << this->control_mode << std::endl;
  }

  // Get the next state
  return this->nextStateName;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template <typename T>
TransitionData<T> FSM_State_Passive<T>::transition() {
  // Finish Transition
  this->transitionData.done = true;

  // Return the transition data to the FSM
  return this->transitionData;
}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_Passive<T>::onExit() {
  // Nothing to clean up when exiting
}

// template class FSM_State_Passive<double>;
template class FSM_State_Passive<float>;
