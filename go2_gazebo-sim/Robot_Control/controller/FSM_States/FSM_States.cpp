/*============================= FSM State =============================*/
/**
 * FSM State base class
 */

#include "FSM_States.h"
/**
 * Constructor for the FSM State class.
 *	构建FSM类
 * @param _controlFSMData holds all of the relevant control data
 * @param stateNameIn the enumerated state name 
 * @param stateStringIn the string name of the current FSM state
 */
template <typename T>
FSM_State<T>::FSM_State(ControlFSMData* _controlFSMData,
                        FSM_StateName stateNameIn, std::string stateStringIn)
    : _data(_controlFSMData),
      stateName(stateNameIn),
      stateString(stateStringIn) {
  transitionData.zero();
  std::cout << "[FSM_State] Initialized FSM state: " << stateStringIn  << std::endl;
}

template <typename T>
void FSM_State<T>::jointPDControl(int leg, Vec3<T> qDes, Vec3<T> qdDes) {
    kpMat << 120, 0, 0, 0, 120, 0, 0, 0, 120;
    kdMat << 2, 0, 0, 0, 2, 0, 0, 0, 2;

    // kpMat << 300, 0, 0, 0, 300, 0, 0, 0, 300;
    // kdMat << 5, 0, 0, 0, 5, 0, 0, 0, 5;
    _data->_legController->commands[leg].qDes = qDes;
    _data->_legController->commands[leg].qdDes = qdDes;
    _data->_legController->commands[leg].kpJoint = kpMat;
    _data->_legController->commands[leg].kdJoint = kdMat;
}

/**
 * Gait independent formulation for choosing appropriate GRF and step locations
 * as well as converting them to leg controller understandable values.
 * 运行不同类型的控制器算法
 */
template <typename T>
void FSM_State<T>::runControls() {
  // This option should be set from the user interface or autonomously
  // eventually
  int CONTROLLER_OPTION = 1;

  // Reset the forces and steps to 0
  footFeedForwardForces = Mat34<T>::Zero();
  footstepLocations = Mat34<T>::Zero();

  // Choose the controller to run for picking step locations and balance forces
  if (CONTROLLER_OPTION == 0) { //模式0是半个身高站立
   
  } 
  else if (CONTROLLER_OPTION == 1) { //模式1是QP平衡控制
   
  } else if (CONTROLLER_OPTION == 2) { //模式2是WBC
    
  } else if (CONTROLLER_OPTION == 3) { //模式3是MPC
   

  } else if (CONTROLLER_OPTION == 4) { //模式4 通用预测控制RPC
    

  } else {
    // Zero out the commands if a controller was not selected
    // Print an error message
    std::cout << "[FSM_State] ERROR: No known controller was selected: "
              << CONTROLLER_OPTION << std::endl;
  }
}

/**
 * Gait independent formulation for choosing appropriate GRF and step locations
 * as well as converting them to leg controller understandable values.
 */
template <typename T>
void FSM_State<T>::turnOnAllSafetyChecks() {
  
  checkSafeOrientation = true;  // check roll and pitch
  // Post control safety checks
  checkPDesFoot = true;          // do not command footsetps too far
  checkForceFeedForward = true;  // do not command huge forces
  checkLegSingularity = true;    // do not let leg
}

/**
 *
 */
template <typename T>
void FSM_State<T>::turnOffAllSafetyChecks() {
  
  checkSafeOrientation = false;  // check roll and pitch
  // Post control safety checks
  checkPDesFoot = false;          // do not command footsetps too far
  checkForceFeedForward = false;  // do not command huge forces
  checkLegSingularity = false;    // do not let leg
}

// template class FSM_State<double>;
template class FSM_State<float>;
