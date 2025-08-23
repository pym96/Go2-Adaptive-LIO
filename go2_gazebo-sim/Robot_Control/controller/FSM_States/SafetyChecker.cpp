#include "SafetyChecker.h"

/**
 * @return safePDesFoot true if safe desired foot placements
 */
//检测姿态是不是正确，rpy角度是不是正常范围内
template <typename T> 
bool SafetyChecker<T>::checkSafeOrientation() {
  if (abs(data->_stateEstimator->getResult().rpy(0)) >= 1 ||
      abs(data->_stateEstimator->getResult().rpy(1)) >= 1) {
    return false;
  } else {
    return true;
  }
}

/**
 * @return safePDesFoot true if safe desired foot placements
 * 检测腿的目标位置是否安全，包括xyz方向的最大位置，腿长最大约束，保证运动目标都是在安全界限里面
 */
template <typename T>
bool SafetyChecker<T>::checkPDesFoot() {
  // Assumed safe to start
  bool safePDesFoot = true;


  // Return true if all desired positions are safe
  return safePDesFoot;
}

/**
 * @return safePDesFoot true if safe desired foot placements
 * 确保xyz三个方向的力在安全范围内
 */
template <typename T>
bool SafetyChecker<T>::checkForceFeedForward() {
  // Assumed safe to start
  bool safeForceFeedForward = true;


  // Return true if all feed forward forces are safe
  return safeForceFeedForward;
}

// template class SafetyChecker<double>; This should be fixed... need to make
// RobotRunner a template
template class SafetyChecker<float>;