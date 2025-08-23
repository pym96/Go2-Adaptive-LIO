#include "OrientationEstimator.h"
#include "orientation_tools.h"

Vec3<float> desired_command;

//真正运行机器人时候的orientation估计
template <typename T>
void IMUOrientationEstimator<T>::run() {
  this->_stateEstimatorData.result->orientation[0] =
          this->_stateEstimatorData.lowState->imu.quaternion[0];
  this->_stateEstimatorData.result->orientation[1] =
          this->_stateEstimatorData.lowState->imu.quaternion[1];
  this->_stateEstimatorData.result->orientation[2] =
          this->_stateEstimatorData.lowState->imu.quaternion[2];
  this->_stateEstimatorData.result->orientation[3] =
          this->_stateEstimatorData.lowState->imu.quaternion[3];
  if(_b_first_visit){
    Vec3<T> rpy_ini = ori::quatToRPY(this->_stateEstimatorData.result->orientation);
    rpy_ini[0] = 0;
    rpy_ini[1] = 0;
    _ori_ini_inv = ori::rpyToQuat(-rpy_ini);
    _b_first_visit = false;
  }
  this->_stateEstimatorData.result->orientation =
          ori::quatProduct(_ori_ini_inv, this->_stateEstimatorData.result->orientation);

  this->_stateEstimatorData.result->rpy =
          ori::quatToRPY(this->_stateEstimatorData.result->orientation);

  this->_stateEstimatorData.result->rBody = ori::quaternionToRotationMatrix(
          this->_stateEstimatorData.result->orientation);

  for(int i = 0; i < 3; i++)
  {
    this->_stateEstimatorData.result->omegaBody[i] =
            this->_stateEstimatorData.lowState->imu.gyroscope[i];
  }


  this->_stateEstimatorData.result->omegaWorld =
          this->_stateEstimatorData.result->rBody.transpose() *
          this->_stateEstimatorData.result->omegaBody;

  for(int i = 0; i < 3; i++)
  {
    this->_stateEstimatorData.result->aBody[i] =
            this->_stateEstimatorData.lowState->imu.accelerometer[i]; // 0 0 9.8
  }
  this->_stateEstimatorData.result->aWorld =
          this->_stateEstimatorData.result->rBody.transpose() *
          this->_stateEstimatorData.result->aBody;

}
template <typename T>
void IMUOrientationEstimator<T>:: getUserDesiredCommands()
{
    
}


template class IMUOrientationEstimator<float>;
template class IMUOrientationEstimator<double>;
