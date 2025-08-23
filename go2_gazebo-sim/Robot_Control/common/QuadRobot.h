#ifndef PROJECT_HUMANOID_H
#define PROJECT_HUMANOID_H

#include <eigen3/Eigen/StdVector>
#include "SpatialInertial.h"

class QuadRobot {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  float _bodyLength, _bodyWidth, _bodyHeight, _bodyMass;
  float _abadLinkLength, _hipLinkLength, _kneeLinkLength, _maxLegLength;
  float _motorKT, _motorR, _batteryV;
  float _motorTauMax;
  float _jointDamping, _jointDryFriction;
  SpatialInertia<float> _abadInertia, _hipInertia, _kneeInertia, _abadRotorInertia, _hipRotorInertia, _kneeRotorInertia, _bodyInertia;
  //基于躯干质心的位置向量
  Vec3<float> _abadLocation, _abadRotorLocation, _hipLocation, _hipRotorLocation,
      _kneeLocation, _kneeRotorLocation; 
  int _abadGearRatio, _hipGearRatio,_kneeGearRatio;


};



#endif