#ifndef CONTROLFSMDATA_H
#define CONTROLFSMDATA_H

#include "../../common/LegController.h"
#include "../../common/StateEstimatorContainer.h"
#include "../../common/QuadRobot.h"

struct ControlFSMData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  QuadRobot* _humanoid;
  StateEstimatorContainer<float>* _stateEstimator;
  LegController* _legController;

};


#endif  // CONTROLFSM_H