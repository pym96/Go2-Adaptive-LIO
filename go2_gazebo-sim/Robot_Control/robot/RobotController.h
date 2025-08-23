#ifndef ROBOT_CONTROLLER_H_
#define ROBOT_CONTROLLER_H_
#include "../common/LegController.h"
#include "../common/StateEstimatorContainer.h"
#include "../common/QuadRobot.h"


class RobotController {
	//friend class RobotRunner;
public:
	RobotController() {}
//	virtual ~RobotController() {}

	 void initializeController() ;
	 void runController() ;
	
// protected:
	QuadRobot* _humanoid;
	LegController* _legController;
	StateEstimatorContainer<float>* _stateEstimator;
	StateEstimate<float>* _stateEstimate;
};
#endif