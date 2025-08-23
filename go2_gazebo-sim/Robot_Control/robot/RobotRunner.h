#ifndef PRO_ROBOTRUNNER_H_
#define PRO_ROBOTRUNNER_H_

#include <eigen3/Eigen/Core>
#include "../controller/CT_Controller.h"
#include "../common/QuadRobot.h"
#include "../common/OrientationEstimator.h"

#include "message/LowlevelState.h"
#include "message/LowlevelCmd.h"
#include "interface/IOInterface.h"

class QuadRobotRunner 
{
public:
	// EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	QuadRobotRunner(IOInterface *ioInter):ioInter(ioInter){
		init();
	};
  virtual ~QuadRobotRunner();

	void init();// override;
	void run();// override;

	void slowToInitPosition();
	void initializeStateEstimator();

	CT_Controller* ct_ctrl;

	float _ini_yaw;
	int iter = 0;
	QuadRobot _QuadRobot;
	LegController* _legController;// = nullptr;
	StateEstimate<float>* _stateEstimate;
	StateEstimatorContainer<float>* _stateEstimator;
	IMUData *imuData;

	u64 _iterations;
	float desired_init_time; //初始化时间
	float slowToInitPosition_time=0.; //初始进行的时间
	Vec5<float> leg_joint_init[2],leg_joint_target[2];

  LowlevelCmd *lowCmd;
  LowlevelState *lowState;
  IOInterface *ioInter;
};
#endif  // PROJECT_ROBOTRUNN