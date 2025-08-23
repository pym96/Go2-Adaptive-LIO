
#ifndef PROJECT_ORIENTATIONESTIMATOR_H
#define PROJECT_ORIENTATIONESTIMATOR_H

#include "StateEstimatorContainer.h"
#include "orientation_tools.h"

template <typename T>
class IMUOrientationEstimator : public GenericEstimator<T> {
public:
	IMUOrientationEstimator(){};
	
	virtual void run();
	virtual void setup() {}
	void getUserDesiredCommands();
  bool _b_first_visit = true;
  Quat<T> _ori_ini_inv;
};


#endif  // PROJECT_ORIENTATIONESTIMATOR_H
