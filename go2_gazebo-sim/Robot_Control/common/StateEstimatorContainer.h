#ifndef PROJECT_STATEESTIMATOR_H
#define PROJECT_STATEESTIMATOR_H

#include "LegController.h"

#include "message/LowlevelState.h"
template <typename T>
struct StateEstimate {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Vec4<T> contactEstimate; 	//touch ground or not contact
	Vec3<T> position;             // torso posiotn
	Vec3<T> vBody;                // torso velocity
	Quat<T> orientation;          // torso orientation  (equal to)  rpy  //wxyz in turns
	Vec3<T> omegaBody;            // angular velocity in body frame
	RotMat<T> rBody;              // torso rotation matrix
	Vec3<T> rpy;                  //rpy

	Vec3<T> omegaWorld;          //angular velocity in world frame
	Vec3<T> vWorld;              //
	Vec3<T> aBody, aWorld;
	// Vec3<T> desired_command;
};

struct IMUData {
 Vec3<float> accelerometer;
 Vec3<float> gyro;
 Quat<float> quat;
 // todo is there status for the vectornav?
};

template <typename T>
struct StateEstimatorData {
	StateEstimate<T>* result;  // where to write the output to
	IMUData* imuData;
	LegControllerData* legControllerData;
	Vec2<T>* contactPhase;
  LowlevelState *lowState;
};

/*!
* All Estimators should inherit from this class
* 所有估计器的基类，通用估计器
*/
template <typename T>
class GenericEstimator {
public:
	virtual void run() = 0;
	virtual void setup() = 0;

	void setData(StateEstimatorData<T> data) { _stateEstimatorData = data; }
	virtual ~GenericEstimator() = default;
	StateEstimatorData<T> _stateEstimatorData;
};

template <typename T>
class StateEstimatorContainer {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW


  StateEstimatorContainer(IMUData* imuData,
                          LegControllerData* legControllerData,
                          StateEstimate<T>* stateEstimate,
                          LowlevelState *lowState)
	{
	  _data.imuData = imuData;
	  _data.legControllerData = legControllerData;
	  _data.result = stateEstimate;
	  _phase = Vec2<T>::Zero();
	  _data.contactPhase = &_phase;
    _data.lowState = lowState;
	}

	/*!
	* Run all estimators
	* 运行所有的状态估计器
	*/
	void run()
	{
	    for (auto estimator : _estimators)
	    {
		    estimator->run();
	    }
	}

	/*!
	* Get the result
	*/
	const StateEstimate<T>& getResult() { return *_data.result; }

	/*!
	* Set the contact phase
	*/
	void setContactPhase(Vec2<T>& phase) { *_data.contactPhase = phase; }

	/*!
	* Add an estimator of the given type
	* @tparam EstimatorToAdd
	*/
	template <typename EstimatorToAdd>
	void addEstimator() {
		auto* estimator = new EstimatorToAdd();
		estimator->setData(_data);
		estimator->setup(); //这个是初始化
		_estimators.push_back(estimator);
	}
	/*!
	* Remove all estimators
	* 删除所有的状态估计器
	*/
	void removeAllEstimators() {
		for (auto estimator : _estimators) {
			delete estimator;
		}
		_estimators.clear();
	}

	~StateEstimatorContainer() {
		for (auto estimator : _estimators) {
			delete estimator;
		}
	}

private:
	StateEstimatorData<T> _data;
	std::vector<GenericEstimator<T>*> _estimators;
	Vec2<T> _phase;
};

#endif