#ifndef PROJECT_LEGCONTROLLER_H
#define PROJECT_LEGCONTROLLER_H
#include "cTypes.h"
#include "cppTypes.h"
#include "message/LowlevelState.h"
#include "message/LowlevelCmd.h"

// template <typename T>
struct LegControllerCommand{
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LegControllerCommand() {zero();}
  void zero();
  Vec3<float> tauFeedForward,  qDes, qdDes; // 关节量
  Vec3<float> forceFeedForward, pDes, vDes; // 工作空间量
  Mat3<float> kpCartesian, kdCartesian;
  Mat3<float> kpJoint, kdJoint;
};

// template <typename T>
struct LegControllerData {
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LegControllerData() { zero(); }

	void zero();
	Vec3<float>  p, v;
  Vec3<float> q, qd;
  Mat3<float> J;
	Vec3<float> tauEstimate, tauActuatual;
};

// template <typename T>
class LegController {
public:
	LegController(LowlevelCmd* lowCmd, LowlevelState* lowState): _lowCmd(lowCmd), _lowState(lowState) {
		Init();
	}
  ~LegController(){
    delete _lowCmd;
    delete _lowState;
  }
	void Init(); 
	void zeroCommand();
	void updateData();
	void updateCommand();
	void setEnabled(bool enabled) { _legsEnabled = enabled; };
	Vec3<float> torque_clip(Vec3<float> input, Vec3<float> min, Vec3<float> max);

	LegControllerCommand commands[4];
	LegControllerData datas[4];
	bool _legsEnabled = false;
	Vec3<float> JointMaxTorque[4];

  LowlevelCmd *_lowCmd;
  LowlevelState *_lowState;

};
#endif