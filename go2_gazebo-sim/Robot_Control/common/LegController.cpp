#include "LegController.h"
#include <iostream>
#include <iostream>
#include <thread>
#include <unistd.h>
#include <iomanip>

void LegControllerCommand::zero() {
	tauFeedForward = Vec3<float>::Zero();
	forceFeedForward = Vec3<float>::Zero();
	qDes = Vec3<float>::Zero();
	qdDes = Vec3<float>::Zero();
	pDes = Vec3<float>::Zero();
	vDes = Vec3<float>::Zero();
	kpCartesian = Mat3<float>::Zero();
	kdCartesian = Mat3<float>::Zero();
	kpJoint = Mat3<float>::Zero();
	kdJoint = Mat3<float>::Zero();
}


void LegControllerData::zero() {
	q = Vec3<float>::Zero();
	qd = Vec3<float>::Zero();
	p = Vec3<float>::Zero();
	v = Vec3<float>::Zero();
	J = Mat3<float>::Zero();
	tauEstimate = Vec3<float>::Zero();
  tauActuatual = Vec3<float>::Zero();
}

void LegController::zeroCommand() {
	for (auto& cmd : commands) {
		cmd.zero(); //调用zero函数，置零
	}
	_legsEnabled = false;
}

void LegController::updateData() {
  for (int leg = 0; leg < 4; leg++) {
    if(std::isnan(_lowState->motorState[3*leg    ].q))
    {
      printf("sim leg q error !!!!!!!!!!\n");
      datas[leg].q(0) =  datas[leg].q(0);
      datas[leg].q(1) =  datas[leg].q(1);
      datas[leg].q(2) =  datas[leg].q(2);

      // qd
      datas[leg].qd(0) =  datas[leg].qd(0) ;//*1.25;
      datas[leg].qd(1) =  datas[leg].qd(1) ;//*1.25;
      datas[leg].qd(2) =  datas[leg].qd(2) ;//*1.25;
    }else {
      // q:
      datas[leg].q(0) = _lowState->motorState[3*leg    ].q;
      datas[leg].q(1) = _lowState->motorState[3*leg + 1].q;
      datas[leg].q(2) = _lowState->motorState[3*leg + 2].q;

      // qd
      datas[leg].qd(0) = _lowState->motorState[3*leg    ].dq;//*1.25;
      datas[leg].qd(1) = _lowState->motorState[3*leg + 1].dq;//*1.25;
      datas[leg].qd(2) = _lowState->motorState[3*leg + 2].dq;//*1.25;
    }

    //添加的实际扭矩
    datas[leg].tauActuatual(0) = _lowState->motorState[3*leg    ].tauEst;
    datas[leg].tauActuatual(1) = _lowState->motorState[3*leg + 1].tauEst;
    datas[leg].tauActuatual(2) = _lowState->motorState[3*leg + 2].tauEst;
  }

}

void LegController::updateCommand()
{
  Vec3<float> kp, kd, q_des, qd_des;
  int mode = 0;
  for (int leg = 0; leg < 4; leg++) {
    // tauFF
    Vec3<float> legTorque = commands[leg].tauFeedForward;

    _lowCmd->setTau(leg, legTorque.template cast<double>());
    // joint space pd
    // joint space PD
    kd[0] = commands[leg].kdJoint(0, 0);
    kd[1] = commands[leg].kdJoint(1, 1);
    kd[2] = commands[leg].kdJoint(2, 2);
    _lowCmd->setKd(leg, kd.template cast<double>());

    kp[0] = commands[leg].kpJoint(0, 0);
    kp[1] = commands[leg].kpJoint(1, 1);
    kp[2] = commands[leg].kpJoint(2, 2);
    _lowCmd->setKp(leg, kp.template cast<double>());

    q_des[0] = commands[leg].qDes(0);
    q_des[1] = commands[leg].qDes(1);
    q_des[2] = commands[leg].qDes(2);
    _lowCmd->setQ(leg, q_des.template cast<double>());

    qd_des[0] = commands[leg].qdDes(0);
    qd_des[1] = commands[leg].qdDes(1);
    qd_des[2] = commands[leg].qdDes(2);
    _lowCmd->setQd(leg, qd_des.template cast<double>());

    // estimate torque
    datas[leg].tauEstimate =
            legTorque +
            commands[leg].kpJoint * (commands[leg].qDes - datas[leg].q) +
            commands[leg].kdJoint * (commands[leg].qdDes - datas[leg].qd);
  }
  mode = _legsEnabled ? 10 : 0;

  for(int i = 0; i < 12; i++)
  {
    _lowCmd->motorCmd[i].mode = mode;
  }

}

void LegController::Init()
{
  JointMaxTorque[0] << 48, 48, 48;
  JointMaxTorque[1] << 48, 48, 48;
  JointMaxTorque[2] << 48, 48, 48;
  JointMaxTorque[3] << 48, 48, 48;
}

Vec3<float> LegController::torque_clip(Vec3<float> input, Vec3<float> min, Vec3<float> max)
{
	Vec3<float> rs;
	for(int i =0; i<3; i++)
	{
		rs(i) = input(i) > max(i) ? max(i): input(i);
		rs(i) = rs(i) < min(i) ? min(i): rs(i);
	}
	return rs;
}