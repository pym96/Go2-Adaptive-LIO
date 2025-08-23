#ifndef PRO_QuadRobotMODEL_H
#define PRO_QuadRobotMODEL_H

#include "QuadRobot.h"

QuadRobot buildQuadRobotModel() {
  QuadRobot x_QuadRobot;
  
  x_QuadRobot._bodyMass = 3.3;
  x_QuadRobot._bodyLength =0.2*2;// 0.19 * 2;//躯干中点到侧摆电机连杆的距离
  x_QuadRobot._bodyWidth = 0.055*2;//0.049 * 2;//侧摆电机之间的距离
  x_QuadRobot._bodyHeight = 0.057*2;//0.05 * 2; //上下躯干距离
  x_QuadRobot._abadGearRatio = 6; 
  x_QuadRobot._hipGearRatio = 6;
  x_QuadRobot._kneeGearRatio = 9.47;//9.33;  //同步带传送比是1：1.555
  x_QuadRobot._abadLinkLength =0.0738;//0.085;// 0.062;//侧摆电机轴线到小腿平面的距离 //0.085
  x_QuadRobot._hipLinkLength = 0.222;//0.211;////0.209; //大腿长度 0.211
  x_QuadRobot._kneeLinkLength = 0.181;//0.203;//0.175; //小腿长度 0.185 //0.203
  x_QuadRobot._maxLegLength = 0.403;//0.414;//0.384;   //两者只和

  x_QuadRobot._motorTauMax = 3.f; //电机最大扭矩3Nm，对应膝关节28Nm，侧摆和大腿18Nm
  x_QuadRobot._batteryV = 24;
  x_QuadRobot._motorKT = .05;  // this is flux linkage * pole pairs
  x_QuadRobot._motorR = 0.173;
  x_QuadRobot._jointDamping = .01;
  x_QuadRobot._jointDryFriction = .2;

  // locations 基于躯干的位置向量
  x_QuadRobot._abadRotorLocation = Vec3<float>(0.125, 0.049, 0);//躯干中心到侧摆电机的位置

  x_QuadRobot._abadLocation = Vec3<float>(x_QuadRobot._bodyLength, x_QuadRobot._bodyWidth, 0) * 0.5;
  x_QuadRobot._hipLocation = Vec3<float>(0, x_QuadRobot._abadLinkLength, 0);
  x_QuadRobot._hipRotorLocation = Vec3<float>(0, 0.04, 0);
  x_QuadRobot._kneeLocation = Vec3<float>(0, 0, -x_QuadRobot._hipLinkLength);
  x_QuadRobot._kneeRotorLocation = Vec3<float>(0, 0, 0);

  return x_QuadRobot;
}
#endif