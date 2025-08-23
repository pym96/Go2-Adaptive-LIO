//
// Created by jh on 23-3-30.
//

#ifndef SDUOG_GUIDE_IOROS_H
#define SDUOG_GUIDE_IOROS_H

#include "ros/ros.h"
#include "interface/IOInterface.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include <sensor_msgs/Imu.h>
#include <string>

class IOROS : public IOInterface{
public:
    IOROS();
    ~IOROS();
    void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);
private:
    void sendCmd(const LowlevelCmd *cmd);
    void recvState(LowlevelState *state);
    ros::NodeHandle _nm;
    ros::Subscriber _servo_sub[12], _imu_sub;
    ros::Publisher _servo_pub[12];
    unitree_legged_msgs::LowCmd _lowCmd;
    unitree_legged_msgs::LowState _lowState;
    std::string _robot_name;

    //repeated functions for multi-thread
    void initRecv();
    void initSend();

    //Callback functions for ROS
    void imuCallback(const sensor_msgs::Imu & msg);

    void FRhipCallback(const unitree_legged_msgs::MotorState& msg);
    void FRthighCallback(const unitree_legged_msgs::MotorState& msg);
    void FRcalfCallback(const unitree_legged_msgs::MotorState& msg);

    void FLhipCallback(const unitree_legged_msgs::MotorState& msg);
    void FLthighCallback(const unitree_legged_msgs::MotorState& msg);
    void FLcalfCallback(const unitree_legged_msgs::MotorState& msg);

    void RRhipCallback(const unitree_legged_msgs::MotorState& msg);
    void RRthighCallback(const unitree_legged_msgs::MotorState& msg);
    void RRcalfCallback(const unitree_legged_msgs::MotorState& msg);

    void RLhipCallback(const unitree_legged_msgs::MotorState& msg);
    void RLthighCallback(const unitree_legged_msgs::MotorState& msg);
    void RLcalfCallback(const unitree_legged_msgs::MotorState& msg);
};


#endif //SDUOG_GUIDE_IOROS_H
