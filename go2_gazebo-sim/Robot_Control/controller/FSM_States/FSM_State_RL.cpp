#include "FSM_State_RL.h"
//#include <Configuration.h>
#include <iostream>
#include <thread>
#include <unistd.h>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <vector>


// 对观测值进行标准化
Eigen::VectorXf standardizeObservations( Eigen::VectorXf observations,  Eigen::VectorXf mean,  Eigen::VectorXf std) {
     return (observations - mean).array() / std.array();
}

template <typename T>
FSM_State_RL<T>::FSM_State_RL(ControlFSMData* _controlFSMData)
        : FSM_State<T>(_controlFSMData, FSM_StateName::RL, "RL_JOINT_PD"),
          _ini_jpos(12), policy({ 512,256,128}),adaptation({256, 128,64}), historyWrapper(OBSDIM, NUM_HISTORY){


    _obsDim = OBSDIM;
    nJoints_ = 12;
    actionDim_ = nJoints_;
    _obs.setZero(_obsDim);
    _obs_norm.setZero(_obsDim);

    _actorInput.setZero(OBSDIM+LATENTDIM);
    _adaptationInput.setZero(OBSDIM*NUM_HISTORY);
    _adaptationInput_norm.setZero(OBSDIM*NUM_HISTORY);
    adaptationOut_.setZero(LATENTDIM);
    normalized_latent.setZero(LATENTDIM);
    footPos_.setZero(4 * 3);
    command_.setZero();

    q_init.setZero(12);
    pTarget12_.setZero(12);
    pTarget12_actual.setZero(12);
    action.setZero(12);
    // Action.setZero(12);
    Action_filter.setZero(12);
    Action_scale.setZero(12);
    previousAction_.setZero(12);
    q_init << 0.1, 0.8, -1.5, -0.1, 0.8, -1.5, 0.1, 1, -1.5, -0.1, 1, -1.5;

    foot_Posx.setZero();
    foot_Posy.setZero();
    foot_Posz.setZero();

    this->kdMat.setZero();
    this->kpMat.setZero();
    rl_kp.setZero();
    rl_kd.setZero();
    _cmdVelSub = _nm.subscribe("/quad_cmd_vel", 1, &FSM_State_RL::cmdVelCallback, this);
    // ONXX 模型
    std::string model_path = ros::package::getPath("robot_control") + "/actor_model/actor.onnx";
    model_runner = new OnnxModelRunner(model_path);

    begin_ = std::chrono::steady_clock::now();
    std::cout<<"FSM_State_RLJointPD begin:"<<begin_.time_since_epoch().count()<<std::endl;

}

template <typename T>
void FSM_State_RL<T>::onEnter() {
    // Default is to not transition
    this->nextStateName = this->stateName;
    // Reset the transition data
    this->transitionData.zero();

    _obs.setZero(OBSDIM);
    _obs_norm.setZero(_obsDim);
    _adaptationInput_norm.setZero(OBSDIM*NUM_HISTORY);
    _actorInput.setZero(OBSDIM+LATENTDIM);
    adaptationOut_.setZero(LATENTDIM);
    historyWrapper.reset();
    historyWrapper.set_length(OBSDIM, NUM_HISTORY);
    historyWrapper.init_buffer();

    _jointQ << this->_data->_legController->datas[1].q,
               this->_data->_legController->datas[0].q,
               this->_data->_legController->datas[3].q,
               this->_data->_legController->datas[2].q;

    _jointQd <<this->_data->_legController->datas[1].qd,
               this->_data->_legController->datas[0].qd,
               this->_data->_legController->datas[3].qd,
               this->_data->_legController->datas[2].qd;
    // std::cout<<std::setprecision(3)<<"_jointQ:"<<_jointQ<<std::endl;
    pTarget12_ =_jointQ;
    action.setZero();
    // Action.setZero();
    Action_filter.setZero();
    Action_scale.setZero();
    previousAction_.setZero();//新添加

    pTarget12_actual = pTarget12_;
    for(int i=0; i<4; i++) {
        preCommands[i].zero();
    }
    emergency_stop = false;
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_RL<T>::run() {

    if(std::acos(this->_data->_stateEstimator->getResult().rBody.transpose().row(2)(2)) > 3.1415*30./180.) {
    }

    if (emergency_stop) return;

    end_ = std::chrono::steady_clock::now();

    if (std::chrono::duration_cast<std::chrono::microseconds>(end_ - begin_).count() > int(control_dt_ * 1000000)) { 
        begin_ = std::chrono::steady_clock::now(); 
    } else {
        for (int leg(0); leg < 4; ++leg) { 
            this->_data->_legController->commands[leg] = preCommands[leg];
        }
        return;
    }

    command_(0) =  _cmdVel.linearVel_x;
    command_(1) = _cmdVel.linearVel_y;
    command_(2) = _cmdVel.anguler_z;
    if (command_.norm() < 0.2)
        command_.setZero();

    this->kpMat = Vec3<T>(40, 40, 40).asDiagonal();
    this->kdMat = Vec3<T>(1, 1, 1).asDiagonal();

    updateObservation(); 
    updatePreviousActions();//新添加
    computeObservation();

    for (int i = 0; i < _obs.size(); i++) { 
        if (_obs(i) > 100)
            _obs(i) = 100.0;
        if (_obs(i) < -100)
            _obs(i) = -100.0;
    }
    std::chrono::steady_clock::time_point _end = std::chrono::steady_clock::now();
    std::vector<float> obs_data(_obs.data(), _obs.data() + _obs.size());
    std::vector<float> history_obs_data(_adaptationInput.data(), _adaptationInput.data() + _adaptationInput.size());

    std::vector<int64_t> obs_dims = {1, OBSDIM};
    std::vector<int64_t> history_dims = {1, NUM_HISTORY,OBSDIM};
    auto output_values = model_runner->RunInference(obs_data, obs_dims, history_obs_data, history_dims);
    action = Eigen::Map<Eigen::VectorXf>(output_values.data(), output_values.size());
    std::chrono::steady_clock::time_point _begin = std::chrono::steady_clock::now();

    for (int i = 0; i < pTarget12_.size(); i++) {
        if (action(i) > 100)
            action(i) = 100.0;
        if (action(i) < -100)
            action(i) = -100.0;
    }

    Action_filter=0.2*previousAction_+ 0.8*action;
    Action_scale=Action_filter*action_scale;
    Action_scale(0)=Action_scale(0)*hip_scale_reduction;
    Action_scale(3)=Action_scale(3)*hip_scale_reduction;
    Action_scale(6)=Action_scale(6)*hip_scale_reduction;
    Action_scale(9)=Action_scale(9)*hip_scale_reduction;
    pTarget12_ = Action_scale +q_init;

    pTarget12_actual(3) = pTarget12_(0);
    pTarget12_actual(4) = pTarget12_(1);
    pTarget12_actual(5) = pTarget12_(2);

    pTarget12_actual(0) = pTarget12_(3);
    pTarget12_actual(1) = pTarget12_(4);
    pTarget12_actual(2) = pTarget12_(5);

    pTarget12_actual(9) = pTarget12_(6);
    pTarget12_actual(10) = pTarget12_(7);
    pTarget12_actual(11) = pTarget12_(8);

    pTarget12_actual(6) = pTarget12_(9);
    pTarget12_actual(7) = pTarget12_(10);
    pTarget12_actual(8) = pTarget12_(11);
    this->_data->_legController->_legsEnabled = true;

    for (int leg(0); leg < 4; ++leg) { 
        for (int jidx(0); jidx < 3; ++jidx) {
            this->_data->_legController->commands[leg].qDes[jidx] = pTarget12_actual(leg * 3 + jidx);
            this->_data->_legController->commands[leg].qdDes[jidx] = 0.;
        }
        this->_data->_legController->commands[leg].kpJoint = this->kpMat;
        this->_data->_legController->commands[leg].kdJoint = this->kdMat;

        preCommands[leg] = this->_data->_legController->commands[leg];
    }

}

template <typename T>
FSM_StateName FSM_State_RL<T>::checkTransition() {
    this->nextStateName = this->stateName;

    iter++;

    // Switch FSM control mode
    switch (this->control_mode) {
        case K_RL:
            break;

        case K_PASSIVE:
            // Requested change to BALANCE_STAND
            this->nextStateName = FSM_StateName::PASSIVE;

            // Transition time is immediate
            this->transitionDuration = 0.0;

            break;

        default:
            std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                        << K_JOINT_CONTROL<< " to "
                        << this->control_mode << std::endl;
        }
    // Get the next state
    return this->nextStateName;
}

template <typename T>
TransitionData<T> FSM_State_RL<T>::transition() {
    // Switch FSM control mode
  switch (this->nextStateName) {
    case FSM_StateName::PASSIVE:
      this->turnOffAllSafetyChecks();
      this->transitionData.done = true;
      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                << K_JOINT_CONTROL << " to "
                << this->control_mode << std::endl;
  }
  // Finish transition
  this->transitionData.done = true;

  // Return the transition data to the FSM
  return this->transitionData;
}

// 安全检查
template<typename T>
bool FSM_State_RL<T>::locomotionSafe(){
    auto& seResult = this->_data->_stateEstimator->getResult();

    const T max_roll = 55;
    const T max_pitch = 55;

    for(int leg = 0; leg < 4; leg++){
        auto p_leg = this->_data->_legController->datas[leg].p;
        auto v_leg = this->_data->_legController->datas[leg].v.norm();
        auto v_joint = this->_data->_legController->datas[leg].qd;
    }

    return true;
}


template <typename T>
void FSM_State_RL<T>::onExit() {
    // Nothing to clean up when exiting
}

template <typename T>
void FSM_State_RL<T>::updateObservation() { //更新observation
    Vec4<T> isContact;
    isContact<< 0.5,0.5,0.5,0.5;

    this->_data->_stateEstimator->run();
    Vec3<float> up_z;
    up_z<<0,0,-1;

    _bodyHeight = this->_data->_stateEstimator->getResult().position(2);
    projected_gravity = this->_data->_stateEstimator->getResult().rBody*up_z;
    _jointQ << this->_data->_legController->datas[1].q,
               this->_data->_legController->datas[0].q,
               this->_data->_legController->datas[3].q,
               this->_data->_legController->datas[2].q;
    _bodyAngularVel << this->_data->_stateEstimator->getResult().omegaBody; 
    _jointQd <<this->_data->_legController->datas[1].qd,
               this->_data->_legController->datas[0].qd,
               this->_data->_legController->datas[3].qd,
               this->_data->_legController->datas[2].qd;

}

template <typename T>
void FSM_State_RL<T>::updatePreviousActions() { //新添加
    previousAction_ = action;
}

template <typename T>
void FSM_State_RL<T>::computeObservation() { 
    float  ang_vel=0.25, dof_pos = 1.0, dof_vel = 0.05, ang_gravity=1.0;
    Vec3<float> command_scal;
    command_scal<<2.,2.,0.25;

    _adaptationInput = historyWrapper.get_obs_history();
    _obs<<_bodyAngularVel*ang_vel, 
            projected_gravity*ang_gravity,
            command_(0) * command_scal(0), 
            command_(1) *command_scal(1),
            command_(2) *command_scal(2), 
            (_jointQ-q_init)*dof_pos,
            _jointQd*dof_vel,
            action;
    historyWrapper.add_obs(_obs);
}

template <typename T>
void FSM_State_RL<T>::cmdVelCallback(const unitree_legged_msgs::keyboardCmd& msg)
{
  _cmdVel.linearVel_x = msg.linearVel_x;
  _cmdVel.linearVel_y = msg.linearVel_y;
  _cmdVel.anguler_z = msg.anguler_z;
  _cmdVel.step_Frequency = msg.step_Frequency;
  _cmdVel.step_Height = msg.step_Height;
  _cmdVel.cppAD = msg.cppAD;
  _cmdVel.mode = msg.mode;
  _cmdVel.gait = msg.gait;
  _cmdVel.startFlag = msg.startFlag;
  _cmdVel.roll = msg.roll;
  _cmdVel.pitch = msg.pitch;
  _cmdVel.body_Height = msg.body_Height;
}


template class FSM_State_RL<float>;






