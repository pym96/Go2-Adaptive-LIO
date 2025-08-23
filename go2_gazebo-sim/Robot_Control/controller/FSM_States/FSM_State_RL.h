#ifndef FSM_STATE_RL_H
#define FSM_STATE_RL_H
#include "FSM_States.h"
#include <cstring>
#include <experimental/filesystem>
#include "cpuMLP.h"
#include "../common/StateEstimatorContainer.h"
#include "../common/LegController.h"
#include "./history_wrapper.h"
#include <ros/ros.h>
#include <ros/package.h>
#include "unitree_legged_msgs/keyboardCmd.h"
#include <cpu_provider_factory.h>
#include <onnxruntime_cxx_api.h>

#define OBSDIM 45
#define LATENTDIM 16
#define NUM_HISTORY 10
#define DEPTH 1

class OnnxModelRunner {
public:
    OnnxModelRunner( std::string& model_path) {
        // 初始化 ONNX Runtime 环境
        env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "OnnxModelRunner");
        
        // 创建会话选项
        session_options_ = std::make_unique<Ort::SessionOptions>();
        session_options_->SetIntraOpNumThreads(1);
        Ort::ThrowOnError(OrtSessionOptionsAppendExecutionProvider_CPU(*session_options_, 1));
        
        // 加载 ONNX 模型
        session_ = std::make_unique<Ort::Session>(*env_, model_path.c_str(), *session_options_);
        
        // 获取模型输入信息
        Ort::AllocatorWithDefaultOptions allocator;
        input_name_ = session_->GetInputNameAllocated(0, allocator).get();
        history_input_name_ = session_->GetInputNameAllocated(1, allocator).get();

        input_shape_ = session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        history_input_shape_ = session_->GetInputTypeInfo(1).GetTensorTypeAndShapeInfo().GetShape();
        
        // 打印模型信息
        std::cout << "Model loaded: " << model_path << std::endl;
        std::cout << "Input names: " << input_name_ << ", " << history_input_name_ << std::endl;
        std::cout << "Input shapes: [";
        for ( auto& dim : input_shape_) std::cout << dim << " ";
        std::cout << "], [";
        for ( auto& dim : history_input_shape_) std::cout << dim << " ";
        std::cout << "]" << std::endl;
    }

    std::vector<float> RunInference( std::vector<float>& obs_data, 
                                     std::vector<int64_t>& obs_dims, 
                                     std::vector<float>& history_obs_data, 
                                     std::vector<int64_t>& history_dims, 
                                     const std::string& output_name = "actions") {
        // 创建内存信息
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

        // 创建输入张量
        Ort::Value obs_tensor = Ort::Value::CreateTensor<float>(
            memory_info, obs_data.data(), obs_data.size(), obs_dims.data(), obs_dims.size());
        Ort::Value history_tensor = Ort::Value::CreateTensor<float>(
            memory_info, history_obs_data.data(), history_obs_data.size(), history_dims.data(), history_dims.size());

        // 输入名称数组
        std::vector<const char*> input_node_names = {input_name_.c_str(), history_input_name_.c_str()};

        // 输入张量数组
        std::vector<Ort::Value> ort_inputs;
        ort_inputs.push_back(std::move(obs_tensor));
        ort_inputs.push_back(std::move(history_tensor));

        // 输出名称数组
        std::vector<const char*> output_node_names = {output_name.c_str()};

        // 推理
        auto output_tensors = session_->Run(
            Ort::RunOptions{nullptr}, 
            input_node_names.data(), 
            ort_inputs.data(), 
            ort_inputs.size(), // 输入数量
            output_node_names.data(), 
            1                   // 输出数量
        );

        auto output_shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
        auto* output_data = output_tensors[0].GetTensorData<float>();

        std::vector<float> output_values(output_tensors[0].GetTensorTypeAndShapeInfo().GetElementCount());
        std::memcpy(output_values.data(), output_data, sizeof(float) * output_values.size());

        return output_values;
    }

    std::unique_ptr<Ort::Env> env_;
    std::unique_ptr<Ort::SessionOptions> session_options_;
    std::unique_ptr<Ort::Session> session_;
    std::string input_name_;
    std::string history_input_name_;
    std::vector<int64_t> input_shape_;
    std::vector<int64_t> history_input_shape_;
};


template <typename T>
class FSM_State_RL : public FSM_State<T> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FSM_State_RL(ControlFSMData* _controlFSMData);
    void onEnter();
    void run();
    FSM_StateName checkTransition();
    TransitionData<T> transition();
    void onExit();
    virtual void updateObservation();
    virtual void updatePreviousActions();
    virtual void computeObservation();

private:
    bool locomotionSafe();
    int iter = 0;
    DVec<T> _ini_jpos;
    Vec3<double> rl_kp,rl_kd;
    Vec3<float> rl_kpf,rl_kdf;

protected:

    float _bodyHeight;

    Eigen::Matrix<float, LATENTDIM, 1> normalized_latent;
    Eigen::Matrix<float, 3, 1> _bodyOri;
    Eigen::Matrix<float, 12, 1> _jointQ; 
    Eigen::Matrix<float, 3, 1> _bodyVel; 
    Eigen::Matrix<float, 3, 1> _bodyAngularVel;
    Eigen::Matrix<float, 3, 1> projected_gravity; 
    Eigen::Matrix<float, 12, 1> _jointQd; 
    Eigen::Matrix<float, OBSDIM, 1> _obs, _obs_norm;
    Eigen::Matrix<float, LATENTDIM, 1> adaptationOut_;
    Eigen::Matrix<float, OBSDIM+LATENTDIM, 1> _actorInput;
    Eigen::Matrix<float, OBSDIM*NUM_HISTORY, 1> _adaptationInput, _adaptationInput_norm;

    int _obsDim, historyLength_, nJoints_, actionDim_;
    Eigen::VectorXf _obsMean, _obsVar;
    Eigen::VectorXf q_init;
    Eigen::VectorXf pTarget12_, pTarget12_prev_,pTarget12_actual;
    Eigen::VectorXf action,previousAction_,Action,Action_filter,Action_scale;
    Eigen::VectorXf footPos_;  
    Eigen::Vector3f command_;  
    Eigen::Vector4f foot_Posx, foot_Posy, foot_Posz;
    std::string _loadPath;

    std::chrono::steady_clock::time_point begin_;
    std::chrono::steady_clock::time_point end_;

    rai::FuncApprox::MLP_fullyconnected<float, OBSDIM + LATENTDIM,   12,       rai::FuncApprox::ActivationType::elu> policy;
    rai::FuncApprox::MLP_fullyconnected<float, OBSDIM * NUM_HISTORY,   LATENTDIM,  rai::FuncApprox::ActivationType::elu> adaptation;

    double control_dt_ = 0.02;
    bool emergency_stop = false;
    LegControllerCommand preCommands[4];
    float action_scale=0.25;
    float hip_scale_reduction=0.5;

    wrapper::History_wrapper<float, DEPTH> historyWrapper;

    ros::Subscriber _cmdVelSub;
    ros::NodeHandle _nm;
    unitree_legged_msgs::keyboardCmd _cmdVel;
    void cmdVelCallback(const unitree_legged_msgs::keyboardCmd& msg);

    // ONNX
    // 初始化 OnnxModelRunner
    OnnxModelRunner *model_runner;
};


#endif //FSM_STATE_RL_H
