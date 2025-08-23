#include "CT_Controller.h"

//初始化状态机，状态机里有步态管理器，目标状态管理器
void CT_Controller::initializeController() {

   _controlFSM = new ControlFSM<float>(_humanoid, _stateEstimator,_legController);//

}

//机器人基本的腿部初始化完成后执行具体的控制器的入口
void CT_Controller::runController() {
    // Run the Control FSM code
    _controlFSM->runFSM();
}