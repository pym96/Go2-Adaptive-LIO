#ifndef CT_CONTROLLER_H
#define CT_CONTROLLER_H

// #include "../robot/RobotRunner.h"
#include "../robot/RobotController.h"
#include "FSM_States/ControlFSM.h"

class CT_Controller: public RobotController {
public:
	CT_Controller() {};
//	virtual ~CT_Controller() {}

	 void initializeController();
	 void runController();

//protected:
//	//控制里面需要一个状态机、一个目标状态
	ControlFSM<float>* _controlFSM;
//	DesiredStateCommand<float>* _desiredStateCommand;
};
#endif