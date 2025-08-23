//
// Created by jh on 23-3-30.
//

#ifndef SDUOG_GUIDE_IOINTERFACE_H
#define SDUOG_GUIDE_IOINTERFACE_H

#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include <string>

class IOInterface{
public:
    IOInterface(){}
    ~IOInterface(){
        //delete cmdPanel;
    }
    virtual void sendRecv(const LowlevelCmd *cmd, LowlevelState *state) = 0;
    bool _userFlag;
    //void zeroCmdPanel(){cmdPanel->setZero();}
    //void setPassive(){cmdPanel->setPassive();}

protected:
    //CmdPanel *cmdPanel;
};

#endif //SDUOG_GUIDE_IOINTERFACE_H
