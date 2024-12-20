#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>

#include <common/ControlFSMData.h>
#include <common/ContactEstimator.h>

#include <common/OrientationEstimator.h>
#include <common/PositionVelocityEstimator.h>
#include <FSM/FSM.h>

#include "IOSDK.h"
#include "ManipulationUDP.h"

bool running = true;

void ShutDown(int sig)
{
    std::cout << "stop" << std::endl;
    running = false;
}

void setProcessScheduler()
{
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1)
    {
        std::cout << "[ERROR] function set process scheduler failed \n ";
    }
}

int main(int argc, char **argv)
{
    setProcessScheduler();

    double dt = 0.001;
    int cmd_panel_id; // Wireless=1, keyboard=2
    
    if (argc > 1) {
        if(std::string(argv[1]) == "wireless") {
            cmd_panel_id = 1;
        } else if(std::string(argv[1]) == "keyboard") {
            cmd_panel_id = 2;
        } else {
            std::cout << "Invalid command panel. Please enter either 'wireless' or 'keyboard'." << std::endl;
            return 1;
        }
    }
    else {
        cmd_panel_id = 2; // Default to keyboard
    }

    IOInterface *ioInter;
    ManipulationUDP *manipulationIO;

#ifdef A1_ROBOT
    ioInter = new IOSDK(UNITREE_LEGGED_SDK::LeggedType::A1, cmd_panel_id);
    manipulationIO = new ManipulationUDP(8178);
    Quadruped quad("a1");
#elif ALIENGO
    ioInter = new IOSDK(UNITREE_LEGGED_SDK::LeggedType::Aliengo, cmd_panel_id);
    manipulationIO = new ManipulationUDP(8188);
    Quadruped quad("aliengo");
#elif GO1
    ioInter = new IOSDK(UNITREE_LEGGED_SDK::LeggedType::Go1, cmd_panel_id);
    manipulationIO = new ManipulationUDP(8198);
    Quadruped quad("go1");
#endif

    LegController *legController = new LegController(quad);
    LowlevelCmd *lowCmd = new LowlevelCmd();
    LowlevelState *lowState = new LowlevelState();
    StateEstimate stateEstimate;
    StateEstimatorContainer *stateEstimator = new StateEstimatorContainer(lowState,
                                                                          legController->data,
                                                                          &stateEstimate);

    stateEstimator->addEstimator<ContactEstimator>();
    stateEstimator->addEstimator<VectorNavOrientationEstimator>();
    stateEstimator->addEstimator<LinearKFPositionVelocityEstimator>();

    DesiredStateCommand *desiredStateCommand = new DesiredStateCommand(&stateEstimate, dt);

    ControlFSMData *_controlData = new ControlFSMData;
    _controlData->_quadruped = &quad;
    _controlData->_stateEstimator = stateEstimator;
    _controlData->_legController = legController;
    _controlData->_desiredStateCommand = desiredStateCommand;
    _controlData->_interface = ioInter;
    _controlData->_lowCmd = lowCmd;
    _controlData->_lowState = lowState;
    _controlData->_manipulationIO = manipulationIO;

    FSM *_FSMController = new FSM(_controlData);

    UNITREE_LEGGED_SDK::LoopFunc loop_control("control_loop", dt, boost::bind(&FSM::run, _FSMController));
    UNITREE_LEGGED_SDK::LoopFunc loop_udpSend("udp_send", dt, 3, boost::bind(&ControlFSMData::sendRecv, _controlData));

    loop_udpSend.start();
    loop_control.start();

    while (1)
    {
        +sleep(10);
    }

    delete _FSMController;
    delete _controlData;

    return 0;
}
