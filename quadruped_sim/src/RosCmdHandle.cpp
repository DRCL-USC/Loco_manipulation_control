#include "RosCmdHandle.h"

RosCmdHandle::RosCmdHandle()
{
    KeyBoard();
    FSM_sub = nh.subscribe("FSM", 1, &RosCmdHandle::FSMCallback, this);
}

void RosCmdHandle::FSMCallback(const std_msgs::String &msg)
{
    if (msg.data == "PASSIVE")
    {
        userCmd = UserCommand::L2_B;
    }
    else if (msg.data == "PDSTAND")
    {
        userCmd = UserCommand::L2_A;
    }
    else if (msg.data == "QPSTAND")
    {
        userCmd = UserCommand::L1_X;
    }
    else if (msg.data == "WALKING")
    {
        userCmd = UserCommand::START;
    }
    else if (msg.data == "MANIPULATION")
    {
        userCmd = UserCommand::L1_A;
    }
}
