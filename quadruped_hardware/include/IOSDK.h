#ifndef IOSDK_H
#define IOSDK_H

#include <interface/IOInterface.h>
#include "WirelessHandle.h"
#include "interface/KeyBoard.h"

#ifdef GO1
#include "sdk_3_8_0/include/unitree_legged_sdk/quadruped.h"
#include "sdk_3_8_0/include/unitree_legged_sdk/udp.h"
#include "sdk_3_8_0/include/unitree_legged_sdk/safety.h"
#else
#include "sdk_3_3_1/include/unitree_legged_sdk/quadruped.h"
#include "sdk_3_3_1/include/unitree_legged_sdk/udp.h"
#include "sdk_3_3_1/include/unitree_legged_sdk/safety.h"
#endif


class IOSDK : public IOInterface
{
public:
    IOSDK(UNITREE_LEGGED_SDK::LeggedType robot, int cmd_panel_id);
    void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);

    UNITREE_LEGGED_SDK::UDP _udp;
    UNITREE_LEGGED_SDK::Safety _control;
    UNITREE_LEGGED_SDK::LowCmd _lowCmd = {0};
    UNITREE_LEGGED_SDK::LowState _lowState = {0};
};

#endif // IOSDK_H