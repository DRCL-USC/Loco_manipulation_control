
#include "IOSDK.h"
#include <stdio.h>

IOSDK::IOSDK(UNITREE_LEGGED_SDK::LeggedType robot, int cmd_panel_id) : _control(robot),
#ifdef GO1
                                                                                 _udp(UNITREE_LEGGED_SDK::LOWLEVEL, 8090, "192.168.123.10", 8007)
#else
                                                                                 _udp(UNITREE_LEGGED_SDK::LOWLEVEL)
#endif
{

    _udp.InitCmdData(_lowCmd);
    if (cmd_panel_id == 1)
    {
        cmdPanel = new WirelessHandle(&_lowState);
    }
    else if (cmd_panel_id == 2)
    {
        cmdPanel = new KeyBoard();
    }
}

void IOSDK::sendRecv(const LowlevelCmd *cmd, LowlevelState *state)
{
    _udp.Recv();
    _udp.GetRecv(_lowState);
    for (int i(0); i < 12; ++i)
    {
        _lowCmd.motorCmd[i].mode = 0X0A;
        _lowCmd.motorCmd[i].q = cmd->motorCmd[i].q;
        _lowCmd.motorCmd[i].dq = cmd->motorCmd[i].dq;
        _lowCmd.motorCmd[i].Kp = cmd->motorCmd[i].Kp;
        _lowCmd.motorCmd[i].Kd = cmd->motorCmd[i].Kd;
        _lowCmd.motorCmd[i].tau = cmd->motorCmd[i].tau;
    }

    for (int i(0); i < 12; ++i)
    {
        state->motorState[i].q = _lowState.motorState[i].q;
        state->motorState[i].dq = _lowState.motorState[i].dq;
        state->motorState[i].tauEst = _lowState.motorState[i].tauEst;
        state->motorState[i].mode = _lowState.motorState[i].mode;
    }

    for (int i(0); i < 3; ++i)
    {
        state->imu.quaternion[i] = _lowState.imu.quaternion[i];
        state->imu.gyroscope[i] = _lowState.imu.gyroscope[i];
        state->imu.accelerometer[i] = _lowState.imu.accelerometer[i];
    }
    state->imu.quaternion[3] = _lowState.imu.quaternion[3];

    for (int i = 0; i < 4; i++)
    {
        state->FootForce[i] = _lowState.footForce[i];
    }

    state->userCmd = cmdPanel->getUserCmd();
    state->userValue = cmdPanel->getUserValue();

    // _control.PowerProtect(_lowCmd, _lowState, 10);

    _udp.SetSend(_lowCmd);
    _udp.Send();
}