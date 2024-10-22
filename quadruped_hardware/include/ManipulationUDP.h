#ifndef ManipulationUDP_H
#define ManipulationUDP_H

#include <interface/ManipulationIO.h>
#include <messages/HighlevelCmd.h>
#include <boost/bind.hpp>

#ifdef GO1
#include "sdk_3_8_0/include/unitree_legged_sdk/loop.h"
#else
#include "sdk_3_3_1/include/unitree_legged_sdk/loop.h"
#endif

struct DataRecv
{
    float quaternion[4];
    float position[3];
    float velocity[3];
    float force[3];
    float omega[3];
    DataRecv()
    {
        for (int i = 0; i < 4; i++)
        {
            quaternion[i] = 0;
        }
        for (int i = 0; i < 3; i++)
        {
            position[i] = 0;
            velocity[i] = 0;
            force[i] = 0;
            omega[i] = 0;
        }
    }
};

class ManipulationUDP : public ManipulationIO
{
public:
    ManipulationUDP(int port);
    ~ManipulationUDP(){
        loop_loco_manipulation.shutdown();
    }
    void run(LowlevelState *_state);

private:
    int setupSocket(int port);
    int serverSocket;
    int clientSocket;
    DataRecv _dataRecv;

    void socketSendRecv();
    UNITREE_LEGGED_SDK::LoopFunc loop_loco_manipulation;
    HighlevelCmd Highcmd;
    Eigen::Matrix3d rotmat;
};

#endif // ManipulationUDP_H
