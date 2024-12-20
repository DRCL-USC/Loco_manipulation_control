#ifndef LOWLEVELSTATE_H
#define LOWLEVELSTATE_H

#include <iostream>
#include <common/cppTypes.h>
#include <common/enumClass.h>

struct UserValue{
    float lx;
    float ly;
    float rx;
    float ry;
    float L2;
    float vx; // vx in body frame
    float vy; // vy in body frame
    float turn_rate;
    float foothold[8];
    float manipulation_force[3];

    UserValue(){
        setZero();
    }
    void setZero(){
        lx = 0;
        ly = 0;
        rx = 0;
        ry = 0;
        L2 = 0;
        vx = 0;
        vy = 0;
        turn_rate = 0;
        for(int i = 0; i < 8; i++)
        {
            foothold[i] = 0;
        }
        for(int i = 0; i < 3; i++)
        {
            manipulation_force[i] = 0;
        }
    }
};

struct MotorState
{
    unsigned int mode;
    float q;
    float dq;
    float ddq;
    float tauEst;

    MotorState()
    {
        q = 0;
        dq = 0;
        ddq = 0;
        tauEst = 0;
    }
};

struct IMU
{
    float quaternion[4];
    float gyroscope[3];
    float accelerometer[3];

    IMU()
    {
        for(int i = 0; i < 3; i++)
        {
            quaternion[i] = 0;
            gyroscope[i] = 0;
            accelerometer[i] = 0;
        }
        quaternion[3] = 0;
    }
};

struct LowlevelState
{
    IMU imu;
    MotorState motorState[12];
    UserCommand userCmd;
    UserValue userValue;
    float FootForce[4];
    float Init_FootForce[4];
    float foothold_heuristic[8];

    float position[3]; // needed when we need to update the state estimation for high-level planner
    float vWorld[3];
    float rpy[3];

    LowlevelState()
    {
        for(int i = 0; i < 4; i++)
        {
            FootForce[i] = 0;
            Init_FootForce[i] = 0;
        }

        for(int i = 0; i < 3; i++)
        {
            position[i] = 0;
            vWorld[i] = 0;
            rpy[i] = 0;
        }
        for(int i = 0; i < 8; i++)
        {
            foothold_heuristic[i] = 0;
        }
    }
};


#endif //LowlevelState