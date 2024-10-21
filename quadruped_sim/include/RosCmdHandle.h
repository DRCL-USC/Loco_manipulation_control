#include <interface/KeyBoard.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

class RosCmdHandle : public KeyBoard
{
public:
    RosCmdHandle();

private:
    ros::Subscriber FSM_sub;
    ros::NodeHandle nh;
    void FSMCallback(const std_msgs::String& msg);
};