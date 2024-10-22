#ifndef ManipulationROS_H
#define ManipulationROS_H

#include "ros/ros.h"
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <messages/HighlevelCmd.h>
#include <messages/LowlevelState.h>
#include <common/Quadruped.h>
#include <interface/ManipulationIO.h>

class ManipulationRos : public ManipulationIO
{
public:
    ManipulationRos(Quadruped quad);
    void run(LowlevelState *_state);

private:

    ros::NodeHandle nh;
    ros::Subscriber manipulation_force_sub, object_sub[2];
    void ManiForceCallback(const geometry_msgs::Wrench &msg);
    void cmdvelCallback(const geometry_msgs::Twist &msg);
    void poseCallback(const geometry_msgs::Pose &msg);
    void timerCallback(const ros::TimerEvent &);
    bool msg_received = false;
    bool planner_running = false;
    HighlevelCmd Highcmd;
    Eigen::Vector3d cmd_body;
    Eigen::Matrix3d rotmat;
    Eigen::Vector3d pose_world;
    ros::Timer timer;
    Quadruped _quad;
};

#endif // ManipulationROS_H
