#include "ManipulationRos.h"

ManipulationRos::ManipulationRos(Quadruped quad) : ManipulationIO(), _quad(quad)
{
    manipulation_force_sub = nh.subscribe("wrench", 1, &ManipulationRos::ManiForceCallback, this);
    object_sub[0] = nh.subscribe("cmd_vel", 1, &ManipulationRos::cmdvelCallback, this);
    object_sub[1] = nh.subscribe("contactPoint", 1, &ManipulationRos::poseCallback, this);

    // create a ROS timer
        timer = nh.createTimer(ros::Duration(0.5), &ManipulationRos::timerCallback, this);
}

void ManipulationRos::run(LowlevelState *_state)
{

    Eigen::Quaterniond quat(_state->imu.quaternion[0], _state->imu.quaternion[1], _state->imu.quaternion[2], _state->imu.quaternion[3]);
    rotmat = quat.toRotationMatrix();
    pose_world = (Eigen::Vector3d() << _state->position[0], _state->position[1], 0).finished();

    if (planner_running)
    {
        for (int i = 0; i < 2; i++)
        {
            _state->userValue.manipulation_force[i] = Highcmd.manipulation_force(i);
        }
        _state->userValue.vx = Highcmd.velocity_cmd[0];
        _state->userValue.vy = Highcmd.velocity_cmd[1];
        _state->userValue.turn_rate = Highcmd.omega_cmd[2];
    }
    else
    {
        for (int i = 0; i < 2; i++)
        {
            _state->userValue.manipulation_force[i] = 0;
        }
        _state->userValue.vx = 0;
        _state->userValue.vy = 0;
        _state->userValue.turn_rate = 0;

        ROS_INFO_THROTTLE(1, "Waiting for new Target");
    }
}

void ManipulationRos::ManiForceCallback(const geometry_msgs::Wrench &msg)
{
    Eigen::Vector3d force_body = (Eigen::Vector3d() << msg.force.x, msg.force.y, msg.force.z).finished();
    Highcmd.manipulation_force = rotmat * force_body;
    msg_received = true;
    // ROS_INFO("I heard: x =%f, y=%f, z=%f", Highcmd.manipulation_force[0], Highcmd.manipulation_force[1], Highcmd.manipulation_force[2]);
}

void ManipulationRos::cmdvelCallback(const geometry_msgs::Twist &msg)
{
    cmd_body << msg.linear.x, msg.linear.y, 0.0;
    Highcmd.omega_cmd[2] = msg.angular.z;
    msg_received = true;
}

void ManipulationRos::poseCallback(const geometry_msgs::Pose &msg)
{
    msg_received = true;

    Eigen::Vector3d contact_point_world = (Eigen::Vector3d() << msg.position.x, msg.position.y, 0).finished();
    Eigen::Vector3d distance_body = rotmat.transpose() * (contact_point_world - pose_world);

    Highcmd.velocity_cmd[0] = 2 * (distance_body[0] - _quad.leg_offset_x);  
    Highcmd.velocity_cmd[1] = cmd_body[1] + 3 * (distance_body[1]); // magic number
    //   ROS_INFO("I heard: x =%f, y=%f, z=%f", Highcmd.velocity_cmd[0], Highcmd.velocity_cmd[1], Highcmd.omega_cmd[2]);
}

void ManipulationRos::timerCallback(const ros::TimerEvent &)
{
    if (msg_received)
    {
        msg_received = false;
        planner_running = true;
    }
    else
    {
        planner_running = false;
    }
}