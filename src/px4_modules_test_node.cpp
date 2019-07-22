//
// Created by kehan on 19-7-22.
//

/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "PX4Interface.h"

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh("~");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20);


    geometry_msgs::PoseStamped temp_pose;
    temp_pose.pose.position.x = 0;
    temp_pose.pose.position.y = 0;
    temp_pose.pose.position.z = 2;
    // send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        vwpp::PX4Interface::getInstance()->publishLocalPose(temp_pose);
        ros::spinOnce();
        rate.sleep();
    }

    vwpp::PX4Interface::getInstance()->switchOffboard();
    vwpp::PX4Interface::getInstance()->unlockVehicle();

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 1;


    while (ros::ok())
    {
        vwpp::PX4Interface::getInstance()->publishLocalVel(cmd_vel);
        // vwpp::PX4Interface::getInstance()->publishLocalPose(temp_pose);
        vwpp::PX4Interface::getInstance()->update();
        ROS_INFO("current z: %lf", vwpp::PX4Interface::getInstance()->getCurZ());
        rate.sleep();
    }


    return 0;
}

