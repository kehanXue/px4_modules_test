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
#include <cmath>

#include "PX4Interface.h"
#include "PIDController.h"
#include "utils.h"

mavros_msgs::State current_state;


void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh("~");

    // The setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20);


    geometry_msgs::PoseStamped temp_pose;
    temp_pose.pose.position.x = 0;
    temp_pose.pose.position.y = 0;
    temp_pose.pose.position.z = 2;
    // Send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        vwpp::PX4Interface::getInstance()->publishLocalPose(temp_pose);
        ros::spinOnce();
        rate.sleep();
    }


    vwpp::PX4Interface::getInstance()->switchOffboard();
    vwpp::PX4Interface::getInstance()->unlockVehicle();


    vwpp::PIDController pid_controller_x(1.0, 0, 1.0);
    vwpp::PIDController pid_controller_y(1.0, 0, 1.0);
    vwpp::PIDController pid_controller_z(1.0, 0, 1.0);
    vwpp::PIDController pid_controller_yaw(1, 0, 0.5);


    pid_controller_x.setTarget(0.);
    pid_controller_y.setTarget(0.);
    pid_controller_z.setTarget(2.);
    pid_controller_yaw.setTarget(vwpp::PX4Interface::getInstance()->getCurYaw() + M_PI);
    ROS_INFO("Target yaw: %lf", vwpp::PX4Interface::getInstance()->getCurYaw() + M_PI);

    // TODO rad

    while (ros::ok())
    {
        vwpp::PX4Interface::getInstance()->update();

        pid_controller_x.update(vwpp::PX4Interface::getInstance()->getCurX());
        pid_controller_y.update(vwpp::PX4Interface::getInstance()->getCurY());
        pid_controller_z.update(vwpp::PX4Interface::getInstance()->getCurZ());
        pid_controller_yaw.update(convertCurYaw2FabsYawThetaBetweenPI(pid_controller_yaw.getTarget(),
                                                                      vwpp::PX4Interface::getInstance()->getCurYaw()));
        ROS_INFO("Current yaw: %lf", vwpp::PX4Interface::getInstance()->getCurYaw());

        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = pid_controller_x.output();
        cmd_vel.linear.y = pid_controller_y.output();
        cmd_vel.linear.z = pid_controller_z.output();
        cmd_vel.angular.x = 0;
        cmd_vel.angular.y = 0;
        cmd_vel.angular.z = pid_controller_yaw.output();
        ROS_INFO("yaw velocity output: %lf", cmd_vel.angular.z);
        vwpp::PX4Interface::getInstance()->publishLocalVel(cmd_vel);

        rate.sleep();
    }


    return 0;
}

