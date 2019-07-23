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
#include <tf/transform_listener.h>

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

    tf::TransformListener odom_base_tf_listener;
    geometry_msgs::Vector3Stamped linear_body_vel{};
    linear_body_vel.header.stamp = ros::Time::now();
    linear_body_vel.header.frame_id = "camera_link";
    linear_body_vel.vector.x = 1.;
    linear_body_vel.vector.y = 0.;
    linear_body_vel.vector.z = 0.;

    // geometry_msgs::PoseStamped temp_pose;
    // temp_pose.pose.position.x = 0;
    // temp_pose.pose.position.y = 0;
    // temp_pose.pose.position.z = 2;
    // // Send a few setpoints before starting
    // for (int i = 100; ros::ok() && i > 0; --i)
    // {
    //     vwpp::PX4Interface::getInstance()->publishLocalPose(temp_pose);
    //     ros::spinOnce();
    //     rate.sleep();
    // }


    vwpp::PX4Interface::getInstance()->switchOffboard();
    vwpp::PX4Interface::getInstance()->unlockVehicle();


    vwpp::PIDController pid_controller_x(1.0, 0, 1.4);
    vwpp::PIDController pid_controller_y(1.0, 0, 1.4);
    vwpp::PIDController pid_controller_z(3.0, 0, 2.8);
    vwpp::PIDController pid_controller_yaw(0.5, 0, 1.2);


    pid_controller_x.setTarget(0.);
    pid_controller_y.setTarget(0.);
    pid_controller_z.setTarget(1.);
    // pid_controller_yaw.setTarget(vwpp::PX4Interface::getInstance()->getCurYaw());
    pid_controller_yaw.setTarget(vwpp::PX4Interface::getInstance()->getCurYaw() + M_PI);
    ROS_INFO("Target yaw: %lf", vwpp::PX4Interface::getInstance()->getCurYaw());

    // TODO rad

    while (ros::ok())
    {
        vwpp::PX4Interface::getInstance()->update();

        pid_controller_x.update(vwpp::PX4Interface::getInstance()->getCurX());
        pid_controller_y.update(vwpp::PX4Interface::getInstance()->getCurY());
        pid_controller_z.update(vwpp::PX4Interface::getInstance()->getCurZ());
        pid_controller_yaw.update(convertCurYaw2FabsYawThetaBetweenPI(pid_controller_yaw.getTarget(),
                                                                      vwpp::PX4Interface::getInstance()->getCurYaw()));
        // ROS_INFO("Current yaw: %lf", vwpp::PX4Interface::getInstance()->getCurYaw());

        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = pid_controller_x.output();
        cmd_vel.linear.y = pid_controller_y.output();
        cmd_vel.linear.z = pid_controller_z.output();
        // ROS_INFO("Cmd_vel X: %lf, Cmd_vel Y: %lf, Cmd_vel Z: %lf", cmd_vel.linear.x, cmd_vel.linear.y,
        //          cmd_vel.linear.z);
        cmd_vel.angular.x = 0;
        cmd_vel.angular.y = 0;
        cmd_vel.angular.z = pid_controller_yaw.output();
        // ROS_INFO("Yaw velocity output: %lf", cmd_vel.angular.z);
        // vwpp::PX4Interface::getInstance()->publishLocalVel(cmd_vel);


        // Test tf vector transformer
        geometry_msgs::Vector3Stamped linear_local_vel{};
        try
        {
            odom_base_tf_listener.transformVector("camera_odom_frame", linear_body_vel, linear_local_vel);
        }
        catch (tf::TransformException &tf_ex)
        {
            ROS_ERROR("%s", tf_ex.what());
            ros::Duration(1.0).sleep();
        }
        ROS_INFO("Transformed X: %lf\nTransformed Y: %lf\nTransformed Z: %lf\n", linear_local_vel.vector.x,
                 linear_local_vel.vector.y, linear_local_vel.vector.z);

        rate.sleep();
    }


    return 0;
}

