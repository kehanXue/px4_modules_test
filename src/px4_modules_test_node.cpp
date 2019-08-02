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
#include <dynamic_reconfigure/server.h>


#include "px4_modules_test/fira_test_dynamic_cfgConfig.h"
#include "PX4Interface.h"
#include "PIDController.h"
#include "utils.h"

mavros_msgs::State current_state;

double_t altitude_p = 0;


void reconfig_cb(px4_modules_test::fira_test_dynamic_cfgConfig &config, uint32_t level)
{
    altitude_p = config.altitude_kp;
    ROS_INFO("Changed altitude: %lf", altitude_p);
}


int main(int argc, char** argv)
{
    // std::cout << "__       __"   << std::endl <<
    //           "\\ \\     / /" << std::endl <<
    //           " \\ \\   / / " << std::endl <<
    //           "  \\ \\_/ /  " << std::endl <<
    //           "   \\___/   " << std::endl;
    //
    std::cout << R"(__       __  _        __        _  | ___ \  | ___ \)" << std::endl <<
                 R"(\ \     / /  \\      //\\      //  | |_/ /  | |_/ / )" << std::endl <<
                 R"( \ \   / /    \\    //  \\    //   |  __/   |  __/   )" << std::endl <<
                 R"(  \ \_/ /      \\  //    \\  //    | |      | |       )" << std::endl <<
                 R"(   \___/        \\//      \\//     \_|      \_|     )" << std::endl;

    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh("~");

    dynamic_reconfigure::Server<px4_modules_test::fira_test_dynamic_cfgConfig> dyconfig_server;
    dynamic_reconfigure::Server<px4_modules_test::fira_test_dynamic_cfgConfig>::CallbackType dyconfig_type;
    dyconfig_type = boost::bind(&reconfig_cb, _1, _2);
    dyconfig_server.setCallback(dyconfig_type);

    // The setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20);

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

    tf::TransformListener odom_base_tf_listener(ros::Duration(2.));

    vwpp::PX4Interface::getInstance()->switchOffboard();
    vwpp::PX4Interface::getInstance()->unlockVehicle();


    vwpp::PIDController pid_controller_x(1.0, 0, 1.4, true, 0.8);
    vwpp::PIDController pid_controller_y(1.0, 0, 1.4);
    vwpp::PIDController pid_controller_z(3.0, 0, 3.4);
    vwpp::PIDController pid_controller_yaw(0.5, 0, 1.2);


    pid_controller_x.setTarget(0.);
    pid_controller_y.setTarget(-10.);
    pid_controller_z.setTarget(0.5);
    pid_controller_yaw.setTarget(vwpp::PX4Interface::getInstance()->getCurYaw());
    // pid_controller_yaw.setTarget(vwpp::PX4Interface::getInstance()->getCurYaw() + 3.5 * M_PI);
    // ROS_INFO("Target yaw: %lf", vwpp::PX4Interface::getInstance()->getCurYaw());

    // TODO rad

    int64_t cnt = 0;
    while (ros::ok())
    {

        vwpp::PX4Interface::getInstance()->update();

        // if (fabs(vwpp::PX4Interface::getInstance()->getCurZ() - pid_controller_z.getTarget()) <= 0.10)
        // {
        //     cnt++;
        //     if (cnt >= 25)
        //     {
        //         pid_controller_x.setTarget(3.0);
        //         cnt = 0;
        //     }
        // }

        pid_controller_x.update(vwpp::PX4Interface::getInstance()->getCurX());
        pid_controller_y.update(vwpp::PX4Interface::getInstance()->getCurY());
        pid_controller_z.update(vwpp::PX4Interface::getInstance()->getCurZ());
        pid_controller_yaw.update(convertCurYaw2FabsYawThetaBetweenPI(pid_controller_yaw.getTarget(),
                                                                      vwpp::PX4Interface::getInstance()->getCurYaw() +
                                                                      2 * M_PI));
        ROS_INFO("Target yaw: %lf", pid_controller_yaw.getTarget());
        ROS_INFO("Current yaw: %lf", vwpp::PX4Interface::getInstance()->getCurYaw());

        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = pid_controller_x.output();
        cmd_vel.linear.y = pid_controller_y.output();
        cmd_vel.linear.z = pid_controller_z.output();
        // ROS_INFO("Cmd_vel X: %lf, Cmd_vel Y: %lf, Cmd_vel Z: %lf", cmd_vel.linear.x, cmd_vel.linear.y,
        //          cmd_vel.linear.z);
        cmd_vel.angular.x = 0;
        cmd_vel.angular.y = 0;
        cmd_vel.angular.z = pid_controller_yaw.output();
        ROS_INFO("Yaw velocity output: %lf", cmd_vel.angular.z);
        // vwpp::PX4Interface::getInstance()->publishSetpointVel(cmd_vel);


        mavros_msgs::PositionTarget position_target;
        position_target.coordinate_frame = position_target.FRAME_LOCAL_NED;
        position_target.type_mask =
                position_target.IGNORE_AFX | position_target.IGNORE_AFY | position_target.IGNORE_AFZ |
                // position_target.IGNORE_PX | position_target.IGNORE_PY |
                position_target.IGNORE_PZ |
                // position_target.IGNORE_VZ |
                // position_target.IGNORE_VY | position_target.IGNORE_VX |
                position_target.IGNORE_YAW_RATE;
        position_target.velocity.x = pid_controller_x.output();
        position_target.velocity.y = pid_controller_y.output();
        position_target.velocity.z = pid_controller_z.output();
        // position_target.position.x = pid_controller_x.getTarget();
        // position_target.position.y = pid_controller_y.getTarget();
        // position_target.position.z = pid_controller_z.getTarget();
        position_target.yaw = pid_controller_yaw.getTarget();
        // position_target.yaw_rate = pid_controller_yaw.output();
        vwpp::PX4Interface::getInstance()->publishSetpointRaw(position_target);

        // vwpp::TargetPosXYZYaw target_pos_xyz_yaw{};
        // target_pos_xyz_yaw.px = pid_controller_x.getTarget();
        // target_pos_xyz_yaw.py = pid_controller_y.getTarget();
        // target_pos_xyz_yaw.pz = pid_controller_z.getTarget();
        // target_pos_xyz_yaw.yaw = pid_controller_yaw.getTarget();
        // vwpp::PX4Interface::getInstance()->publishTarget(target_pos_xyz_yaw);

        // vwpp::TargetVelXYPosZYaw target_vel_xy_pos_z_yaw{};
        // target_vel_xy_pos_z_yaw.vx = pid_controller_x.output();
        // target_vel_xy_pos_z_yaw.vy = pid_controller_y.output();
        // target_vel_xy_pos_z_yaw.pz = pid_controller_z.getTarget();
        // target_vel_xy_pos_z_yaw.yaw = pid_controller_yaw.getTarget();
        // vwpp::PX4Interface::getInstance()->publishTarget(target_vel_xy_pos_z_yaw);

        geometry_msgs::Vector3Stamped
                linear_body_vel{};
        linear_body_vel.header.stamp = ros::Time::now();
        linear_body_vel.header.frame_id = "camera_link";
        linear_body_vel.vector.x = 1.;
        linear_body_vel.vector.y = 0.;
        linear_body_vel.vector.z = 0.;

        // Test tf vector transformer
        geometry_msgs::Vector3Stamped linear_local_vel{};
        try
        {
            // odom_base_tf_listener.transformVector("camera_odom_frame", ros::Time(1), linear_body_vel,
            //                                       "camera_odom_frame", linear_local_vel);
            //
            odom_base_tf_listener.transformVector("camera_odom_frame", linear_body_vel, linear_local_vel);
        }
        catch (tf::TransformException &tf_ex)
        {
            // ROS_ERROR("%s", tf_ex.what());
            ros::Duration(0.2).sleep();
        }
        // ROS_INFO("Transformed X: %lf\nTransformed Y: %lf\nTransformed Z: %lf\n", linear_local_vel.vector.x,
        //          linear_local_vel.vector.y, linear_local_vel.vector.z);

        rate.sleep();
    }


    return 0;
}

