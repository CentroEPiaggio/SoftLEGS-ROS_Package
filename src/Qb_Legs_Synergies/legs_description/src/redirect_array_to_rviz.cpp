// ------------------------------------------------------------ //
// This script subscribes to the topic (optimal_walk/trajectorie)
// of the robot joint value (std_msgs/Float64MultiArray) and 
// format the value to a message usable by rviz (sensor_msgs::JointState)
//
// Edoardo Sorelli: edoardo.sorelli@gmail.com
// ------------------------------------------------------------ //

#include "ros/ros.h"

#include <sensor_msgs/JointState.h>         // message used by rviz
#include <std_msgs/Float64MultiArray.h>     // message produce by SoftLegs

sensor_msgs::JointState joint_msgs;

void jointCallback(const std_msgs::Float64MultiArray::ConstPtr &joint_pose)
{
    joint_msgs.position[4] =   joint_pose->data[0];
    joint_msgs.position[5] =   joint_pose->data[1];
    joint_msgs.position[6] =   joint_pose->data[2];
    joint_msgs.position[7] = - joint_pose->data[3];
    joint_msgs.position[8] = - joint_pose->data[4];
    joint_msgs.position[9] = - joint_pose->data[5];
}

int main(int argc, char **argv)
{
    double rate = 30;                          // node frequency
    ros::init(argc, argv, "redirect_array_to_rviz"); // node name

    ros::NodeHandle node;

    // init const values -----
    joint_msgs.name     = {"", "", "", "", "", "", "", "", "", ""};
    joint_msgs.position = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    joint_msgs.name[0] = "base_to_upw_leg";
    joint_msgs.name[1] = "upw_to_fwd_leg";
    joint_msgs.name[2] = "fwd_to_pch_leg";
    joint_msgs.name[3] = "qb_legs_7_joint";
    joint_msgs.name[4] = "qb_legs_3_joint"; // right hip
    joint_msgs.name[5] = "qb_legs_2_joint"; // right knee
    joint_msgs.name[6] = "qb_legs_1_joint"; // right ankle
    joint_msgs.name[7] = "qb_legs_4_joint"; // left hip
    joint_msgs.name[8] = "qb_legs_5_joint"; // left knee
    joint_msgs.name[9] = "qb_legs_6_joint"; // left ankle

    joint_msgs.position[0] = 0;
    joint_msgs.position[1] = 0;
    joint_msgs.position[2] = 0;
    joint_msgs.position[3] = 0;
    
    // -------------------------

    // ----- PUBLISHERS ----- //
    // publishes the joint values properly formatted
    ros::Publisher joint_pub = node.advertise<sensor_msgs::JointState>("/my_data", 1);

    // ----- SUBSCRIBERS ----- //
    // subscribe to topic, which publishes the robot joint values
    ros::Subscriber joint_sub = node.subscribe("optimal_walk/trajectories", 1, jointCallback);

    ROS_INFO("'redirect_array_to_rviz' online");

    ros::Rate loop_rate(rate);

    while (ros::ok())
    {
        joint_pub.publish(joint_msgs);

        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}