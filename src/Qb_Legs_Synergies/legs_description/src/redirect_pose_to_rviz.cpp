// ------------------------------------------------------------ //
// This script subscribes to the topic of the robot joint value
// (geometry_msgs::Pose) and format the value to a message usable
// by rviz (sensor_msgs::JointState)
//
// Edoardo Sorelli: edoardo.sorelli@gmail.com
// ------------------------------------------------------------ //

#include "ros/ros.h"

#include <sensor_msgs/JointState.h> // message used by rviz
#include <geometry_msgs/Pose.h>     // message produce by SoftLegs

sensor_msgs::JointState joint_msgs;

// common constant
float deg2rad = 3.141592 / 180;
// joint bias
float bias[6] = {0, 0, 0, 0, 0, 0};  // deg
// float bias[6] = {20, -10, -5, 20, -10, -5};  // deg

void jointCallback(const geometry_msgs::Pose::ConstPtr &joint_pose)
{
    // left leg
    joint_msgs.position[4] = joint_pose->position.x    + (bias[0] * deg2rad);
    joint_msgs.position[5] = joint_pose->position.y    + (bias[1] * deg2rad);
    joint_msgs.position[6] = joint_pose->position.z    + (bias[2] * deg2rad);
    // right leg
    joint_msgs.position[7] = joint_pose->orientation.x - (bias[3] * deg2rad);
    joint_msgs.position[8] = joint_pose->orientation.y - (bias[4] * deg2rad);
    joint_msgs.position[9] = joint_pose->orientation.z - (bias[5] * deg2rad);
}

int main(int argc, char **argv)
{
    double rate = 30;                          // node frequency
    ros::init(argc, argv, "redirect_to_rviz"); // node name

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
    ros::Subscriber joint_sub = node.subscribe("/qb_legs_trajectories", 1, jointCallback);
    // ros::Subscriber joint_sub = node.subscribe("/qb_legs_measures", 1, jointCallback);

    ROS_INFO("'redirect_to_rviz' online");

    ros::Rate loop_rate(rate);

    while (ros::ok())
    {
        joint_pub.publish(joint_msgs);

        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}