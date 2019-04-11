// ------------------------------------------------------------ //
// This is the `main` file for the `push recovery controller`.
// It runs the node `push_recovery_ctrl` and defines its 
// publishers and subscribers, the controller is implemeneted 
// in the PR_ctrl class (see PushRecoveryClass.h/cpp)
//
// Edoardo Sorelli: edoardo.sorelli@gmail.com
// ------------------------------------------------------------ //

#include "ros/ros.h"

#include <std_msgs/Float32.h>
#include <imu_management/imu_data.h> 

#include "push_recovery_ctrl/PushRecoveryClass.h"

int main(int argc, char **argv)
{
    double rate = 50;                               // node frequency
    ros::init(argc, argv, "push_recovery_ctrl");    // node name
    
    ros::NodeHandle node;

    // ----- PUBLISHERS ----- //
    // The publishers publish the three stride parameters: Velocity, Step Length, Foot Clearance
    // The topics are read by `synergies_qb_legs_control.cpp`
    ros::Publisher vel_pub = node.advertise<std_msgs::Float32>("/synergies_des_speed",       5);
    ros::Publisher len_pub = node.advertise<std_msgs::Float32>("/synergies_des_length",      5);
    ros::Publisher fh_pub  = node.advertise<std_msgs::Float32>("/synergies_des_foot_height", 5);
    // publish value for DEBUG
    ros::Publisher ref_pub    = node.advertise<std_msgs::Float32>("/check_PushRecovery/ref",      2);
    ros::Publisher icp_pub    = node.advertise<std_msgs::Float32>("/check_PushRecovery/icp",      2);
    ros::Publisher fz1_pub    = node.advertise<std_msgs::Float32>("/check_PushRecovery/fz1",      2);
    ros::Publisher fz2_pub    = node.advertise<std_msgs::Float32>("/check_PushRecovery/fz2",      2);
    ros::Publisher rph_pub    = node.advertise<std_msgs::Bool>("/check_PushRecovery/robot_phase", 2);
    ros::Publisher v_deb_pub  = node.advertise<std_msgs::Float32>("/check_PushRecovery/v_debug",  2);
    ros::Publisher L_deb_pub  = node.advertise<std_msgs::Float32>("/check_PushRecovery/L_debug",  2);
    ros::Publisher fh_deb_pub = node.advertise<std_msgs::Float32>("/check_PushRecovery/fh_debug", 2);

    PR_ctrl controller;     // push recovery controller class
    // ----- SUBSCRIBERS ----- //
    // Activate-deactivate controller
    // Desired velocity
    // IMU data: pelvis forward velocity and orientation
    // Distance sensor: The robot velocity is obtained from it
    // Wrench sensors: foot soles contact forces
    // Robot joint coordinates
    ros::Subscriber on_off_sub = node.subscribe("/on_off_PRctrl", 1, &PR_ctrl::on_off_Callback, &controller);
    ros::Subscriber vel_sub    = node.subscribe("/desired_velocity", 1, &PR_ctrl::desired_velocity_Callback, &controller);
    ros::Subscriber imu_sub    = node.subscribe("/imu_data", 1, &PR_ctrl::imu_Callback, &controller);
    ros::Subscriber dist_sub   = node.subscribe("/current_vel", 1, &PR_ctrl::dist_sensor_Callback, &controller);
    ros::Subscriber right_wrenches_sub = node.subscribe("/wrenches/right_wrenches", 1, &PR_ctrl::right_wrenches_Callback, &controller);
    ros::Subscriber left_wrenches_sub  = node.subscribe("/wrenches/left_wrenches",  1, &PR_ctrl::left_wrenches_Callback, &controller);
    ros::Subscriber robot_pose_sub = node.subscribe("/qb_legs_measures", 1, &PR_ctrl::robot_pose_Callback, &controller);

    ROS_INFO("Push Recovery Controller online");
    ROS_INFO("----- Activate the Push Recovery controller:");
    ROS_INFO("rostopic pub -1 /on_off_PRctrl std_msgs/Bool 'data: true'");

    ros::Rate loop_rate(rate);

    while (ros::ok())
    {
        // evaluate Push Recovery Controller
        controller.evaluate_output();
        
        // Publish control values twenty times only when it changes (limit the publication) 
        if (controller.new_control <= 20) {
            if (controller.new_control == 20) ROS_INFO("push recovery action taken");
            vel_pub.publish(controller.v_ctrl);
            len_pub.publish(controller.L_ctrl);
            fh_pub.publish(controller.fh_ctrl);
        }

        // publish values for debug 
        // ref_pub.publish(controller.REF);
        icp_pub.publish(controller.ICP);
        // fz1_pub.publish(controller.filt_fz1);
        // fz2_pub.publish(controller.filt_fz2);
        // rph_pub.publish(controller.robot_phase);
        v_deb_pub.publish(controller.v_debug);
        // L_deb_pub.publish(controller.L_debug);
        // fh_deb_pub.publish(controller.fh_debug);

        // Manage the loop
        ros::spinOnce();
        loop_rate.sleep();
    }

    delete controller.fz1_filter;
    delete controller.fz2_filter;

    return 0;
}
