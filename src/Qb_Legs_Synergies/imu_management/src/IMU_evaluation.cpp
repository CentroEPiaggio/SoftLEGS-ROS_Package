// ------------------------------------------------------------ //
// This runs the node `evaluate_IMU_data` and defines its 
// publishers and subscribers. 
// It takes the accelerations and angles as input and returns 
// the 'forward velocity' and `pitch`.
// The computation functions are implemented in the 'evaluate_IMU'
// class (see EvaluateIMUClass.h/cpp)
//
// Edoardo Sorelli: edoardo.sorelli@gmail.com
// ------------------------------------------------------------ //

#include "ros/ros.h"

#include <std_msgs/Float64.h>
#include <imu_management/imu_data.h>

#include "imu_management/EvaluateIMUClass.h"

int main(int argc, char **argv)
{
    float rate = 250;                              // node frequency
    ros::init(argc, argv, "evaluate_IMU_data");    // node name
    
    ros::NodeHandle node;

    // ----- PUBLISHERS ----- //
    // publish the IMU data elaborated for the push recovery controller
    ros::Publisher IMU_pub = node.advertise<imu_management::imu_data>("/imu_data", 1);
    // publish the acceleration value to debug the filters
    ros::Publisher raw_acc_pub  = node.advertise<std_msgs::Float64>("/check_filter/raw_acc",  1);
    ros::Publisher filt_acc_pub = node.advertise<std_msgs::Float64>("/check_filter/filt_acc", 1);
    ros::Publisher raw_vel_pub  = node.advertise<std_msgs::Float64>("/check_filter/raw_vel",  1);
    ros::Publisher filt_vel_pub = node.advertise<std_msgs::Float64>("/check_filter/filt_vel", 1);

    // ----- SUBSCRIBERS ----- //
    evaluate_IMU IMUeval(rate);     // IMU evaluator class
    // Desired velocity
    ros::Subscriber acc_sub    = node.subscribe("/qb_class_imu/acc",    1, &evaluate_IMU::acc_Callback, &IMUeval);
    ros::Subscriber angles_sub = node.subscribe("/qb_class_imu/angles", 1, &evaluate_IMU::angles_Callback, &IMUeval);


    ROS_INFO("IMU data evaluator online");

    ros::Rate loop_rate(rate);

    while (ros::ok())
    {
        // Evaluate data
        IMUeval.evaluate_IMU_data();
        
        // Publish IMU data 
        IMU_pub.publish(IMUeval.IMUdata);
        // publish acc and vel for debug
        raw_acc_pub.publish(IMUeval.raw_acc);
        filt_acc_pub.publish(IMUeval.filt_acc);
        raw_vel_pub.publish(IMUeval.raw_vel);
        filt_vel_pub.publish(IMUeval.filt_vel);

        // Manage the loop
        ros::spinOnce();
        loop_rate.sleep();
    }

    delete IMUeval.acc_filter;
    return 0;
}