// ------------------------------------------------------------ //
// This header file defines a class that implements the functions
// to elaborate the IMU data to be used in the push recovery control. 
//
// Edoardo Sorelli: edoardo.sorelli@gmail.com
// ------------------------------------------------------------ //

#ifndef IMU_MANAGEMENT_EVALUATE_IMU_CLASS_H
#define IMU_MANAGEMENT_EVALUATE_IMU_CLASS_H

#include "ros/ros.h"

#include <std_msgs/Float64.h>
#include <imu_management/imu_data.h>
#include <qb_interface/inertialSensorArray.h>
#include <qb_interface/anglesArray.h>

#include <math.h>   // sin(), cos()
#include <eigen3/Eigen/Eigen>
#include <imu_management/filt.h>

#define G           9.81  // gravity acceleration
// acceleration filter variables
#define FILT_T      BPF   // type of filter: LPF, HPF, BPF
#define N_TAPS      20    // number of taps the filter is going to use 
#define FL          0.005 // [kHz] lower transition frequencies (for BPF filters)
#define FU          0.020 // [kHz] upper transition frequencies (for BPF filters)

class evaluate_IMU {
    /// ---------- Class variables ---------- ///
    public:
        imu_management::imu_data IMUdata; // message to be published
        Filter *acc_filter;
        // acc and vel for debug
        std_msgs::Float64 raw_acc;
        std_msgs::Float64 filt_acc;
        std_msgs::Float64 raw_vel;
        std_msgs::Float64 filt_vel;
        Eigen::VectorXd array_acc;
    private:
        double xdd, ydd, zdd;   // raw acceleration value
        double r,   p,   y;     // raw roll, pitch, yaw value [degree]
        float  rate;            // node and sampling rate  
    /// ---------- Class functions ---------- ///
    public:
        // constructor
        evaluate_IMU(float node_rate);
        // Callbacks
        void acc_Callback(const qb_interface::inertialSensorArray &acc);
        void angles_Callback(const qb_interface::anglesArray &angles);
        // main function which evaluate the data
        void evaluate_IMU_data();
};

#endif //IMU_MANAGEMENT_EVALUATE_IMU_CLASS_H