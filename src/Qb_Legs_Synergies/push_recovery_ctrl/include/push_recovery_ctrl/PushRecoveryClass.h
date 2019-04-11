// ------------------------------------------------------------ //
// This header file defines a class that implements the push recovery
// controller for the biped SoftLegs. 
// Where possible the function names and variable names are kept the
// same as the `push recovery controller` defined in MATLAB.
//
// Edoardo Sorelli: edoardo.sorelli@gmail.com
// ------------------------------------------------------------ //

#ifndef PR_CLASS_H
#define PR_LASS_H

#include "ros/ros.h"
#include <math.h>      // sin(), cos(), pow(), abs()
#include <algorithm>   // max()
#include <tuple>
#include <vector>
#include <algorithm>   // transform(); vector sum
#include <functional>  // plus(); vector sum

#include "controller_param.h"
#include <imu_management/filt.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <imu_management/imu_data.h>


class PR_ctrl {
    /// ---------- Class variables ---------- ///
    public:
        std_msgs::Float32 v_ctrl, L_ctrl, fh_ctrl;  // formatted output values as ROS msgs, if control deactive they are NULL
        int new_control;                            // state if the controller produces a new control value
        // msg for DEBUG
        std_msgs::Float32 REF;                      // CoM pos w.r.t. ankle position
        std_msgs::Float32 ICP;                      // ICP value for DEBUG
        std_msgs::Float32 filt_fz1;                 
        std_msgs::Float32 filt_fz2;
        std_msgs::Bool    robot_phase; 
        std_msgs::Float32 v_debug, L_debug, fh_debug;  // push recovery control value, for debug, these are never NULL
        // filter of the signal: fz1 & fz2  
        Filter *fz1_filter;             
        Filter *fz2_filter;                          
    private:
        bool   active_ctrl;             // switch on/off the controller
        double v_set, L_set, fh_set;    // controller output values
        double v_des, L_des, fh_des;    // desired values
        double v_max, L_max, fh_max;    // maximum values
        // values from sensors
        double robot_pose[6];           // robot pose
        double p_vel, p_pitch;          // pelvis position and orientation from IMU: in MATLAB are: ud, theta
        bool   fz1, fz2;                // 'true' if foot in contact else 'false'
        // walking states
        bool single_phase;              // single or double phase
        bool recovery;                  // recovery state
    /// ---------- Class functions ---------- ///
    public:
        // constructor
        PR_ctrl();
        // Callbacks
        void on_off_Callback(const std_msgs::Bool &on_off);
        void desired_velocity_Callback(const std_msgs::Float32 &des_vel);   
        void imu_Callback(const imu_management::imu_data &imu);
        void dist_sensor_Callback(const std_msgs::Float32 &vel);
        void right_wrenches_Callback(const std_msgs::Float64MultiArray &right_wrench);     
        void left_wrenches_Callback(const std_msgs::Float64MultiArray &left_wrench);      
        void robot_pose_Callback(const geometry_msgs::Pose &pose);
        // Evaluate push recovery controller
        void evaluate_output();
    private:
        // push recovery controller
        void evaluate_controller();
        void single_phase_controller(double icp, double step_length);
        void double_phase_controller(double icp, double step_length);
        // secondary functions
        double nominal_length(double v);
        void   check_robot_phase();
        std::tuple<double, double> evaluate_stability_param();
        std::tuple<double, double> ankles_x_position();
        std::vector<double>        vector_sum(std::vector<double> v1, std::vector<double> v2);
        void   check_v_max();
        void   check_L_max();
        void   check_fh_max();
};

#endif //PR_CLASS_H