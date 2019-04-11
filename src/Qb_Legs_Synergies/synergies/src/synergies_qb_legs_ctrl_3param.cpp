/* Code provided by Edoardo Sorelli, derived from 'synergies_qb_legs_control.cpp'
 * provided by Gian Maria Gasparri Ph.D. Centro E. Piaggio 
 * 
 * This code allows the robot to transit from different parametrization of the stride. 
 *
 * In the specific, trajectory references are returned by a synergy mapping function
 * which takes the speed and foot height desired values as inputs. 
 * A service allows the user to modify the desired speed. 
 * (IN FUTURE this service will be activated by a speed control.)
 * 
 * Once the desired speed is modified the new trajectories are provided. 
 * Hence previous signals and newest ones has to be coherently concatenated to provide
 * a transient as smooth as possible. The transition occurs  when the norm position error is minimum. 
 * (FUTURE WORK will consider also the speed, i.e. the robot state.)
 * 
 * 
 * The code can be shared in several sections
 * 1 - INIT - Create publisher message and service in order to provide signals and to modify them
 *     when necessary.
 * 2 - INIT - Create matrix and vector support.
 * 3 - TRAJ - Obtain interpolated trajectories exploiting a synergy mapping function. This function
 *     returns resampled data as function of the publishing rate.
 * 4 - TRANS - Find the best transition instant.
 * 5 - TRANS - Wait for it and then switch.
 * 6 - PUB - Publish new trajectories
 * 
*/

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

#include <unistd.h>
#include <string>
#include <array>
#include <math.h>
#include <boost/concept_check.hpp>
#include <std_msgs/String.h>
#include <ros/node_handle.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>

#include <synergies/traj_state.h>
#include <synergies/bias.h>
#include <synergies/data_corr_AT.h>
#include <synergies/gain_bias_err.h>
#include <synergies/spline.h>
#include "synergies/synergy_values.h"

#include <synergies/speed_srv.h>
#include <synergies/warm_start_srv.h>
#include <synergies/warm_stop_srv.h>

#include <synergies/synergy_map_fun.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <map>

#include <fstream>
#include <ctime>
#include <dirent.h>

#include <synergies/data_in_common.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>

// GLOBAL VARIABLES ---------------------------------------------

std::map<int, trajectory_msgs::JointTrajectory> msgs;

const double pi = 3.141592;

double exp_val = 0.56; // beta value in paper equation
double l = 0.3;        // step length
double g = 9.81;       // gravity acceleration

// stride parameters variables
double speed  = 0.0;
double foot_l = 0.05;
double foot_h = 0.05;

// track if the there is a step length or height variation
bool change_step_length = false;
bool change_step_height = false;

//if speed variation is less then 0.05 m/s ingnore it
double min_speed = 0.03;

double Current_Speed = min_speed;

double Var_speed_threshold = 0.005;

double l_des_min = 0.03;  // [m]
double l_des_max = 0.2;   // [m]
double h_des_max = 0.1;   // [m]

double v_des_max = 0.5;   // [m/s]
double v_des_min;

bool Warm_Start = false;
bool Warm_Stop  = false;
// to move the robot through the distance sensor
// bool flag_sensor_subsribe_vel = true;

bool wanna_use_bias_stop = false;
bool wanna_use_bias_trunk_ankle = false;

double interp_Instant = 1;

bool flag_pub_qbLegs_robot = true;

bool flag_concatenate_completed = true;

// BIAS when the robot is standing
std::vector<double> bias = {4, 0, -9, 4, -10, 5}; // [deg]
std::vector<double> old_bias = bias;
std::vector<double> bias_stop = {4, 0, -9, 4, -10, 5};

// tk::spline bias_s;

double try_current;
double index_time(0);
double min_v_des = min_speed;

double Trunk_Gain(0);
double min_Trunk(-10);
double max_Trunk(10);

double Ankle_Gain(-10);
double min_Ankle(17);
double max_Ankle(20);

double err_tilde(0);
double bias_err_tilde_T(0);
double bias_err_tilde_A(0);
double BT(0);

synergies::data_corr_AT data_corr_AT_msg;
ros::Publisher pub_data_corr;
ros::Time begin;
ros::Duration elapsed_time;

// ---------------------------------------- END GLOBAL VARIABLES

//callback update v min max
void callback_update_min_max_v(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    v_des_min = msg->data[0];
    min_speed = v_des_min;
    v_des_max = msg->data[1];
    std::cout << "Update V min " << v_des_min << "V max " << v_des_max << std::endl;
}

// -------------- DEFINE SERVICES -------------- //

// Service to modify speed
bool setSpeedCallback(synergies::speed_srv::Request &request, synergies::speed_srv::Response &response)
{
    ROS_INFO("Desired Speed : %lf ", request.speed);
    Current_Speed = request.speed;

    return true;
}

bool setWarmStartCallback(synergies::warm_start_srv::Request &request, synergies::warm_start_srv::Response &response)
{
    ROS_INFO("Warm_Start");
    Warm_Start = true;
    ROS_INFO("Warm %d", Warm_Start);

    return true;
}

bool setWarmStopCallback(synergies::warm_stop_srv::Request &request, synergies::warm_stop_srv::Response &response)
{
    ROS_INFO("Warm_Stop");
    Warm_Stop = request.Wsto;
    ROS_INFO("Warm %d", Warm_Stop);
    return true;
}

// -----------------------------------------------------------------
// NOMINAL_LENGTH_FROM_SPEED changes the `step_length` as function
// of the robot speed (Current_Speed).
// E.S.
// -----------------------------------------------------------------

void nominal_length_from_speed()
{
    foot_l = pow(Current_Speed / (sqrt(g * l)), exp_val) * l;   // length definition, see referneces

    foot_l = (foot_l <= l_des_max) ? foot_l : l_des_max;        // check maximum value
    foot_l = (foot_l >= l_des_min) ? foot_l : l_des_min;        // check minimum value
    // ROS_INFO("Change step length: %.2lf", foot_l);
}

// -------------------------------------------------- //
// Stride Parameters Callbacks 						  //
// -------------------------------------------------- //

void update_des_speed_CallBack(const std_msgs::Float32::ConstPtr &msg)
{
    try_current = msg->data;

    // Update Current speed only in case of valid value and if you've completed the previous concatenation
    if ((fabs(speed - try_current) >= Var_speed_threshold) && flag_concatenate_completed)
    {
        Current_Speed = (msg->data <= v_des_max) ? msg->data : v_des_max;
        // ROS_INFO("Change SoftLegs velocity: %.2lf", Current_Speed);
        nominal_length_from_speed();        // update step length
        flag_concatenate_completed = false;
    }

    if ((try_current <= min_v_des) && flag_concatenate_completed)
    {
        Current_Speed = min_v_des;
        // ROS_INFO("Change SoftLegs velocity: %.2lf", Current_Speed);
        nominal_length_from_speed();        // update step length
        Warm_Stop  = true;
        Warm_Start = false;
    }
    return;
}

void update_des_length_CallBack(const std_msgs::Float32::ConstPtr &length)
{
    foot_l = (length->data <= l_des_max) ? length->data : l_des_max;
    // ROS_INFO("Change step length: %.2lf", foot_l);
    flag_concatenate_completed = false;
    change_step_length = true;
    return;
}

void update_des_height_CallBack(const std_msgs::Float32::ConstPtr &height)
{
    foot_h = (height->data <= h_des_max) ? height->data : h_des_max;
    // ROS_INFO("Change step clearance: %.2lf", foot_h);
    flag_concatenate_completed = false;
    change_step_height = true;
    return;
}

// -------------------------------------------------- //

void BiasCallback(const synergies::bias &msg)
{
    for (int i = 0; i < 6; i++)
    {
        bias[i] = msg.bias[i];
        old_bias[i] = bias[i];
    }
    return;
}

void Bias_Stop_Callback(const synergies::bias &msg)
{
    for (int i = 0; i < 6; i++)
    {
        bias_stop[i] = msg.bias[i];
    }
    return;
}

// This function update the bias as function of the speed retrieved by the distance sensor
void update_bias_trunk_ankle(const std_msgs::Float32::ConstPtr &msg)
{
    if ((err_tilde != msg->data) && wanna_use_bias_trunk_ankle)
    {
        std::cout << "Distance update bias on Trunk Ankle" << std::endl;
        err_tilde = msg->data;

        //bias trunk 0 and 3
        bias_err_tilde_T = Trunk_Gain * err_tilde;
        data_corr_AT_msg.Trunk_corr = bias_err_tilde_T;
        bias[0] = old_bias[0] + bias_err_tilde_T;
        bias[3] = old_bias[3] + bias_err_tilde_T;
        if (bias[0] >= max_Trunk)
        {
            bias[0] = max_Trunk;
            data_corr_AT_msg.Max_Trunk = 1;
        }
        if (bias[0] <= min_Trunk)
        {
            bias[0] = min_Trunk;
            data_corr_AT_msg.Min_Trunk = 1;
        }
        if (bias[3] >= max_Trunk)
        {
            bias[3] = max_Trunk;
        }
        if (bias[3] <= min_Trunk)
        {
            bias[3] = min_Trunk;
        }

        //bias ankle 2 and 5
        bias_err_tilde_A = Ankle_Gain * err_tilde;
        data_corr_AT_msg.Ankle_corr = bias_err_tilde_A;
        bias[2] = old_bias[2] + bias_err_tilde_A;
        bias[5] = old_bias[5] + bias_err_tilde_A;

        if (bias[2] >= max_Ankle)
        {
            bias[2] = max_Ankle;
            data_corr_AT_msg.Max_Ankle = 1;
        }
        if (bias[2] <= min_Ankle)
        {
            bias[2] = min_Ankle;
            data_corr_AT_msg.Min_Ankle = 1;
        }
        if (bias[5] >= max_Ankle)
        {
            bias[5] = max_Ankle;
        }
        if (bias[5] <= min_Ankle)
        {
            bias[5] = min_Ankle;
        }

        data_corr_AT_msg.err_tilde = err_tilde;
        pub_data_corr.publish(data_corr_AT_msg);
    }
}

void update_trunk_ankle_gain(const synergies::gain_bias_err &msg)
{
    Trunk_Gain = msg.gain_bias_err[0];
    min_Trunk  = msg.gain_bias_err[1];
    max_Trunk  = msg.gain_bias_err[2];
    Ankle_Gain = msg.gain_bias_err[3];
    min_Ankle  = msg.gain_bias_err[4];
    max_Ankle  = msg.gain_bias_err[5];
    std::cout << " Trunk G " << Trunk_Gain << " min " << min_Trunk << " max " << max_Trunk << " Ankle G " << Ankle_Gain << " min " << min_Ankle << " max " << max_Ankle << std::endl;
}

void update_BT(const std_msgs::Float32 &msg)
{
    BT = msg.data;
    std::cout << " New BT " << BT << std::endl;
}

void update_bias_stop_flag(const std_msgs::Bool::ConstPtr &msg)
{
    bool v = msg->data;
    std::cout << " syn flag Bias Stop " << v << std::endl;

    wanna_use_bias_stop = msg->data;

    return;
}

void update_ankle_trunk_gain_flag(const std_msgs::Bool::ConstPtr &msg)
{
    bool v = msg->data;
    std::cout << " syn flag ankle trunk gain " << v << std::endl;

    wanna_use_bias_trunk_ankle = msg->data;

    return;
}

void callback_activate_quant(const std_msgs::Bool::ConstPtr &msg)
{
    if (msg->data)
    {
        std::cout << " syn Activate quant " << std::endl;
        Var_speed_threshold = 0.005;
    }
    else
    {
        std::cout << " syn Deactivate quant " << std::endl;
        Var_speed_threshold = 0.005;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Synergies_3param");
    ros::NodeHandle n;
    std::cout << std::endl;
    std::cout << "-----CMD-----" << std::endl;
    std::cout << "rostopic pub -1 /synergies_bias synergies/bias '{bias: [4, 0, -9, 4, -10, 5]}'" << std::endl;
    std::cout << "rostopic pub -1 /gain_bias_trunk_ankles synergies/gain_bias_err '{gain_bias_err: [0, -10, 10, 0.1, 12, 19]}'" << std::endl;
    std::cout << "rostopic pub -1 /update_BT  std_msgs/Float32 '{data: 1}'" << std::endl;
    std::cout << "rostopic pub -1 /synergies/bias_stop synergies/bias '{bias: [4, 0, -9, 4, -10, 5]}'" << std::endl;
    std::cout << std::endl;
    std::cout << "rostopic echo /synergies_bias_echo" << std::endl;
    std::cout << std::endl;

    // -------------------------INIT----------------------------

    // Set publisher to publish trajectories into "synergies/Trajectories
    pub_data_corr = n.advertise<synergies::data_corr_AT>("/synergies_bias_AT_correction",  10);
    ros::Publisher pub_traj = n.advertise<synergies::traj_state>("synergies/Trajectories", 10);
    ros::Publisher pub_bias_echo = n.advertise<synergies::bias>("/synergies_bias_echo",    10);
    //ros::Publisher pub_speed = n.advertise<std_msgs::Float64>("/robot_speed", 10);

    // subscribe to STRIDE PARAMETERS
    ros::Subscriber sub_speed  = n.subscribe("/synergies_des_speed",       5, &update_des_speed_CallBack);
    ros::Subscriber sub_length = n.subscribe("/synergies_des_length",      5, &update_des_length_CallBack);
    ros::Subscriber sub_height = n.subscribe("/synergies_des_foot_height", 5, &update_des_height_CallBack);

    // subscribe to something  \(°-°)/
    ros::Subscriber sub_bias      = n.subscribe("/synergies_bias", 1, &BiasCallback);
    ros::Subscriber sub_bias_stop = n.subscribe("/synergies_bias_stop", 1, &Bias_Stop_Callback);
    ros::Subscriber sub_err_bias  = n.subscribe("/bias_trunk_ankles", 1, &update_bias_trunk_ankle);
    ros::Subscriber sub_gain_err_bias = n.subscribe("/gain_bias_trunk_ankles", 1, &update_trunk_ankle_gain);
    ros::Subscriber sub_gain_BT   = n.subscribe("/update_BT", 1, &update_BT);
    ros::Subscriber sub_stop_flag = n.subscribe("/enable_bias_stop", 1, &update_bias_stop_flag);
    ros::Subscriber sub_ankle_trunk_gain_flag = n.subscribe("/enable_gain_ankle_trunk", 1, &update_ankle_trunk_gain_flag);
    ros::Subscriber sub_min_max_v = n.subscribe("/update_min_max_v", 10, &callback_update_min_max_v);
    ros::Subscriber sub_activate_quant = n.subscribe("/activate_quant", 10, &callback_activate_quant);

    // Create service to enable speed change by user cmd and to do a warm start and stop
    ros::ServiceServer srv_setSpeed   = n.advertiseService("synergies/Set_Speed", &setSpeedCallback);
    ros::ServiceServer srv_warm_start = n.advertiseService("synergies/Warm_Start", &setWarmStartCallback);
    ros::ServiceServer srv_warm_stop  = n.advertiseService("synergies/Warm_Stop", &setWarmStopCallback);

    initialize_synergies();

    Eigen::MatrixXd New_Traj, Old_Traj, New_Traj_supp;

    bool first_time = true;

    synergies::traj_state msg; // synergies::traj_state --> float64[] joint_pos

    int index = 0;

    std::vector<double> traj_d = {0, 0, 0, 0, 0, 0};

    bool flag_concatenate_started = false;

    double T_old, Ts_old, pub_Ts, curr_Ts;
    double T_new, Ts_new;

    double traj_buffer_size;
    Eigen::VectorXd res_traj;
    std::vector<double> res_traj_2(60);
    int Instant(0);

    double qb_val(0);
    int pm = 1;
    int n_count_qb(0);

    // pub_Ts = 0.006;  // old publication freuency
    pub_Ts = 0.02;
    ros::Rate rate_while = 1 / pub_Ts;

    int new_num, old_num, min_num, pub_num;

    bool one_time = true;

    tk::spline new_s;
    tk::spline old_s;
    tk::spline trans_s; //transition

    Eigen::MatrixXd New_Traj_pub;
    Eigen::MatrixXd Old_Traj_pub;
    Eigen::MatrixXd Middle_Traj_pub;
    Eigen::MatrixXd Output_Traj;
    Eigen::MatrixXd Err_Traj;

    //------------------------INIT Warm Start & Stop--------------

    double Ni_start = std::round(200 * 6 * 0.1);  //200 steps with Ts_pub, i.e. 1.2 seconds , *6 due to the code position see below
    double Ni_stop  = std::round(200 * 6 * 0.5);  // was 1 //200 steps with Ts_pub, i.e. 1.2 seconds , *6 due to the code position see below      // was 1 //200 steps with Ts_pub, i.e. 1.2 seconds , *6 due to the code position see below
    double index_w_start = 1;
    double index_w_stop  = 1;

    //------------------------INIT QB----------------------------

    ros::Publisher qb_legs_pub;
    qb_legs_pub = n.advertise<geometry_msgs::Pose>("/qb_legs_trajectories", 10);
    std::vector<int> ids = {1, 2, 3, 4, 5, 6};

    //------------------------END INIT----------------------------
    // if you go below the min speed your trajectories are set to 0. If you just decrease the
    // min speed to zero due to the stride law the step time increases too much. Hence if you reach
    // min speed trajectories goes to zero then you have to reastart with a warm start

    while (ros::ok())
    {
        if (Current_Speed <= min_speed)
        {
            Current_Speed = min_speed;
            Warm_Stop = true;
            Warm_Start = false;
        }
        else
        {
            Warm_Stop = false;
            Warm_Start = true;
        }

        //--------------------------TRAJ---------------------------

        // Call Mapping function
        if (first_time)
        {
            // speed is the correct desired speed

            if (Current_Speed <= min_v_des)
            {
                Current_Speed = min_v_des;
            }

            speed = Current_Speed;

            T_new  = foot_l / Current_Speed;
            Ts_new = T_new  / 30; // 30 is the sample number of one step (!)

            // Current Sample Time
            curr_Ts = Ts_new;
            // New data dimensions as function of speed and Ts
            new_num = std::round(T_new / pub_Ts) * 2; //because we have two steps in the analyzed trajectories (!)
            old_num = new_num;
            New_Traj = From_Syn_To_Traj(speed, foot_h, foot_l, PCs, Mus);
            Old_Traj = New_Traj;

            first_time = false;
            std::cout << "Activation speed " << std::to_string(Current_Speed) << std::endl;

            New_Traj_pub = Interpolate_and_Resample(New_Traj, Ts_new, pub_Ts);
            Old_Traj_pub = New_Traj_pub;
        }
        else // not first time
        {
            // if it is not the first time and speed has not changed nothing have to be done
            // Otherwise if speed is modified we have to:
            // 1 - Create two batches the first composed of the old and new signals the second just of the new ones
            // 2 - If it is too late continue to publish the old one, then publish the mixed batch and hence the new one.
            // the second step could take awhile hence if concatenate operation is started you do not have to modify stuff
            if ((speed != Current_Speed || change_step_length || change_step_length) && one_time == true)
            {
                // a change occurs hence you need to concatenate
                T_old = T_new;   //This is the time interval within one step has to be done
                Ts_old = Ts_new; // 30 is the number of samples of one step

                T_new = foot_l / Current_Speed;
                Ts_new = T_new / 30; // 30 is the sample number of one step

                Old_Traj = New_Traj;
                New_Traj = From_Syn_To_Traj(speed, foot_h, foot_l, PCs, Mus);

                // 	Prepare the signal batches
                New_Traj_pub = Interpolate_and_Resample(New_Traj, Ts_new, pub_Ts);
                Old_Traj_pub = Interpolate_and_Resample(Old_Traj, Ts_old, pub_Ts);

                //  Find when you have to move from old to new signals
                Eigen::MatrixXf::Index i_g, j_g;

                double val_1;
                double sum_val(0);
                std::vector<double> err(6);
                Eigen::VectorXd ERR(60);
                for (int j = 0; j < 60; j++)
                {
                    sum_val = 0;
                    for (int i = 0; i < 6; i++)
                    {
                        val_1 = (Old_Traj(j, i) - New_Traj(j, i)); // This is the value of the error
                        sum_val += fabs(val_1);
                    }
                    ERR(j) = sum_val;
                }

                val_1 = ERR.minCoeff(&i_g, &j_g); // This is the value of the error

                Instant = i_g; // This is the instant at which you should switch the signals

                if (Instant == 0) Instant = 1;

                interp_Instant = std::round(Instant * Ts_old / pub_Ts);

                // Create the mixed batch
                Middle_Traj_pub = Interpolate_and_Resample_Middle(New_Traj, Ts_new, Old_Traj, Ts_old, Instant, pub_Ts);

                // 	Now all the signals have been created hence i have not to enter in this statement again
                flag_concatenate_started   = true;
                flag_concatenate_completed = false;
            } //end if speed change occurs

        } //end if else first time and speed change occurs

        interp_Instant = std::round(Instant * Ts_old / pub_Ts);

            // if (speed == Current_Speed)
            if (speed == Current_Speed && !change_step_length && !change_step_height)
            {
                Output_Traj = Old_Traj_pub;
                pub_num = Old_Traj_pub.rows();
            }
            else // there has been a change 
            { 
                one_time = false; //new speed variations are disabled

                if (flag_concatenate_started)
                {
                    pub_num = Middle_Traj_pub.rows();
                    Output_Traj = Middle_Traj_pub;
                    flag_concatenate_started = false;
                    flag_concatenate_completed = false;
                }
            }

            if (!flag_concatenate_started && !flag_concatenate_completed)
            {              
                change_step_length = false;
                change_step_height = false;

                index = 0;
                Output_Traj = New_Traj_pub;
                Old_Traj_pub = Output_Traj;

                flag_concatenate_completed = true;
                pub_num = New_Traj_pub.rows();

                speed = Current_Speed;
                one_time = true;
            }

            if (index == pub_num)
                index = 0;

            for (int i = 0; i < 6; i++)
            {
                traj_d[i] = Output_Traj(index, i);
                
                // ------------------ Warm Start & Warm Stop --------------

                if (Warm_Start && !Warm_Stop)
                {
                    if (index_w_start >= Ni_start)
                    {
                        index_w_start = Ni_start;
                        index_w_stop  = 1;
                    }
                    else
                    {
                        index_w_start++;
                    }

                    traj_d[i] = traj_d[i] * index_w_start / Ni_start;

                }
                else if (!Warm_Start && Warm_Stop)
                {
                    if (index_w_stop >= Ni_stop)
                    {
                        index_w_stop  = Ni_stop;
                        index_w_start = 1;
                    }
                    else
                    {
                        index_w_stop++;
                    }

                    traj_d[i] = traj_d[i] * (1 - index_w_stop / Ni_stop);
                }
                else if ((!Warm_Start && !Warm_Stop) || (Warm_Start && Warm_Stop))
                {
                    Warm_Start    = false;
                    Warm_Stop     = true;
                    index_w_stop  = 1;
                    index_w_start = 1;
                }

            } //end for i

            //---------------------------------------PUB----------------------------------

            msg.joint_pos = traj_d;
            pub_traj.publish(msg);

            //---------------------------------------PUB QB LEGS--------------------------
            index_time += pub_Ts;

            synergies::bias bias_msg_echo;

            for (int i = 0; i < 6; i++)
            {
                bias_msg_echo.bias[i] = bias[i];
            }
            pub_bias_echo.publish(bias_msg_echo);

            if (flag_pub_qbLegs_robot)
            {
                geometry_msgs::Pose P_msg;
                for (auto id : ids)
                {

                    if (Current_Speed <= min_speed && wanna_use_bias_stop)
                    {
                        qb_val = traj_d[id - 1] + (bias_stop[id - 1]) * pi / 180;
                    }
                    else
                    {
                        qb_val = traj_d[id - 1] + (bias[id - 1]) * pi / 180;
                    }

                    if (id == 1 || id == 2 || id == 3)  // left leg
                    {
                        qb_val *= -1;                   // the motors have opposite orientation
                    }

                    // joint saturation 90°
                    qb_val = (qb_val <=   pi / 2) ? qb_val :   pi / 2;  // check max value
                    qb_val = (qb_val >= - pi / 2) ? qb_val : - pi / 2;  // check min value

                    if (id == 1)
                    {
                        P_msg.position.x = qb_val;
                    }
                    if (id == 2)
                    {
                        P_msg.position.y = qb_val;
                    }
                    if (id == 3)
                    {
                        P_msg.position.z = qb_val;
                    }
                    if (id == 4)
                    {
                        P_msg.orientation.x = qb_val;
                    }
                    if (id == 5)
                    {
                        P_msg.orientation.y = qb_val;
                    }
                    if (id == 6)
                    {
                        P_msg.orientation.z = qb_val;
                    }

                    bias_msg_echo.bias[id - 1] = bias[id - 1];
                } // end for pub
                qb_legs_pub.publish(P_msg);
            } //end if flag

        index += 1;

        ros::spinOnce();
        rate_while.sleep();
    } //end while

    return 0;
}
