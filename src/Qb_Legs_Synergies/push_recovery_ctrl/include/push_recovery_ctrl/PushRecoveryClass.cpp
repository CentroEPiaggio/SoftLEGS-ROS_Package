// ------------------------------------------------------------ //
// This file defines the functions of a class that implements the push recovery
// controller for the biped SoftLegs. 
// Where possible the function names and variable names are kept the
// same as the `push recovery controller` defined in MATLAB.
//
// Edoardo Sorelli: edoardo.sorelli@gmail.com
// ------------------------------------------------------------ //

#include "push_recovery_ctrl/PushRecoveryClass.h"

/// -------------------- CONSTRUCTOR -------------------- ///

PR_ctrl::PR_ctrl() :
    active_ctrl(false),
    // desired values: default
    v_des  (V_INIT),
    L_des  (L_INIT),
    fh_des (FH_INIT),
    // max value of the parameters
    v_max  (V_MAX),
    L_max  (L_MAX),
    fh_max (FH_MAX),
    // walking state
    single_phase (false),
    recovery (false),
    // controlled values
    v_set  (V_INIT),
    L_set  (L_INIT),
    fh_set (FH_INIT),
    // allow at first the publishing
    new_control (0)
{
    // set up filter for right foot: value in [kHz]
    fz1_filter = new Filter(LPF, 11, 0.050, 0.005);                         // see filt.h for a better explaination 
    ROS_INFO("FILTER ERROR flag = %d <---", fz1_filter->get_error_flag());  // see filt.h for the error value meaning
    if(fz1_filter->get_error_flag() != 0 ) exit(1);                         // abort in an appropriate manner
    // set up filter for left foot: value in [kHz]
    fz2_filter = new Filter(LPF, 11, 0.050, 0.005);                         // see filt.h for a better explaination 
    ROS_INFO("FILTER ERROR flag = %d <---", fz2_filter->get_error_flag());  // see filt.h for the error value meaning
    if(fz2_filter->get_error_flag() != 0 ) exit(1);                         // abort in an appropriate manner
}

/// -------------------- CALLBACKS -------------------- ///

void PR_ctrl::on_off_Callback(const std_msgs::Bool &on_off)
{
    // if `true` the controller works, id `false` the controller output is NULL
    active_ctrl = on_off.data;
    ROS_INFO("Push Recovery Controller STATUS: %d (1 active, 0 not active)", active_ctrl);
}

void PR_ctrl::desired_velocity_Callback(const std_msgs::Float32 &des_vel)
{
    // update desired velocity ad step length
    v_des = des_vel.data;
    L_des = nominal_length(v_des);
}

void PR_ctrl::imu_Callback(const imu_management::imu_data &imu)
{
    // update pelvis position and orientation
    // p_vel   = imu.vel; // the velocity is obtained from the distance sensor
    p_pitch = imu.pitch * DEG2RAD; // convert pitch from degree to radians
}

void PR_ctrl::dist_sensor_Callback(const std_msgs::Float32 &vel)
{
    p_vel = vel.data; 
}

void PR_ctrl::right_wrenches_Callback(const std_msgs::Float64MultiArray &right_wrench)
{
    filt_fz1.data = - fz1_filter->do_sample(right_wrench.data[2]);

    // TRUE if force detected
    fz1 = filt_fz1.data > FZ_DETECT;

    check_robot_phase();
}

void PR_ctrl::left_wrenches_Callback(const std_msgs::Float64MultiArray &left_wrench)
{
    filt_fz2.data = - fz2_filter->do_sample(left_wrench.data[2]);

    // TRUE if force detected
    fz2 = filt_fz2.data > FZ_DETECT;    

    check_robot_phase();
}

void PR_ctrl::robot_pose_Callback(const geometry_msgs::Pose &pose)
{
    // update the robot pose
    // left leg
    robot_pose[0] = - pose.position.x; 
	robot_pose[1] = pose.position.y; 
	robot_pose[2] = pose.position.z; 
    // right leg
	robot_pose[3] = pose.orientation.x; 
	robot_pose[4] = pose.orientation.y;
	robot_pose[5] = pose.orientation.z;
}

/// -------------------- EVALUATE OUTPUT -------------------- ///

// ----------------------------------------------------------------- //
// EVALUATE_OUTPUT() 
// ----------------------------------------------------------------- //
void PR_ctrl::evaluate_output()
{
    // evaluate always the controller
    evaluate_controller();

    // the debug values report always the control values
    v_debug.data  = static_cast<float>(v_set);
    L_debug.data  = static_cast<float>(L_set);
    fh_debug.data = static_cast<float>(fh_set);

    // the control values are NULL if the controller has been deactivated
    if(!active_ctrl) {
        v_ctrl.data   = 0;
        L_ctrl.data   = L_des;
        fh_ctrl.data  = fh_des;
    } else {
        v_ctrl.data   = v_debug.data;
        L_ctrl.data   = L_debug.data;
        fh_ctrl.data  = fh_debug.data;
    }
}

// ----------------------------------------------------------------- //
// EVALUATE_CONTROLLER() computes the push recovey control
// action
// ----------------------------------------------------------------- //
void PR_ctrl::evaluate_controller()
{
    // evaluate ICP and STEP_LENGTH to evaluate the stability bound
    double icp, step_length;
    std::tie(icp, step_length) = evaluate_stability_param();
    double v_old, L_old, fh_old;

    v_old  = v_set;
    L_old  = L_set;
    fh_old = fh_set;

    // if NOT in recovery set the controlled value as the desired
    // v_set  = (!recovery)  ? v_des  : v_set;
    // L_set  = (!recovery)  ? L_des  : L_set;
    // fh_set = (!recovery)  ? fh_des : fh_set;

    // Analyse the robot state, if unstable activate push recovery controller
    if (single_phase) 
    {
        single_phase_controller(icp, step_length); 
    } 
    else // double phase
    {
        double_phase_controller(icp, step_length);
    }

    // check if it has been evaluated a new control action
    new_control = (v_old != v_set || L_old != L_set || fh_old != fh_set) ? 0 : new_control;
    new_control++;
}

// -------------------- PUSH RECOVERY CONTROLLERS -------------------- //

// ----------------------------------------------------------------- //
// SINGLE_PHASE_CONTROLLER() push recovery
// ----------------------------------------------------------------- //
void PR_ctrl::single_phase_controller(double icp, double step_length)
{
    // define stability bounds
    bool single_phase_unstable = (icp > L_set + HALF_FOOT && icp > step_length);
    // bool inverse_walking       = (icp < - HALF_FOOT);

    // controller single phase
    if (single_phase_unstable)  // instability detected     
    { 
        ROS_INFO("single phase unstable");
        ROS_INFO("ICP: %lf > %lf + %f", icp, L_set, HALF_FOOT);
        ROS_INFO("&&");
        ROS_INFO("ICP: %lf > %lf", icp, step_length); 

        recovery = true;            // check the flag for instability
        
        // L_set  = icp + HALF_FOOT;   // try to recover extending the step length for a default value
        L_set  = nominal_length(v_set) + HALF_FOOT;
        check_L_max();
        fh_set = 0.05;
    }
    // else if (inverse_walking)   // the robot is falling backward
    // {  
        // ROS_INFO("single phase unstable, inverse walking");
        // ROS_INFO("ICP: %lf < -%f ", icp, HALF_FOOT);

        // v_set  = v_set;
        // L_set  = L_des;
        // fh_set = 0.03;
    // }
}

// ----------------------------------------------------------------- //
// DOUBLE_PHASE_CONTROLLER() push recovery
// ----------------------------------------------------------------- //
void PR_ctrl::double_phase_controller(double icp, double step_length)
{
    // define stability bound (for now the robot cannot walk backward)
    bool double_phase_unstable = (icp > HALF_FOOT && !recovery); 
    bool return2stability      = (icp <= HALF_FOOT && !recovery) && (v_set != v_des || L_set != L_des || fh_set != fh_des);

    // controller double phase
    if (double_phase_unstable) // new instability detected
    {
        ROS_INFO("double phase unstable");
        ROS_INFO("ICP: %lf > %f && not recovery", icp, HALF_FOOT);

        recovery = true;

        // v_set  = (v_des == 0) ? 0.2 : v_set + 0.1;
        // L_set  = L_des + HALF_FOOT;
        // fh_set = fh_des + std::max(std::abs(p_pitch),0.1);
        v_set  = (v_des == 0) ? 0.1 : v_set + 0.05;
        L_set  = nominal_length(v_set);
        fh_set = 0.05;

        check_v_max();
        check_L_max();
        check_fh_max();
    }
    else if (return2stability) // the robot is stable, it has to return to the desired values
    {
        ROS_INFO("double phase return to stability");
        ROS_INFO("ICP: %lf", icp);

        recovery = true;

        // v_set  =  (v_set + v_des)  / 2;
        // L_set  =  (L_set + L_des)  / 2;
        // fh_set = (fh_set + fh_des) / 2;
        v_set  -= 0.05;
        L_set  -= HALF_FOOT;
        fh_set -= 0.01;

        v_set  = (v_set < v_des + 0.04)      ? v_des  : v_set;
        L_set  = (L_set < L_des + HALF_FOOT) ? L_des  : L_set;
        fh_set = (fh_set < fh_des + 0.009)   ? fh_des : fh_set;

        // if all the parameters are equal to the desired one set RECOVERY = FALSE
        recovery = !(v_set == v_des && L_set == L_des && fh_set == fh_des);
    }
}

/// --------------- SECONDARY PRIVATE FUNCTIONS --------------- ///

// ----------------------------------------------------------------- //
// NOMINAL_LENGTH() evaluates the `step length` in function of 
// the `forward velocity`
// ----------------------------------------------------------------- //
double PR_ctrl::nominal_length(double v)
{
    float g = G;          // gravitational acceleration
    float l = LEG_LENGTH; // robot leg length
    float b = BETA;       // exp coefficient

    double v_norm = v / sqrt(g*l);   // normalized velocity
    double s = pow(v_norm,b);            // normalized step length
    double length = static_cast<double>(s*l); 

    // minimum value
    length = (length <= 0.01) ? 0.01 : length;

    return length;
}

// ----------------------------------------------------------------- //
// CHECK_SINGLE_PHASE() evaluate if the robot is in single
// or double phase and if transit from one to another
// ---------------------------------------------------------------- //
void PR_ctrl::check_robot_phase()
{
    bool old_single_phase = single_phase;
    
    // single_phase is defined as NOT double phase
    single_phase = !(fz1 && fz2); 

    robot_phase.data = single_phase; 
    
    // check if there is a transition from single to double phase
    bool transition_s2d = (old_single_phase && !single_phase); 

    // ROS_INFO("----> transition from single to double phase: %d <-----", transition_s2d);

    // if in transition from single to double reset the recovery variable to allow new control actions
    recovery = (transition_s2d) ? false : recovery;

    if (old_single_phase != single_phase) {
        if (single_phase){
            ROS_INFO("single phase");
        } 
        else {
            ROS_INFO("double phase");
        }
    }
}

// ----------------------------------------------------------------- //
// EVALUATE_STABILITY_PARAM() compute the ICP and STEP_LENGTH, 
// that are used as stability criteria for the robot
// ----------------------------------------------------------------- //
std::tuple<double, double> PR_ctrl::evaluate_stability_param()
{
    double omega = sqrt(G / COM_Z); // pendudum natural frequency
    double x_ankle1, x_ankle2;      // x position of the ankles
    double ref, icp;                // reference ankle and ICP values
    double step_length;             // distance between ankles

    // evaluate ankles X position
    std::tie(x_ankle1, x_ankle2) = ankles_x_position();

    // evaluate ICP
    ref = (x_ankle1 >= x_ankle2) ? x_ankle1 : x_ankle2;
    // icp = -(ref - 0.054) + (p_vel / omega);  // definintion of ICP, the 0.054 value comes from testing
    icp = -(ref) + (p_vel / omega);  // definintion of ICP, the 0.054 value comes from testing

    // store REF and ICP value for publishing and DEBUG
    REF.data = ref; // 0.054 is an offset of the ref with respect the standing position
    ICP.data = icp;

    // evaluate step length
    step_length = std::abs(x_ankle1) + std::abs(x_ankle2);

    return std::make_tuple(icp, step_length); 
}

// ----------------------------------------------------------------- //
// ANKLES_X_POSITION() returns the X position of the ankle w.r.t.
// a reference system centered in the IMU/pelvis with fixed orientation
// equal to the global reference system
// ----------------------------------------------------------------- //
std::tuple<double, double> PR_ctrl::ankles_x_position() 
{
    // REMARK: this function implement a kind of DIRECT KINEMATIC,
    // so if you want a function that evaluates the direct kinematic
    // modify this one!

    double lP = LEN_PELVIS;
    double lF = LEN_FEMUR;
    double lT = LEN_TIBIA;

    double q0 = - p_pitch;     // orientation pelvis
    double q1 = robot_pose[0]; // orientation right hip
    double q2 = robot_pose[1]; // orientation right knee
    double q4 = robot_pose[3]; // orientation left hip
    double q5 = robot_pose[4]; // orientation left knee
 
    // hip joint position 
    std::vector<double> p0 {lP * sin(q0), -lP * cos(q0)};
 
    // joint relative position
    std::vector<double> p01 {lF * sin(q0 + q1),      -lF * cos(q0 + q1)};       // position right knee
    std::vector<double> p12 {lT * sin(q0 + q1 + q2), -lT * cos(q0 + q1 + q2)};  // position right ankle
    std::vector<double> p04 {lF * sin(q0 + q4),      -lF * cos(q0 + q4)};       // position left  knee
    std::vector<double> p45 {lT * sin(q0 + q4 + q5), -lT * cos(q0 + q4 + q5)};  // position left  ankle

    // joint absolute position 
    std::vector<double> p1 = vector_sum(p0, p01);   // position right knee
    std::vector<double> p2 = vector_sum(p1, p12);   // position right ankle
    std::vector<double> p4 = vector_sum(p0, p04);   // position left  knee
    std::vector<double> p5 = vector_sum(p4, p45);   // position left  ankle  

    // return the X values
    return std::make_tuple(p2.at(0), p5.at(0));
}

// ----------------------------------------------------------------- //
// VECTOR_SUM() evaluates a custom element to element sum between 
// two vector of dimension two 
// ----------------------------------------------------------------- //
std::vector<double> PR_ctrl::vector_sum(std::vector<double> v1, std::vector<double> v2)
{
    std::vector<double> result;
    result.reserve(v1.size());

    std::transform(v1.begin(), v1.end(), v2.begin(), 
                   std::back_inserter(result), std::plus<double>());
    return result;
}


// ----------------------------------------------------------------- //
// CHECK_V_MAX() saturates v_set to v_max if v_set > v_max
// ----------------------------------------------------------------- //
void PR_ctrl::check_v_max()
{
    v_set = (v_set > v_max) ? v_max : v_set;
}

// ----------------------------------------------------------------- //
// CHECK_L_MAX() saturates L_set to L_max if L_set > L_max
// ----------------------------------------------------------------- //
void PR_ctrl::check_L_max()
{
    L_set = (L_set > L_max) ? L_max : L_set;
}

// ----------------------------------------------------------------- //
// CHECK_FH_MAX() saturates fh_set to fh_max if fh_set > fh_max
// ----------------------------------------------------------------- //
void PR_ctrl::check_fh_max()
{
    fh_set = (fh_set > fh_max) ? fh_max : fh_set;
}