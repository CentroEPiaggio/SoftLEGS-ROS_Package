// This code provides the reference speed as function of the error of the current robot position w.r.t. the desired one.
// The code can be divided in three main parts
// 1 - Subscribe to the sensor node which rate is 1000Hz
// 2 - Implement a filter at 100Hz considering the work range of the sensor,
//     i.e. if the robot remains within some boundaries the measurement makes sense, otherwise is dangerous
// 3 - Implement a second filter which window size depends on the speed of the system and the step time.
// 4 - Calculate the desired speed and publish it.


#include <ros/ros.h>
#include <ros/rate.h>
#include <eigen3/Eigen/Eigen>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <math.h>
#include <phidgets_interface_kit/AnalogArray.h>
#include <imu_management/filt.h>
#include <imu_management/imu_data.h>
#include <qb_interface/inertialSensorArray.h>

// Filter define
#define FILT_T      LPF     // type of filter: LPF, HPF, BPF
#define N_TAPS_D    21      // number of taps the filter is going to use 
#define FS_D        0.4     // [kHz] sensor frequency
#define FX_D        0.010   // [kHz] upper transition frequencies (for HPF filters)
#define N_TAPS_V    11      // number of taps the filter is going to use  
#define FS_V        0.4     // [kHz] sensor frequency
#define FX_V        0.002   // [kHz] upper transition frequencies (for HPF filters)

// Initial value
std_msgs::Float32 dist_old, dist;  
std_msgs::Float32 dist_filt, dist_raw;
std_msgs::Float32 vel, vel_filt, dist_mean;
double rateHZ = 400;      // frequenza nodo
double pitch;             // pitch value to compensate the vel estimate
double omega, omega_off;
bool   one_time; 

// Filter variables
Filter *filt_vel;
Filter *filt_dist;

Eigen::VectorXd array_sensor_pre_filter;
Eigen::VectorXd array_dist_estimate;

//------------------CALLBACKS---------------------

//Sensor measurement callback
void callback_sensor_IR(const phidgets_interface_kit::AnalogArray::ConstPtr& msg)
{
  // consider as measure the mean of the last 30 samples
  array_sensor_pre_filter << array_sensor_pre_filter.tail(array_sensor_pre_filter.size() - 1), msg->values[2];
  dist_raw.data = array_sensor_pre_filter.mean();
}

void callback_sensor_pitch(const imu_management::imu_data::ConstPtr& data)
{
  pitch = data->pitch * 3.1415 / 180;
}

void callback_sensor_omega(const qb_interface::inertialSensorArray &om)
{
  if (one_time) {
    omega_off = om.m[0].y;
    one_time = false;
  } 
  omega = om.m[0].y - omega_off;
}

//-------------------------MAIN---------------------

int main(int argc, char **argv)
{
  float alpha = 0.60;  // variable for convex combination of two velocity estimation

  one_time = true;
  // array for mean computation of distance measurement
  array_sensor_pre_filter = Eigen::VectorXd::Zero(50);
  array_dist_estimate     = Eigen::VectorXd::Zero(20);

  ros::init(argc, argv, "compute_vel");
  ros::NodeHandle n_;

  ros::Rate r(rateHZ);

  //Topic you want to pub
  ros::Publisher  pub_dist_no_filt = n_.advertise<std_msgs::Float32>("no_filt_dist", 100);
  ros::Publisher  pub_dist_filt    = n_.advertise<std_msgs::Float32>("current_dist", 100);
  ros::Publisher  pub_vel_filt     = n_.advertise<std_msgs::Float32>("current_vel", 100);
  ros::Publisher  pub_dist_mean    = n_.advertise<std_msgs::Float32>("dist_mean", 100);

  //Topics to subscribe to: IR sensor for distance, imu_data for pitch measurement and gyro for angular velocity
  ros::Subscriber sub_sensor_IR, sub_sensor_pitch, sub_sensor_omega;
  sub_sensor_IR    = n_.subscribe("/cl4_gpio/analog_in", 10, &callback_sensor_IR);
  sub_sensor_pitch = n_.subscribe("/imu_data",           10, &callback_sensor_pitch);
  sub_sensor_omega = n_.subscribe("/qb_class_imu/gyro",  10, &callback_sensor_omega);

  // filter instance
  filt_dist = new Filter(FILT_T, N_TAPS_D, FS_D, FX_D);
  filt_vel  = new Filter(FILT_T, N_TAPS_V, FS_V, FX_V);

  // ROS return if there are errors on the filtering
  ROS_INFO("FILTER ERROR flag = %d <---", filt_dist->get_error_flag());  // see filt.h for the error value meaning
  if(filt_dist->get_error_flag() != 0 ) exit(1);
  ROS_INFO("FILTER ERROR flag = %d <---", filt_vel->get_error_flag());   // see filt.h for the error value meaning
  if(filt_vel->get_error_flag() != 0 ) exit(1);

  std_msgs::Float32 dist_no_corr;
  double vel_tmp;

  while(ros::ok())
  {
    // Convert sensor data in centimeters [cm], from datasheet
    dist.data = fabs(9462.0 / (dist_raw.data - 16.92)) * cos(pitch);  // with pitch compensation
    dist_no_corr.data = fabs(9462.0 / (dist_raw.data - 16.92));       // without pitch compensation
    // Filtering distance
    dist_filt.data = filt_dist->do_sample(dist.data); // [cm] 
    // Saturate the distance according to what did in Giamma code (maybe this is due to the capability of the IR sensor)
    if (dist_filt.data <= 20)  dist_filt.data = 20;
    if (dist_filt.data >= 100) dist_filt.data = 100;

    array_dist_estimate << array_dist_estimate.tail(array_dist_estimate.size() - 1), dist_filt.data;
    dist_mean.data = array_dist_estimate.mean();
    
    // Compute velocity
    vel.data = rateHZ * (dist_mean.data - dist_old.data); // [cm/sec]
    vel_tmp = filt_vel->do_sample(vel.data);

    vel_filt.data = (1 - alpha) * (- vel_tmp/100) + (alpha) * (- omega * 0.30 * 3.1415 / 180) ; // convert to m/sec

    // Saturate velocity
    //if (fabs(vel_filt.data) <= 0.05)  vel_filt.data = 0.0;
    if (vel_filt.data >= 0.3) vel_filt.data = +0.3;
    if (vel_filt.data <= -0.3) vel_filt.data = -0.3;

    //------------------------------------PUBLISH ACTUAL DISTANCE & SPEED-----------------------
    pub_dist_no_filt.publish(dist);
    pub_dist_filt.publish(dist_no_corr);
    pub_dist_mean.publish(dist_mean);
	  pub_vel_filt.publish(vel_filt);

    ros::spinOnce();
    dist_old.data = dist_mean.data;
    r.sleep();
        
  }// end while()

delete filt_vel;
delete filt_dist;
  
return 0;
}
