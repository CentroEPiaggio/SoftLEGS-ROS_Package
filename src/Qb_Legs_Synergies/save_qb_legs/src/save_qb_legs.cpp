// This code simply spawn a set of subscriber which retrieve the robot signals and print them on a specific file

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
#include <geometry_msgs/Pose.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <map>

#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <control_msgs/JointTrajectoryControllerState.h>

#include <synergies/bias.h>

#include <iostream>
#include <fstream>
#include <ctime>
#include <dirent.h>
#include <synergies/traj_state.h>
#include <save_qb_legs/data_msg.h>

bool FLAG_OPT;

double des_speed(0);
double Current_Speed(0);
std::vector<double> pos(6);
std::vector<double> traj(6);
std::vector<double> wrench_L(6);
std::vector<double> wrench_R(6);
std::vector<double> bias(6);
int id_p(0);
double dist(0);
double num_sim(0);
const double pi=3.1415926535;
// std_msgs::Float64MultiArray curr_msg;
std::vector<double> curr_meas(12);
std::vector<double> volt_meas(6);

std::ofstream myfile_WL;
std::ofstream myfile_WR;
std::ofstream myfile_pv;
std::ofstream myfile_traj;
std::ofstream myfile_pos;
std::ofstream myfile_curr;
std::ofstream myfile_volt;

ros::Time begin;
ros::Duration elapsed_time;

void write_des_speed(const std_msgs::Float32& msg){
  Current_Speed=msg.data;
}

// void write_dist(const std_msgs::Float32& msg){
//   dist=msg.data;
// }

void write_dist_speed(const save_qb_legs::data_msg& msg){
  des_speed=msg.des_speed;
  dist=msg.distance;
}

void write_filtered_dist(const std_msgs::Float32& msg){
//   des_speed=msg.des_speed;
  dist=msg.data;
}

void opt_write_sim(const std_msgs::Float64& msg){

  num_sim=msg.data;
}

void zmp_write_sim(const std_msgs::Float64& msg){

  num_sim=msg.data;
}

// void write_cube(const control_msgs::JointTrajectoryControllerState& msg){
//   pos[id_p]=msg.actual.positions[0];
// }

void write_traj_meas(const geometry_msgs::Pose& msg){
  traj[0]=msg.position.x;
  traj[1]=msg.position.y;
  traj[2]=msg.position.z;
  traj[3]=msg.orientation.x;
  traj[4]=msg.orientation.y;
  traj[5]=msg.orientation.z;
  
}

void opt_write_traj_meas(const geometry_msgs::Pose& msg){
  traj[0]=msg.position.x;
  traj[1]=msg.position.y;
  traj[2]=msg.position.z;
  traj[3]=msg.orientation.x;
  traj[4]=msg.orientation.y;
  traj[5]=msg.orientation.z;
  
}

void write_left_wrench(const std_msgs::Float64MultiArray& msg){

  for (int i = 0; i < 6; i++)
  {
      wrench_L[i]= msg.data[i];
  } 
}

void write_right_wrench(const std_msgs::Float64MultiArray& msg){
  for (int i = 0; i < 6; i++)
  {
      wrench_R[i]= msg.data[i];
  } 
}

void write_bias(const synergies::bias& msg){
  for(int i=0;i<6;i++){
  bias[i]=msg.bias[i];
  }  
}

void opt_write_bias(const std_msgs::Float64MultiArray::ConstPtr& msg){
    for(int i=0;i<6;i++){
  bias[i]=msg->data.at(i);
  }  
}

void zmp_write_bias(const std_msgs::Float64MultiArray::ConstPtr& msg){
    for(int i=0;i<6;i++){
  bias[i]=msg->data.at(i);
  }  
}

void write_eq_pos(const geometry_msgs::Pose& msg){
  pos[0]=msg.position.x;
  pos[1]=msg.position.y;
  pos[2]=msg.position.z;
  pos[3]=msg.orientation.x;
  pos[4]=msg.orientation.y;
  pos[5]=msg.orientation.z;
}

void opt_write_eq_pos(const geometry_msgs::Pose& msg){
  pos[0]=msg.position.x;
  pos[1]=msg.position.y;
  pos[2]=msg.position.z;
  pos[3]=msg.orientation.x;
  pos[4]=msg.orientation.y;
  pos[5]=msg.orientation.z;
}

void write_file_left_wrench(){
  myfile_WL<<elapsed_time<<"\t";
    for (int i = 0; i < 6; i++)
  {
      myfile_WL<<wrench_L[i]<<"\t";
  } 
  myfile_WL<<"\n";
  
return;
  
}

void write_file_right_wrench(){
  myfile_WR<<elapsed_time<<"\t";
    for (int i = 0; i < 6; i++)
  {
      myfile_WR<<wrench_R[i]<<"\t";
  } 
  
  myfile_WR<<"\n";
  
return; 
}

void write_file_traj(){
  myfile_traj<<elapsed_time<<"\t";
    for (int i = 0; i < 6; i++)
  {
      myfile_traj<<traj[i]<<"\t";
  } 
  myfile_traj<<"\n";
  
return; 
}

void write_file_dist_vel(){
  myfile_pv<<elapsed_time<<"\t";

  myfile_pv<<dist<<"\t"<<des_speed<<"\t"<<Current_Speed<<"\t";
   
  myfile_pv<<"\n";
  
return; 
}

void write_file_filtered_dist(){
    myfile_pv<<elapsed_time<<"\t";

  myfile_pv<<dist<<"\t"<<num_sim<<"\t";
   
  myfile_pv<<"\n";
}

void write_file_eq_pos(){
   myfile_pos<<elapsed_time<<"\t";
  for (int i = 0; i < 6; i++)
  {
//       myfile_pos<<pos[i]+bias[i]*pi/180<<"\t";
    myfile_pos<<pos[i]<<"\t";//now bias is considered in synergies .cpp
  } 
  myfile_pos<<"\n";  
  return;
}



void opt_write_curr_meas(const std_msgs::Float64MultiArray::ConstPtr& msg){
  curr_meas.resize(msg->data.size());
  std::cout<<"size "<<msg->data.size()<<std::endl;
  for(int i=0;i<msg->data.size();i++){
    curr_meas[i]=msg->data.at(i);    
  }
  
}

void opt_write_curr_volt(const std_msgs::Float64MultiArray::ConstPtr& msg){
  volt_meas.resize(msg->data.size());
  std::cout<<"size "<<msg->data.size()<<std::endl;
  for(int i=0;i<msg->data.size();i++){
    volt_meas[i]=msg->data.at(i);    
  }
  
}

void write_file_curr(){
     myfile_curr<<elapsed_time<<"\t";
  for (int i = 0; i < curr_meas.size(); i++)
  {
//       myfile_pos<<pos[i]+bias[i]*pi/180<<"\t";
    myfile_curr<<curr_meas[i]<<"\t";//now bias is considered in synergies .cpp
  } 
  myfile_curr<<"\n";  
  return;
}

void write_file_volt(){
       myfile_volt<<elapsed_time<<"\t";
  for (int i = 0; i < volt_meas.size(); i++)
  {
//       myfile_pos<<pos[i]+bias[i]*pi/180<<"\t";
    myfile_volt<<volt_meas[i]<<"\t";//now bias is considered in synergies .cpp
  } 
  myfile_volt<<"\n";  
  return;
}
int main(int argc, char** argv){
    ros::init(argc,argv,"Save");
    ros::NodeHandle n;

//--------------------------------------------
    std::string file_results_WL="Left_Wrench.txt";
    std::string file_results_WR="Right_Wrench.txt";
    std::string file_results_pv="Distance_Speed.txt";
    std::string file_results_traj="Joint_Traj.txt";
    std::string file_results_pos="Eq_pos.txt";
    std::string file_results_curr="Current.txt";
    std::string file_resutls_voltage="Voltage.txt";



    
//     std::string path_file_results="/home/Documents/Leg_Synergies_models/Treadmill_Exp_Results/";
    std::string path_file_results="/home/gian/Documents/Treadmill_Exp_Results/";
    
//     New coding for path file
    time_t now = time(0);
    tm *ltm = localtime(&now);
    std::string Year=std::to_string( 1900 + ltm->tm_year);
    std::string Month=std::to_string( ltm->tm_mon+1);
    std::string Day=std::to_string( ltm->tm_mday);
    std::string Hour=std::to_string(ltm->tm_hour);
    std::string Min=std::to_string(ltm->tm_min);
    std::string folder="Res_"+Year+"_"+Day+"_"+Month+"_"+Hour+"_"+Min;
    std::string cmd="mkdir -p "+ path_file_results + folder;

    std::string check_fold=path_file_results + folder;

    
    DIR* dir = opendir(check_fold.c_str());
if (dir)
{
    /* Directory exists. */
    closedir(dir);
}
else 
{
    /* Directory does not exist. */
    std::cout<<cmd<<std::endl;
    system(cmd.c_str());  
    ROS_INFO_STREAM("*** FOLDER DOES NOT EXIST! ***");
}

   
    file_results_WL=path_file_results+folder+"/"+file_results_WL;
    file_results_WR=path_file_results+folder+"/"+file_results_WR;
    file_results_pv=path_file_results+folder+"/"+file_results_pv;
    file_results_traj=path_file_results+folder+"/"+file_results_traj;
    file_results_pos=path_file_results+folder+"/"+file_results_pos;
    file_results_curr=path_file_results+folder+"/"+file_results_curr;
    file_resutls_voltage=path_file_results+folder+"/"+file_resutls_voltage;
    
    myfile_WL.open(file_results_WL.c_str());
    myfile_WR.open(file_results_WR.c_str());
    myfile_pv.open(file_results_pv.c_str());
    myfile_traj.open(file_results_traj.c_str());
    myfile_pos.open(file_results_pos.c_str());
    myfile_curr.open(file_results_curr.c_str());
    myfile_volt.open(file_resutls_voltage.c_str());
   
    
    
//--------------SUBSCRIBER--------------------          

//     ros::Subscriber opt_sub_bias=n.subscribe("/optimal_walk_run/optimal_bias",10,&opt_write_bias);
    ros::Subscriber opt_sub_bias=n.subscribe("/optimal_walk/optimal_bias",10,&opt_write_bias);
    ros::Subscriber zmp_sub_bias=n.subscribe("/zmp_walk/optimal_bias",10,&zmp_write_bias);
    ros::Subscriber opt_sub_wrenches_R=n.subscribe("/wrenches/right_wrenches",10,&write_right_wrench);
    ros::Subscriber opt_sub_wrenches_L=n.subscribe("/wrenches/left_wrenches",10,&write_left_wrench);
    ros::Subscriber opt_sub_eq_pos=n.subscribe("/qb_legs_trajectories",10,&opt_write_eq_pos);
    ros::Subscriber opt_sub_traj_meas=n.subscribe("/qb_legs_measures",10,&opt_write_traj_meas);
    ros::Subscriber opt_sub_curr_meas=n.subscribe("/qb_legs_currents",10,&opt_write_curr_meas);
    ros::Subscriber sub_distance_data_sensor=n.subscribe("/filtered_distance",10,&write_filtered_dist);
    ros::Subscriber opt_sub_volt_meas=n.subscribe("/qb_legs_voltage",10,&opt_write_curr_volt);
    ros::Subscriber opt_sub_sim=n.subscribe("/optimal_walk/des_opt",10,&opt_write_sim);
    ros::Subscriber zmp_sub_sim=n.subscribe("/zmp_walk/des_opt",10,&zmp_write_sim);
    
    begin = ros::Time::now();    
    ros::Rate while_rate(300);
    
    
    while(ros::ok()){
      

      
      elapsed_time=ros::Time::now()-begin;
      
      write_file_right_wrench();
      write_file_left_wrench();
      write_file_traj();
      write_file_eq_pos();
//       write_file_dist_vel();
      write_file_filtered_dist();
      write_file_curr();
      write_file_volt();
      	  
      ros::spinOnce();
      while_rate.sleep();
    }
    
    myfile_WL.close();
    myfile_WR.close();
    myfile_pv.close();
    myfile_traj.close();
    myfile_pos.close();
    myfile_volt.close();
    

}
