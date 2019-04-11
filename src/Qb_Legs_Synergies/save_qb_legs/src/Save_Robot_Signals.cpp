// This code simply spawn a set of subscriber which retrieve the robot signals and print them on a specific file

#include <iostream>
#include <vector>
#include </lib/eigen-eigen/Eigen/Dense>

#include <unistd.h>
#include <string>
#include <array>
#include <math.h>   
#include <boost/concept_check.hpp>
#include <std_msgs/String.h>
#include <ros/node_handle.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

#include <synergies/traj_state.h>
#include <synergies/bias.h>
#include <synergies/gain_bias_err.h>


#include <synergies/speed_srv.h>
#include <synergies/warm_start_srv.h>
#include <synergies/warm_stop_srv.h>


#include <trajectory_msgs/JointTrajectory.h>
#include <map>

#include <std_msgs/Float32.h>
#include <control_msgs/JointTrajectoryControllerState.h>


#include <iostream>
#include <fstream>
#include <ctime>
#include <dirent.h>

double des_speed(0);
std::vector<double> pos(6);

void write_des_speed(const std_msgs::Float32& msg){
  des_speed=msg.data;
}

void write_cube(const control_msgs::JointTrajectoryControllerStateConstPtr& msg){
//   pos()<<msg.actual.positions[0];
}


bool flag_write=false;
std::ofstream myfile;
ros::Time begin;
ros::Duration elapsed_time;

int main(int argc, char** argv){
    ros::init(argc,argv,"Save");
    ros::NodeHandle n;
    
    
    ros::Subscriber sub_des_speed=n.subscribe("/synergies_des_speed",10,&write_des_speed);
    std::map<int,ros::Subscriber> sub_qb_leg_cubes;
    
//     std::vector<int> ids = {1,2,3,4,5,6};
//     for(auto id:ids)
//     {
//         std::string qb_topic = "/qb_legs/cube"  + std::to_string(id) + "/cube" + std::to_string(id) + "_position_and_preset_trajectory_controller/state";
// 	sub_qb_leg_cubes[id] = n.subscribe(qb_topic,10,&write_cube);
//     }
//   
// 
//     std::string file_results="joints.txt";
// //     std::string path_file_results="/home/Documents/Leg_Synergies_models/Treadmill_Exp_Results/";
//     std::string path_file_results="/home/gian/Treadmill_Exp_Results/";
//     
// //     New coding for path file
//     time_t now = time(0);
//     tm *ltm = localtime(&now);
//     std::string Year=std::to_string( 1900 + ltm->tm_year);
//     std::string Month=std::to_string( ltm->tm_mon+1);
//     std::string Day=std::to_string( ltm->tm_mday);
//     std::string Hour=std::to_string(ltm->tm_hour);
//     std::string Min=std::to_string(ltm->tm_min);
//     std::string folder="Res_"+Year+"_"+Day+"_"+Month+"_"+Hour+"_"+Min;
//     std::string cmd="mkdir -p "+ path_file_results + folder;
// //     ROS_INFO_STREAM("***** FOLDER ******");
//     std::string check_fold=path_file_results + folder;
// //     ROS_INFO_STREAM("***** FOLDER ******");
//     
//     DIR* dir = opendir(check_fold.c_str());
// if (dir)
// {
//     /* Directory exists. */
//     closedir(dir);
// }
// else 
// {
//     /* Directory does not exist. */
//     std::cout<<cmd<<std::endl;
//     system(cmd.c_str());  
//     ROS_INFO_STREAM("*** FOLDER DOES NOT EXIST! ***");
// }
// 
//     file_results=path_file_results+folder+"/"+file_results;
//     myfile.open(file_results.c_str());
//     
// //     ros::Time 
//     begin = ros::Time::now();
// //     ros::Duration elapsed_time;


} 
  
  
  
  
//   myfile<<qb_val<<"\t";
//   }
// 
//     } //end for pub qb_legs cube
//     if(flag_write){
//       elapsed_time = ros::Time::now() - begin;
//       myfile<< elapsed_time<<"\n";}
