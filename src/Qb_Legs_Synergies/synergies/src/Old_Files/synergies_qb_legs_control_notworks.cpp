/* Code provided by Gian Maria Gasparri Ph.D. Centro E. Piaggio 09/07/2017
 * 
 * This code allows the robot to transit from one speed to another one. 
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
 * 1 - INIT - Create publisher message and service in order to provide signlas and to modify them
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
#include <synergies/spline.h>
#include "synergies/synergy_values.h"

#include <synergies/speed_srv.h>
#include <synergies/warm_start_srv.h>
#include <synergies/warm_stop_srv.h>

#include <synergies/synergy_map_fun.h>


#include <trajectory_msgs/JointTrajectory.h>
#include <map>

#include <std_msgs/Float32.h>


#include <iostream>
#include <fstream>
#include <ctime>
#include <thread>
#include <dirent.h>

#include <mutex>


std::map<int,trajectory_msgs::JointTrajectory> msgs;

const double pi=3.141592;   


bool Warm_Start=false;
bool Warm_Stop=false;
// to move the robot through the distance sensor
bool flag_sensor_subsribe_vel=true;

double interp_Instant=1;

bool flag_pub_qbLegs_robot=true;

bool flag_concatenate_completed=true;

std::vector<double> bias={-1,0,17,-3,0,18}; //deg
std::vector<double> old_bias=bias;
std::vector<double> bias_stop={0,0,17,-2,0,19};
// std::vector<double> bias={-1,0,17,-2,0,18}; //deg
// std::vector<double> bias={4,0,17,3,0,18}; //deg
tk::spline bias_s;

//if speed variation is less then 0.01 m/s ingnore it

double min_speed=0.05;//0.025;
double Current_Speed=min_speed;
double speed=min_speed;

double Var_speed_threshold=0.005;

double try_current;
double index_time(0);
double min_v_des=min_speed;

double Trunk_Gain(0);
double min_Trunk(-10);
double max_Trunk(10);
// double Ankle_Gain(0);
// double min_Ankle(15);
// double max_Ankle(25);
double Ankle_Gain(-10);
double min_Ankle(17);
double max_Ankle(20);


double err_tilde(0);
double bias_err_tilde_T(0);
double bias_err_tilde_A(0);
double BT(0);


// Service to modify speed   
bool setSpeedCallback(synergies::speed_srv::Request& request, synergies::speed_srv::Response& response)
{
   ROS_INFO("Desired Speed : %lf ", request.speed);
   Current_Speed=request.speed;
 
   return true;
}

void BiasCallback(const synergies::bias& msg)
{
   std::cout<<"Current bias = ";
   for(int i=0;i<6;i++){
     std::cout<< bias[i]<<", ";
  }
  std::cout<<std::endl;
  
    std::cout<<"New bias = ";
   for(int i=0;i<6;i++){
     bias[i]=msg.bias[i];
     old_bias[i]=bias[i];
     std::cout<< bias[i]<<", ";
  }
  std::cout<<std::endl; 
  
return;
}


// rostopic pub -1 /synergies/bias synergies/bias '{bias: [0, 0, 15, -2, 0, 16]}'

void Bias_Stop_Callback(const synergies::bias& msg)
{
   std::cout<<"Current bias_stop = ";
   for(int i=0;i<6;i++){
     std::cout<< bias_stop[i]<<", ";
  }
  std::cout<<std::endl;
  
    std::cout<<"New bias_stop = ";
   for(int i=0;i<6;i++){
     bias_stop[i]=msg.bias[i];
//      old_bias[i]=bias[i];
     std::cout<< bias_stop[i]<<", ";
  }
  std::cout<<std::endl; 
  
return;
}

// rostopic pub -1 /synergies/bias_stop synergies/bias '{bias: [-1, 0, 17, -3, 0, 18]}'



void update_bias_trunk_ankle(const std_msgs::Float32::ConstPtr& msg){
//   if(fabs(err_tilde-(msg->data))){return;}else{
//   std::cout<<"Distance update bias"<<std::endl;
  
  err_tilde=msg->data;
//   std::cout<<"Distance update bias "<<err_tilde<<std::endl;
  
  //bias trunk 0 and 3
  bias_err_tilde_T=Trunk_Gain*err_tilde;
//   if(bias_err_tilde>=max_trunk_bias){bias_err_tilde_T=0};
//   if(bias_err_tilde<=min_trunk_bias){bias_err_tilde_T=0};
  bias[0]=old_bias[0]+bias_err_tilde_T;
  bias[3]=old_bias[3]+bias_err_tilde_T;
  if(bias[0]>=max_Trunk){bias[0]=max_Trunk; std::cout<<" Max Trunk bias "<<std::endl;}
  if(bias[0]<=min_Trunk){bias[0]=min_Trunk; std::cout<<" Min Trunk bias "<<std::endl;}
  if(bias[3]>=max_Trunk){bias[3]=max_Trunk;}
  if(bias[3]<=min_Trunk){bias[3]=min_Trunk;}
  
  //bias ankle 2 and 5
  bias_err_tilde_A=Ankle_Gain*err_tilde;
//   if(bias_err_tilde>=max_ankle_bias){bias_err_tilde_A=0};
//   if(bias_err_tilde<=min_ankle_bias){bias_err_tilde_A=0};
  bias[2]=old_bias[2]+bias_err_tilde_A;
  bias[5]=old_bias[5]+bias_err_tilde_A;
  
  if(bias[2]>=max_Ankle){bias[2]=max_Ankle; std::cout<<" Max Ankle bias "<<std::endl;}
  if(bias[2]<=min_Ankle){bias[2]=min_Ankle; std::cout<<" Min Ankle bias "<<std::endl;}  
  if(bias[5]>=max_Ankle){bias[5]=max_Ankle;}
  if(bias[5]<=min_Ankle){bias[5]=min_Ankle;}
  
  
//   for(int i=0;i<6;i++){
    
  
      std::cout<<"New bias = ";
   for(int i=0;i<6;i++){
     std::cout<< bias[i]<<", ";
  }
  std::cout<<std::endl;
  
  
  
//   } 
  
}

void update_trunk_ankle_gain(const synergies::gain_bias_err& msg){
//  std::cout<< " cane " <<std::endl;
Trunk_Gain=msg.gain_bias_err[0];  
min_Trunk=msg.gain_bias_err[1];
max_Trunk=msg.gain_bias_err[2];
Ankle_Gain=msg.gain_bias_err[3];
min_Ankle=msg.gain_bias_err[4];
max_Ankle=msg.gain_bias_err[5];
// ROS_INFO(" ************** UU ************* ");
std::cout<<" Trunk G "<<Trunk_Gain<<" min "<<min_Trunk<<" max "<<max_Trunk<<" Ankle G "<<Ankle_Gain<<" min "<<min_Ankle<<" max "<<max_Ankle<<std::endl;
}


// rostopic pub -1 /gain_bias_trunk_ankles synergies/gain_bias_err '{gain_bias_err: [0, -10, 10, 0.1, 12, 19]}'


void update_BT(const std_msgs::Float32& msg){

  BT=msg.data;

std::cout<<" New BT "<<BT<<std::endl;
}

// rostopic pub -1 /update_BT  std_msgs/Float32 '{data: 1}'

bool setWarmStartCallback(synergies::warm_start_srv::Request& request, synergies::warm_start_srv::Response& response)
{
   ROS_INFO("Warm_Start");
   Warm_Start=true;
   ROS_INFO("Warm %d",Warm_Start);
//    if(Warm_Start==false)
//    Warm_Start=true;
 
   return true;
}

bool setWarmStopCallback(synergies::warm_stop_srv::Request& request, synergies::warm_stop_srv::Response& response)
{
   ROS_INFO("Warm_Stop");
//    if(Warm_Stop==false)
   Warm_Stop=request.Wsto;
 ROS_INFO("Warm %d",Warm_Stop);
   return true;
}



void update_des_speed_CallBack(const std_msgs::Float32::ConstPtr& msg){
  
try_current=msg->data;

// Update Current speed only in case of valid value and if you've completed the previous concatenation
if((fabs(speed-try_current)>=Var_speed_threshold) && flag_concatenate_completed && try_current>min_v_des){ 
  Current_Speed=msg->data;
flag_concatenate_completed=false;
// std::cout<<"Current Speed callback update "<<Current_Speed<<std::endl;
  
}

if((try_current<=min_v_des)  && (flag_concatenate_completed)){
Current_Speed=min_v_des;
Warm_Stop=0;
Warm_Start=1;
// std::cout<<"Current Speed callback min v des"<<Current_Speed<<std::endl;  
}

return;
}




//---------------------------------------------
//-----------------------------------------------------------


    int index_traj;
    
    std::vector<double> traj_d={0,0,0,0,0,0};
    
    bool flag_pub_traj=true;
    bool flag_concatenate_started=false;
    
    double Step_old,T_old,Ts_old,pub_Ts(0.006),curr_Ts;
    double Step_new,T_new,Ts_new;
    
    double exp_val=0.56;
    double l=0.3;
    double g=9.81;
    
 
    Eigen::VectorXd res_traj;
    std::vector<double> res_traj_2(60);
    int Instant(0),Current_Instant(0);
    
//     pub_Ts=0.006;

    double qb_val(0);
    int pm(1);
    int n_count_qb(0);
    
    
    
    int new_num, old_num, min_num, pub_num;
    
//     bool flag_concatenate_completed=true;
    bool one_time=true;
    
    tk::spline new_s;
    tk::spline old_s;
    tk::spline trans_s; //transition
    
    Eigen::MatrixXd New_Traj_pub(1,6);
    Eigen::MatrixXd Old_Traj_pub(1,6);
    Eigen::MatrixXd Middle_Traj_pub(1,6);
    Eigen::MatrixXd Output_Traj(1,6);
    Eigen::MatrixXd Err_Traj(1,6);
    
    std::vector<double> bt,bs;

    double sup_bias;
    std::vector<double> Current_Speed_v;
    
       Eigen::MatrixXd New_Traj(1,6),Old_Traj(1,6),New_Traj_supp(1,6);
    
   bool first_time=true; 

    
   Eigen::MatrixXd First_mu;
   Eigen::MatrixXd First_Synergy;
   Eigen::MatrixXd Second_mu;
   Eigen::MatrixXd Second_Synergy;
   
   double foot_h=0.04; //1.5 cm
   
   //--------------------------------------------

   bool data_sent=true;
   bool thread_completed=true;
   std::mutex m;
   
void interpolate_thread(){
  
  while(1){
    m.lock();
    if(data_sent && thread_completed){
      
      thread_completed=false;
      data_sent=false;
    
    std::cout<<" while thread"<<std::endl;
      
//--------------------------TRAJ---------------------------      
      
//     Call Mapping function 
      if(first_time){
	
	  // speed is the correct desired speed
	  if(flag_sensor_subsribe_vel==false){
	  speed=0.061;
// 	  	Warm_Stop=false;
// 	Warm_Start=true;
// 	std::cout<<"Start: "<<Warm_Start<<" Stop: "<<Warm_Stop<<std::endl;
// 	  Current_Speed=speed; 
	    }else{
	      if(Current_Speed<=min_v_des) {Current_Speed=min_v_des; }  //first_time=true;} modified 21/07/2017}
	      
	      speed=Current_Speed;
	      
	    }
	    
	  Current_Speed=speed;
	  
	  Step_new=pow(Current_Speed/(sqrt(g*l)),exp_val)*l;
	  T_new=Step_new/Current_Speed;
	  std::cout<<"T_new "<<T_new<<std::endl;
	  Ts_new=T_new/30; // 30 is the sample number of one step (!)
	  
// 	  Current Sample Time
	  curr_Ts=Ts_new;
// 	  New data dimensions as function of speed and Ts
	  new_num=std::round(T_new/pub_Ts)*2; //because we have two steps in the analyzed trajectories (!)
	  old_num=new_num;
	  std::cout<<"first "<< "speed "<<speed<<" foot "<<foot_h<<std::endl;
	  New_Traj=From_Syn_To_Traj( speed, foot_h, First_Synergy, Second_Synergy,First_mu,Second_mu);
	  std::cout<<"after first"<<std::endl;
	  Old_Traj=New_Traj;
	      
	  first_time=false;
	  std::cout<<"Activation speed "<<std::to_string(Current_Speed)<<std::endl;
	  	  
	  New_Traj_pub=Interpolate_and_Resample(New_Traj, Ts_new, pub_Ts);
	  Old_Traj_pub=New_Traj_pub;	  
	  	  
      }else{ std::cout<<"else"<<std::endl;
	// if it is not the first time and speed has not changed nothing have to be done
	// Otherwise if speed is modified we have to:
	// 1 - Create two batches the first composed of the old and new signals the second just of the new ones
	// 2 - If it is too late continue to publish the old one, then publish the mixed batch and hence the new one.
	// the second step could take awhile hence if concatenate operation is started you do not have to modify stuff
	
	if(speed!=Current_Speed && one_time==true && fabs(Current_Speed-speed)>=Var_speed_threshold){
	  std::cout<<"Speed "<<speed<< " Current "<<Current_Speed<<std::endl;
	  //a change occurs hence you need to concatenate
	      Step_old=Step_new;
	      T_old=T_new; //This is the time interval within one step has to be done
	      std::cout<<"T_old "<<T_old<<" speed "<<speed<<" Current_Spee "<<Current_Speed<<std::endl;
	      Ts_old=Ts_new; // 30 is the number of samples of one step
	      
	      Step_new=pow(Current_Speed/(sqrt(g*l)),exp_val)*l;
	      T_new=Step_new/Current_Speed;
	      Ts_new=T_new/30; // 30 is the sample number of one step
	      
	      Old_Traj=New_Traj;
	      New_Traj=From_Syn_To_Traj(Current_Speed, foot_h, First_Synergy, Second_Synergy, First_mu, Second_mu);
	    std::cout<<"Interpolate "<<"N "<<New_Traj.rows()<<" "<<New_Traj.cols()<<" Ts_new "<<Ts_new <<" pub_Ts "<<pub_Ts<<std::endl;
// 	      Prepare the signal batches
	      New_Traj_pub=Interpolate_and_Resample(New_Traj, Ts_new, pub_Ts);
	      std::cout<<"Interpolate "<<"O "<<Old_Traj.rows()<<" "<<Old_Traj.cols()<<" Ts_old "<<Ts_old <<" pub_Ts "<<pub_Ts<<std::endl;
	      Old_Traj_pub=Interpolate_and_Resample(Old_Traj, Ts_old, pub_Ts);
	      
// 	      Find when you have to move from old to new signals
	      Eigen::Index i_g, j_g;
	      

	      double val_1;
	      double sum_val(0);
	      std::vector<double> err(6);
	      Eigen::VectorXd ERR(60);
	      for(int j=0;j<60;j++){
		sum_val=0;
		for(int i=0;i<6;i++){
		val_1=(Old_Traj(j,i)-New_Traj(j,i)); // This is the value of the error
// 		std::cout<<"val "<<val_1<<std::endl;
		sum_val+=fabs(val_1);
// 		std::cout<<"err[i] "<<err[i]<<std::endl;
		}
		ERR(j)= sum_val;
		
	      }

	      val_1=ERR.minCoeff(&i_g,&j_g); // This is the value of the error

	      Instant=i_g; // This is the instant at which you should switch the signals
	      if(Instant==0) Instant=1;
	      
	      std::cout<<"before interp_Instant "<<std::endl;

	      
	      //      double interp_Instant=Instant;
	      interp_Instant=std::round(Instant*Ts_old/pub_Ts);  
	      
	      std::cout<<"interp_Instant "<<std::endl;
	      
// 	      Create the mixed batch
	      Middle_Traj_pub=Interpolate_and_Resample_Middle(New_Traj, Ts_new, Old_Traj, Ts_old, Instant, pub_Ts);

// 	      Now all the signals have been created hence i have not to enter in this statement again
	      flag_concatenate_started=true;
	      flag_concatenate_completed=false;
	      } //end if speed change occurs
	      
	      std::cout<<"after speed change"<<std::endl;
	      
      } //end if else first time and speed change occurs
      thread_completed=true;
    }// end data and thread
     m.unlock();
} //end while

  std::cout<<" end thread"<<std::endl;
 
  
}//end thread function




int main(int argc, char** argv){
  
    ros::init(argc,argv,"Synergies");
    ros::NodeHandle n;
  

//--------------------------INIT----------------------------
// Initialize  

    
    ros::Rate rate_while(1/pub_Ts);
    
 
//     Set publisher to publish trajectories into "synergies/Trajectories
    ros::Publisher pub_traj= n.advertise<synergies::traj_state>("synergies/Trajectories",10);
    ros::Subscriber sub_bias=n.subscribe("/synergies/bias",1,&BiasCallback);
    ros::Subscriber sub_bias_stop=n.subscribe("/synergies/bias_stop",1,&Bias_Stop_Callback);
    
//     TO PUBLISH{15,0,18,0,0,20}
//     rostopic pub -1 /synergies/bias synergies/bias '{bias: [8, 0, 18, 0, 0, 20]}'
    
//     std::cout<<"Publisher"<<std::endl; 
//     ros::Subscriber sub_traj=n.subscribe<synergies::joint_state>("synergies/Trajectories",&TrajectoryCallBack());
    ros::Subscriber sub_speed=n.subscribe("/synergies_des_speed",1,&update_des_speed_CallBack);
    
    ros::Subscriber sub_err_bias=n.subscribe("/bias_trunk_ankles",1,&update_bias_trunk_ankle);
    ros::Subscriber sub_gain_err_bias=n.subscribe("/gain_bias_trunk_ankles",1,&update_trunk_ankle_gain);
    ros::Subscriber sub_gain_BT=n.subscribe("/update_BT",1,&update_BT);
    
   
    // Create service to enable speed change by user cmd and to do a warm start and stop
    ros::ServiceServer srv_setSpeed = n.advertiseService("synergies/Set_Speed", &setSpeedCallback);
    std::cout<<"Service synergies/Set_Speed"<<std::endl; 
    ros::ServiceServer srv_warm_start = n.advertiseService("synergies/Warm_Start", &setWarmStartCallback);
    std::cout<<"Service synergies/Warm_Start"<<std::endl; 
    ros::ServiceServer srv_warm_stop = n.advertiseService("synergies/Warm_Stop", &setWarmStopCallback);
    std::cout<<"Service synergies/Warm_Start"<<std::endl; 
    
    
    
    

// Initialize synergies, i.e. the synergies SVD matrices are written in U_mu_1/2 with gloabl visibility



    
   synergies::traj_state msg;

//  Choose rate 
   initialize_synergies();
   
   First_mu=U_mu_1;
   First_Synergy=U_1;
   Second_mu=U_mu_2;
   Second_Synergy=U_2;

    
    
    
//------------------------INIT Warm Start & Stop--------------

double Ni_start=std::round(200*6*0.1); //200 steps with Ts_pub, i.e. 1.2 seconds , *6 due to the code position see below
double Ni_stop=std::round(200*6*0.5);// was 1 //200 steps with Ts_pub, i.e. 1.2 seconds , *6 due to the code position see below
double index_w_start=1;
double index_w_stop=1;
    
//------------------------INIT QB----------------------------
std::cout<<"init QB "<<std::endl;
//  std::vector<int> ids = {1,2,3,4,5,6};
 std::vector<int> ids = {1,2,3,4,5,6};
//  std::vector<int> ids = {2};
    
    std::map<int,ros::Publisher> qb_legs_pubs;
    
    double stiff_ref=0.97;
 // Initialize qb at 0 position
    for(auto id:ids)
    {
        std::string qb_topic = "/qb_legs/cube"  + std::to_string(id) + "/cube" + std::to_string(id) + "_position_and_preset_trajectory_controller/command";
	qb_legs_pubs[id] = n.advertise<trajectory_msgs::JointTrajectory>(qb_topic,10);
	
	//i create the msgs here in order to overwrite everytime and push data
	trajectory_msgs::JointTrajectoryPoint point;
	point.time_from_start.sec=index_time;
        point.time_from_start.nsec=pub_Ts/1e-9;
	
	// Equilibrium position reference
	point.positions.push_back(0.0);
	//Stiffness reference in percentage
	if((id==2)){stiff_ref=0.97;}else if((id==3)||(id==6)){
	stiff_ref=1;}
	if(id==5){stiff_ref=1;}
	point.positions.push_back(stiff_ref);
	
	//message to publish
	trajectory_msgs::JointTrajectory traj;
	
	std::string name_pos = "cube" + std::to_string(id) + "_shaft_joint";
	std::string name_stiff = "cube" + std::to_string(id) + "_stiffness_preset_virtual_joint";
	traj.joint_names.push_back(name_pos);
	traj.joint_names.push_back(name_stiff);
	traj.points.push_back(point);

	msgs[id] = traj;
	//publish the first position and stiffness to cube[id]
	qb_legs_pubs.at(id).publish(msgs.at(id));
    }
    
    
    ros::spinOnce(); 
    
    std::cout<<"end QB "<<std::endl;
//------------------------END INIT----------------------------    
      bool restart_again=true; //this works like an emergency stop and a fast restart.
      // if you go below the min speed your trajectories are set to 0. If you just decrease the 
      // min speed to zero due to the stride law the step time increases too much. Hence if you reach
      // min speed trajectories goes to zero then you have to reastart with a warm start
      index_traj=0;
      std::cout<<" Before thread"<<std::endl;
      
      std::cout<<speed<<std::endl;
      New_Traj=From_Syn_To_Traj( speed, foot_h, First_Synergy, Second_Synergy,First_mu,Second_mu);
      
      std::thread manage_speed_variation(interpolate_thread);
      
    while(ros::ok()){

	
      //if speed <speed_min
      if(Current_Speed <= min_speed){
	
	Current_Speed=min_speed;
	Warm_Stop=true;
	Warm_Start=false;
// 	if(!published){
// 	std::cout<<"Start: "<<Warm_Start<<" Stop: "<<Warm_Stop<<std::endl; published=!published}
// 	std::cout<<"Inside "<<" Curr "<<Current_Speed<<" speed "<<speed<<" "<<restart_again<<std::endl;	
      }else{
	Warm_Stop=false;
	Warm_Start=true;
// 	std::cout<<"Outside "<<" "<<restart_again<<std::endl;
// 	if(!published){
// 	std::cout<<"Start: "<<Warm_Start<<" Stop: "<<Warm_Stop<<std::endl;
      }
      
      
//--------------------------TRAJ---------------------------      
      
// // // // //     Call Mapping function 
// // // //       if(first_time){
// // // // 	
// // // // 	  // speed is the correct desired speed
// // // // 	  if(flag_sensor_subsribe_vel==false){
// // // // 	  speed=0.061;
// // // // // 	  	Warm_Stop=false;
// // // // // 	Warm_Start=true;
// // // // // 	std::cout<<"Start: "<<Warm_Start<<" Stop: "<<Warm_Stop<<std::endl;
// // // // // 	  Current_Speed=speed; 
// // // // 	    }else{
// // // // 	      if(Current_Speed<=min_v_des) {Current_Speed=min_v_des; }  //first_time=true;} modified 21/07/2017}
// // // // 	      
// // // // 	      speed=Current_Speed;
// // // // 	      
// // // // 	    }
// // // // 	    
// // // // 	  Current_Speed=speed;
// // // // 	  
// // // // 	  Step_new=pow(Current_Speed/(sqrt(g*l)),exp_val)*l;
// // // // 	  T_new=Step_new/Current_Speed;
// // // // // 	  std::cout<<"T_new "<<T_new<<std::endl;
// // // // 	  Ts_new=T_new/30; // 30 is the sample number of one step (!)
// // // // 	  
// // // // // 	  Current Sample Time
// // // // 	  curr_Ts=Ts_new;
// // // // // 	  New data dimensions as function of speed and Ts
// // // // 	  new_num=std::round(T_new/pub_Ts)*2; //because we have two steps in the analyzed trajectories (!)
// // // // 	  old_num=new_num;
// // // // // 	  std::cout<<"new_num "<<new_num<<std::endl;
// // // // // 	  std::cout<<"old_num "<<old_num<<std::endl;
// // // // // 	  New_Traj=From_Syn_To_Traj(speed, foot_h, First_Synergy, Second_Synergy, First_mu, Second_mu);
// // // // // 	  std::cout<<"des "<<pub_Ts<<" curr "<<curr_Ts<<std::endl;
// // // // // 	  New_Traj=From_Syn_To_Traj_Interpolated_Sampled_TEST(speed, foot_h, First_Synergy, Second_Synergy, First_mu, Second_mu,new_num,curr_Ts,pub_Ts);
// // // // 	  New_Traj=From_Syn_To_Traj( speed, foot_h, First_Synergy, Second_Synergy,First_mu,Second_mu);
// // // // 	  Old_Traj=New_Traj;
// // // // 	      
// // // // 	  first_time=false;
// // // // 	  std::cout<<"Activation speed "<<std::to_string(Current_Speed)<<std::endl;
// // // // 	  	  
// // // // 	  New_Traj_pub=Interpolate_and_Resample(New_Traj, Ts_new, pub_Ts);
// // // // 	  Old_Traj_pub=New_Traj_pub;
// // // // // 	  std::cout<<"New Traj size "<<New_Traj_pub.rows()<<" "<<New_Traj_pub.rows()<<std::endl;
// // // // // 	  std::cout<<"Old Traj size "<<Old_Traj_pub.rows()<<" "<<Old_Traj_pub.rows()<<std::endl;
// // // // // 	  pub_num=Old_Traj_pub.rows();
// // // // // 	  std::cout<<pub_num<<std::endl;
// // // // 	  
// // // // 	  
// // // // 	  	  
// // // //       }else{  //qui potrei mettere il thread
// // // // 	// if it is not the first time and speed has not changed nothing have to be done
// // // // 	// Otherwise if speed is modified we have to:
// // // // 	// 1 - Create two batches the first composed of the old and new signals the second just of the new ones
// // // // 	// 2 - If it is too late continue to publish the old one, then publish the mixed batch and hence the new one.
// // // // 	// the second step could take awhile hence if concatenate operation is started you do not have to modify stuff
// // // // 	if(speed!=Current_Speed && one_time==true && fabs(Current_Speed-speed)>=Var_speed_threshold){
// // // // // 	if(speed!=Current_Speed && flag_concatenate_started==false){
// // // // 	  
// // // // 	  //a change occurs hence you need to concatenate
// // // // 	      Step_old=Step_new;
// // // // 	      T_old=T_new; //This is the time interval within one step has to be done
// // // // // 	      std::cout<<"T_old "<<T_old<<std::endl;
// // // // 	      Ts_old=Ts_new; // 30 is the number of samples of one step
// // // // 	      
// // // // 	      Step_new=pow(Current_Speed/(sqrt(g*l)),exp_val)*l;
// // // // 	      T_new=Step_new/Current_Speed;
// // // // 	      Ts_new=T_new/30; // 30 is the sample number of one step
// // // // 	      
// // // // 	      Old_Traj=New_Traj;
// // // // 	      New_Traj=From_Syn_To_Traj(Current_Speed, foot_h, First_Synergy, Second_Synergy, First_mu, Second_mu);
// // // // 
// // // // // 	      Prepare the signal batches
// // // // 	      New_Traj_pub=Interpolate_and_Resample(New_Traj, Ts_new, pub_Ts);
// // // // 	      Old_Traj_pub=Interpolate_and_Resample(Old_Traj, Ts_old, pub_Ts);
// // // // 	      
// // // // // 	      Find when you have to move from old to new signals
// // // // 	      Eigen::Index i_g, j_g;
// // // // 	      
// // // // 
// // // // 	      double val_1;
// // // // 	      double sum_val(0);
// // // // 	      std::vector<double> err(6);
// // // // 	      Eigen::VectorXd ERR(60);
// // // // 	      for(int j=0;j<60;j++){
// // // // 		sum_val=0;
// // // // 		for(int i=0;i<6;i++){
// // // // 		val_1=(Old_Traj(j,i)-New_Traj(j,i)); // This is the value of the error
// // // // // 		std::cout<<"val "<<val_1<<std::endl;
// // // // 		sum_val+=fabs(val_1);
// // // // // 		std::cout<<"err[i] "<<err[i]<<std::endl;
// // // // 		}
// // // // 		ERR(j)= sum_val;
// // // // 		
// // // // 	      }
// // // // 
// // // // 	      val_1=ERR.minCoeff(&i_g,&j_g); // This is the value of the error
// // // // 
// // // // 	      Instant=i_g; // This is the instant at which you should switch the signals
// // // // 	      if(Instant==0) Instant=1;
// // // // 
// // // // 	      
// // // // 	      //      double interp_Instant=Instant;
// // // // 	      interp_Instant=std::round(Instant*Ts_old/pub_Ts);  
// // // // 	      //if you passed the optimal switching time search for a sub optimal one
// // // // // 	      if(index>interp_Instant){
// // // // // 		for(int j=0;j<60;j++){
// // // // 		  //15% of error
// // // // // 		  if((ERR(j)<=1.15*val_1) && (j>=index)){
// // // // // 		    Instant=j;
// // // // // 		    break;
// // // // // 		  }
// // // // // 		}
// // // // // 	      }
// // // // 	      
// // // // 	      
// // // // // 	      Create the mixed batch
// // // // 	      Middle_Traj_pub=Interpolate_and_Resample_Middle(New_Traj, Ts_new, Old_Traj, Ts_old, Instant, pub_Ts);
// // // // 
// // // // // 	      Now all the signals have been created hence i have not to enter in this statement again
// // // // 	      flag_concatenate_started=true;
// // // // 	      flag_concatenate_completed=false;
// // // // 	      } //end if speed change occurs
// // // // 	      
// // // //       } //end if else first time and speed change occurs
// // // // 

 interp_Instant=std::round(Instant*Ts_old/pub_Ts); 
     if(flag_pub_traj && thread_completed && !data_sent){ 
//        	      std::cout<<"New_Traj 0"<<New_Traj(0,0)<<std::endl;
// 	      std::cout<<"Current_Speed"<<Current_Speed<<std::endl;
// 	      std::cout<<"foot_h"<<foot_h<<std::endl;
       
		  if(speed==Current_Speed){
		    
		    Output_Traj=Old_Traj_pub;
		    pub_num=Old_Traj_pub.rows();
		  }else{ // else if speed!= Current_Speed
			one_time=false; //new speed variations are disabled
			
	    
		      if(flag_concatenate_started){ 
		  
			    if(index_traj>interp_Instant){
	      // 		      std::cout<<"too_soon"<<std::endl;
				Output_Traj=Old_Traj_pub;
				pub_num=Old_Traj_pub.rows();
			    //need to wait the next step, i.e. continue with current Output_Traj
			    }else{ 	   
				  if(index_traj==interp_Instant){
// 				      std::cout<<"Index==interp_Instant"<<std::endl;
				      pub_num=Middle_Traj_pub.rows();
				      Output_Traj=Middle_Traj_pub;
				      flag_concatenate_started=false;
				      flag_concatenate_completed=false;
// 				      std::cout<<"pub num "<<pub_num<<std::endl;
				  }else{
				      Output_Traj=Old_Traj_pub;
				      pub_num=Old_Traj_pub.rows();
			      }
			    }
		      }
		  }
	    
	    if(flag_concatenate_started==false &&  flag_concatenate_completed==false){
		if(index_traj==pub_num){
		    index_traj=0;
		    Output_Traj=New_Traj_pub;
		    Old_Traj_pub=Output_Traj;
		  
		    flag_concatenate_completed=true;   
		    pub_num=New_Traj_pub.rows();
		    std::cout<<"Speed modified from "<< speed <<" to "<<Current_Speed<<std::endl;
		    speed=Current_Speed;
		    one_time=true;
		    
	      }
	    }     
	  
	    if(index_traj==pub_num) index_traj=0;
   
	    
	    for(int i=0;i<6;i++){
	      
//--------------------------------------Warm Start & Warm Stop---------------
		
		traj_d[i]=Output_Traj(index_traj,i);
		
		if(Warm_Start && !Warm_Stop){
// 		  std::cout<<index_w_start<<" "<<Ni_start<<std::endl;
		  if(index_w_start>=Ni_start){
		    index_w_start=Ni_start;
		    index_w_stop=1;
// 		    return 0;
		    }else{
		    index_w_start+=1;
		  }
		  
		    traj_d[i]=traj_d[i]*index_w_start/Ni_start;


		}else if(!Warm_Start && Warm_Stop){
		  
		  if(index_w_stop>=Ni_stop){
// 		    std::cout<<" end "<<std::endl;
// 		    traj_d[i]=0;
// 		    index_w=1;
		    index_w_stop=Ni_stop;
		    index_w_start=1;
// 		    first_time=true; //modified 21/07/2017
// 		    flag_concatenate_started=false;
// 		    flag_concatenate_completed=true;
// 		    Warm_Start=false;
// 		    Warm_Stop=false;
		    
		  }else{
		    index_w_stop+=1;
// 		    std::cout<<((Ni-index_w)/Ni)<<std::endl;
		  }
		  
		    traj_d[i]=traj_d[i]*(1-index_w_stop/Ni_stop);

		  
	      }else if((!Warm_Start && !Warm_Stop)||(Warm_Start && Warm_Stop)) {
		Warm_Start=false;
		Warm_Stop=true;
// 				    flag_concatenate_started=false;
// 		    flag_concatenate_completed=true;
		index_w_stop=1;
		index_w_start=1;
	      }
		
// 		traj_d[i]*=pi/180;
	    } //end for i 



     
//   for(int i=0;i<6;i++){
//     traj_d[i]=traj_d[i];
//   }
//   std::cout<<"Curr "<<Current_Speed<<std::endl;
    
// 	    std::cout<<"New_Traj 0"<<New_Traj(0,0)<<std::endl;
// 	      std::cout<<"Current_Speed"<<Current_Speed<<std::endl;
// 	      std::cout<<"foot_h"<<foot_h<<std::endl;
//---------------------------------------PUB----------------------------------

//     std::cout<<std::to_string(New_Traj(0,0))<<std::endl;
	msg.joint_pos=traj_d;  
	pub_traj.publish(msg);
	
//---------------------------------------PUB QB LEGS--------------------------
    index_time+=pub_Ts;

    if(flag_pub_qbLegs_robot){
//       std::cout<< " init pub 1" <<std::endl;
    for(auto id:ids)
    {
	trajectory_msgs::JointTrajectoryPoint point;
	trajectory_msgs::JointTrajectory traj;
	
	// Equilibrium position reference
	point.time_from_start.sec=0;

        point.time_from_start.nsec=pub_Ts/1e-9;

	if((id==1) || (id==4)){
	  sup_bias=((Current_Speed-min_speed)/(0.15-min_speed))*BT;
	  if(sub_bias<=0){sup_bias=0;}
	}
	else{sup_bias=0;}
// // 	
	if(Current_Speed<=min_speed){
	qb_val=traj_d[id-1]+(bias_stop[id-1])*pi/180;}
	else{
	  qb_val=traj_d[id-1]+(bias[id-1]-sup_bias)*pi/180;
	}

	if(id==1 || id==2 || id==3){
	  qb_val*=-1;
	}
	point.positions.push_back(qb_val);
	//Stiffness reference 0.87-> 87% or radiants?
	point.positions.push_back(stiff_ref);
	
	std::string name_pos = "cube" + std::to_string(id) + "_shaft_joint";
	std::string name_stiff = "cube" + std::to_string(id) + "_stiffness_preset_virtual_joint";
	traj.joint_names.push_back(name_pos);
	traj.joint_names.push_back(name_stiff);
	traj.points.push_back(point);

	msgs[id] = traj;
	//publish the first position and stiffness to cube[id]
	qb_legs_pubs.at(id).publish(msgs.at(id));
	


    } //end for pub qb_legs cube

    
    } //end if flag_pub

	data_sent=true;
	
	} // end if pub flag_pub_traj
	Current_Instant=index_traj;
// 	std::cout<<Current_Instant<<std::endl;
	index_traj+=1;

    
    ros::spinOnce();
    rate_while.sleep();
   
    } //end while

    
return 0;

  
}