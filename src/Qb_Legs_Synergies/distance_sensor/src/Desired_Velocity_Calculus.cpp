// This code provide the reference speed as function of the error of the current robot position w.r.t. the desired one.
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
#include <cmath>
#include <math.h>
#include <phidgets_interface_kit/AnalogArray.h>
#include <distance_sensor/pid_msg.h>
#include <distance_sensor/N_msg.h>
#include <distance_sensor/GG_msg.h>
#include <distance_sensor/data_msg.h>

#include <distance_sensor/spline.h>
// #include "../../synergies/include/synergies/distance_sensor.h"

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>


# define EV 0.57
# define l 0.3
# define g 9.81
# define Ts 0.002 //the publish step time

// # define N_s_max 50        //max number of samples of the second filter
# define N_s_min 1          //min sample number
// # define WS_min 1e-4	//min speed value

double min_speed=0.065;
double v_des_min=min_speed;
double v_des_max=0.18;
double WS_min=min_speed;
double V0;
double GG(0.0024);
double v_des_out;

double v_des_out_r;
double quant=0.005;

//---------------GLOBAL---------------
Eigen::VectorXd sensor_IR_, array_sensor_pre_,dist_err;
double dist_err_vel;
// TO NOTE: msg->values[0] it means that i'm working with the measurement provided by the sensor 
// connected to the first channel CHECK THIS IF YOU ARE NOT READING SOMETHING


//desired distance
double desired_distance_ = 40.0; 
double distance_err = 0.0;
double distance_err_old = 0.0;

// to check if it is the first time that you take the measurement
bool flag_first_meas_ = true;
// PID gains
double kp_, ki_, kd_; 

// Initial value
double sensor_mean_distance_ = 0.0;

// Window -  static (true) or dynamic (false) dimension
bool N_const=true;

int N, N_old,N_s_max(30);
int N_v=30;

double coeff_correct_Ns=4; //modified 21/07/2017


bool activate_quant_flag=false;

//----------------GLOBAL To adjust measurements---------------

      double num_cond=8;
      std::vector<double> cond={20,30,40,50,60,70,80,90};
      std::vector<double> adj={0,3,5,5,6,7,8,9};
      double adj_val(0);
      double adj_estimated_current_distance;
      double acc_des;
      double k_wu;
      double anti_wu;
      tk::spline adj_s;
      
      
//------------------to adjust bias

double index_distance(0);
double err_tilde(0);
      
//       adj_s.set_points(cond,adj);


//------------------CALLBACKS---------------------

//Sensor measurement callback, first filter
void callback_sensor_IR(const phidgets_interface_kit::AnalogArray::ConstPtr& msg)
{
    // check NAN
//   std::cout<<__func__<<""<<std::endl;
    if(msg->values.size()<8) {std::cout<<"no size"<<std::endl;return;};
// std::cout<<__func__<<""<<msg->values[1]<<std::endl;

     if(!std::isnan(msg->values[1]))
    { //Convert to cm
//       std::cout<<__func__<<""<<msg->values[1]<<std::endl;
      sensor_IR_(0) = fabs(9462.0 / (msg->values[2] - 16.92)); 
      //Check boundaries
      if(sensor_IR_(0) < 20.0) sensor_IR_(0) = 20.0;
      if(sensor_IR_(0) > 100.0) sensor_IR_(0) = 100.0;
//       push and then calculate mean, update the global value sensor_mean_distance_
      array_sensor_pre_ << array_sensor_pre_.tail(array_sensor_pre_.size() - 1), sensor_IR_(0);
      sensor_mean_distance_ = array_sensor_pre_.mean();
      
    }
    if(!flag_first_meas_)
    {
      //update flag_first_meas_
      flag_first_meas_ = true;
    }

}

//callback update d_des
void callback_d_des(const std_msgs::Float32::ConstPtr& msg)
{
  desired_distance_ = msg->data;     // distanza in cm
std::cout<<"New desired distance "<<desired_distance_<<std::endl;
}

// rostopic pub -1 /d_des std_msgs/Float32 '{data: 10}'


//callback update N
void callback_update_N(const std_msgs::Float32::ConstPtr& msg)
{
  
   std::cout<<__func__ << " Inside N "<<std::endl;
  N=msg->data;
  
  N_old=N;
  N_s_max=N;
  std::cout<<" update N "<<N<<std::endl;
}

//callback update v min max
void callback_update_min_max_v(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  
  v_des_min=msg->data[0];
  v_des_max=msg->data[1];
  std::cout<<"Update V min "<<v_des_min <<" V max "<<v_des_max<<std::endl;
  
}


// rostopic pub -1 /update_N distance_sensor/N_msg '{N: 40}'

void callback_PID(const distance_sensor::pid_msg& msg)
{
ROS_INFO("pid gain kp_: %f, ki_: %f, kd_: %f \n\n", kp_,ki_,kd_);
  kp_ = msg.pid_gain[0];
  ki_ = msg.pid_gain[1];
  kd_ = msg.pid_gain[2];
  k_wu=msg.pid_gain[3];
  
}

// rostopic pub -1 /PID distance_sensor/pid_msg '{pid_gain: [0, 0, 0]}'

void callback_GG(const distance_sensor::GG_msg& msg)
{
// ROS_INFO("pid gain kp_: %f, ki_: %f, kd_: %f \n\n", kp_,ki_,kd_);

  GG = msg.gain;  
  std::cout<<" New GG "<<GG<<std::endl;
}

// rostopic pub -1 /update_GG distance_sensor/GG_msg '{gain: 0.00269}'

// rostopic pub -1 /update_GG distance_sensor/GG_msg '{gain: 450}'

void callback_update_quant(const std_msgs::Float32::ConstPtr& msg){

  quant=msg->data;
std::cout<<" Update quant to "<<quant<<std::endl;
}


void callback_activate_quant(const std_msgs::Bool::ConstPtr& msg){
  
  activate_quant_flag=msg->data;
  if(activate_quant_flag){std::cout<<" Dist Activate quant "<<std::endl;}else{std::cout<<" Dist Deactivate quant "<<std::endl;}
}

// Window size as function of the current speed, WS is the current Walking Speed
int N_function(double WS)
{
  double N_s;

  if(WS > WS_min)
  {

    double WSN = WS / sqrt(g * l);
    double SLN = pow(WSN, EV);
    double SL = SLN * l;
    double T = SL / WS;
    N_s = round(T / Ts /coeff_correct_Ns);
    std::cout<<" N_s "<<N_s<<std::endl;

    if(N_s < N_s_min) N_s = N_s_min;
    if(N_s > N_s_max) N_s = N_s_max;

  }
  else
  {
    N_s = N_s_max;
  }
  
  return N_s;
  
}

double inc_I;
// Eigen::VectorXd array_adj_speed;
bool flag_first_meas_adj=false;
double NI=10; //Window size
double adj_mean,distance_err_I;
double v_des = 0.0;
double GI=0;
double N_inc;

void callback_update_NI_I(const std_msgs::Float64MultiArray::ConstPtr& msg){
  
  NI=msg->data[0];
  GI=msg->data[1];

  std::cout<<" update NI to "<<NI<<" and GI to "<<GI<<std::endl;
}


// std::vector<double> INC_I(2);
// std::vector<double> I_vect;
// // // /*
// // // void incremet_speed(double adj_estimated_current_distance){ 
// // //   I_vect.resize(2);
// // // //tail(n) ->takes the last n elements        
// // //   array_adj_speed << array_adj_speed.tail(array_adj_speed.size() - 1),adj_estimated_current_distance;
// // //   array_adj_speed.conservativeResize(NI);
// // //   adj_mean=array_adj_speed.mean();
// // // //   std_msgs::Float32 adj_msg;
// // // //   adj_msg.data=adj_mean;
// // // //   pub_dist_I(adj_msg);
// // //   
// // //   distance_err_I = -(desired_distance_ - adj_mean);
// // //   
// // //   if (distance_err_I>=-3 && distance_err_I<=5){N_inc--;}
// // //   if(distance_err_I>5 && distance_err_I<=10){
// // //     //Nothing
// // //   }
// // //   if(distance_err_I>10){N_inc++;}
// // //         
// // //   if(v_des_out==0){N_inc=0;}
// // //   
// // //   inc_I=N_inc*GI;
// // //   
// // //   I_vect[0]=adj_mean;
// // //   I_vect[1]=inc_I;
// // // //   *I_vect_arr.data(0)
// // // //   *I_vect_arr.data(0)=adj_mean;
// // // //   *I_vect_arr.data(1)=inc_I;
// // // //   return INC_I;
// // // //   I_vect_arr
// // // }*/

//-------------------------MAIN---------------------

int main(int argc, char **argv)
{
  double rateHZ = 100;        //frequenza nodo
  
  // first desired speed
  
//   double v_des_out=0.0;
//   double v_des_min=min_speed;
//   double v_des_max=0.13;
  // first value
  double estimated_current_distance = 0.0;   //sensore filtrato nel main, quindi l'ultimo filtraggio
  // first gains
  double P = 0.0;
  double I = 0.0;
  double D = 0.0;
  // errors
  //Window size and previous one
//   int N_old;
  
  // array_sensor is the array composed of sensor data
  Eigen::VectorXd array_sensor, tmp,array_adj_speed;
  // here I put the velocity reference that have to be published
  std_msgs::Float32 v_pub;
  std_msgs::Float32 raw_data;
  std_msgs::Float32 filt_data;
  std_msgs::Float32 err_data;
  double FF_V_0=v_des_min;// feedforward speed
  // I use a PID to calculate the desired speed (maybe acceleration in future) as function of the error on position
//   kp_ = 0.0055;
//   ki_ = 0.000;
//   kd_ = 0.0;
// //   k_wu=50;
//   anti_wu=0;

  //initialize the vector where the phidgets put the measurements
  sensor_IR_ = Eigen::VectorXd::Zero(8);
  //array pre filtering process
  array_sensor_pre_= Eigen::VectorXd::Zero(10);
  //array post filtering process
  array_sensor = Eigen::VectorXd::Zero(N_s_min);
  array_adj_speed=Eigen::VectorXd::Zero(1);
  dist_err = Eigen::VectorXd::Zero(N_v);
  
  N = array_sensor.size();
  N_old = array_sensor.size();

 

  ros::init(argc, argv, "target_tracking");
  ros::NodeHandle n_;

  ros::Rate r(rateHZ);

  //Topic you want to pub
  ros::Publisher  pub_v_des = n_.advertise<std_msgs::Float32>("synergies_des_speed", 10);
  ros::Publisher pub_raw_data=n_.advertise<std_msgs::Float32>("raw_distance", 10);
  ros::Publisher pub_filt_data=n_.advertise<std_msgs::Float32>("filtered_distance", 10);
  ros::Publisher pub_err_tilde=n_.advertise<std_msgs::Float32>("bias_trunk_ankles",10);
  ros::Publisher pub_all_data_echo=n_.advertise<distance_sensor::data_msg>("distance_data_sensor",10);
  
  ros::Publisher pub_dist_I=n_.advertise<std_msgs::Float32>("distance_filtered_I",10);
 

  
  //Topics to subscribe to
  ros::Subscriber sub_sensor_IR, sub_d_des, sub_PID, sub_update_N, sub_GG, sub_min_max_v, sub_quant, sub_activate_quant,sub_update_NI_I;
  sub_sensor_IR = n_.subscribe("/cl4_gpio/analog_in", 10, &callback_sensor_IR);
  sub_d_des = n_.subscribe("/update_d_des", 10, &callback_d_des);
  sub_PID = n_.subscribe("/PID", 10, &callback_PID);
  
  sub_GG=n_.subscribe("/update_G",10,&callback_GG);
  sub_min_max_v=n_.subscribe("/update_min_max_v",10,&callback_update_min_max_v);
  sub_quant=n_.subscribe("/update_quant",10,&callback_update_quant);
  sub_activate_quant=n_.subscribe("/activate_quant",10,&callback_activate_quant);
  sub_update_NI_I=n_.subscribe("/update_NI_I",10,&callback_update_NI_I);
  sub_update_N=n_.subscribe("/update_N",10,&callback_update_N);
  
  
//       kp_ = 0.0055;
//       ki_ = 0.00005;
//       kd_ = 0.0;
      //   k_wu=50;
      anti_wu=0;
      
     double flag_err_dist=1;
  
     
     adj_s.set_points(cond,adj);
     distance_sensor::data_msg all_data_msg;
     
      std::cout<<std::endl;
      std::cout<<"----CMD----"<<std::endl;
      std::cout<<"rostopic echo /distance_data_sensor"<<std::endl;
      std::cout<<"rostopic pub -1 /update_GG distance_sensor/GG_msg '{gain: 0.00269}'"<<std::endl;
      std::cout<<"rostopic pub -1 /d_des std_msgs/Float32 '{data: 10}'"<<std::endl;

  while(ros::ok())
  {
    
    if(N_const){
     if(flag_first_meas_){
      N=N_s_max;
//            std::cout<<" N 0"<<std::endl;
      array_sensor << array_sensor.tail(array_sensor.size() - 1),sensor_mean_distance_;
      array_sensor.conservativeResize(N);
// 	std::cout<<" N 1"<<std::endl;
      
       }
      
    }else{
    
    if(flag_first_meas_)
    {

      //calculate the second filter window size
      N = N_function(v_des);
      
      //update dimension of array_sensor
      array_sensor.conservativeResize(N);
      
      if(N > N_old)
	{
	  if((N - N_old) == 1) //i.e. N=1 N_old=0
	  { // my old vector is smaller than the new one. In this case conservativeResize push a zero in the tail. Hence
	    // I take the NEW vector from the head and add on the top the new measurement.
	    array_sensor << array_sensor.head(N_old), sensor_mean_distance_;
	  }
	  else
	  { //Concatenate the vector. I add N-N_old
	    tmp = Eigen::VectorXd::Ones(N - N_old);
	    array_sensor << array_sensor.head(N_old), (tmp * sensor_mean_distance_);
	  }
	} // end if N>N_old
      else
	{
	  array_sensor << array_sensor.tail(array_sensor.size() - 1), sensor_mean_distance_;
	  
	}
      } //if flat first meas

    }//if else end with N_const  

     // To visualize the filter effect we publish filtered and unfiltered signals.
     raw_data.data=array_sensor(N-1);
     pub_raw_data.publish(raw_data);
     
     
     
      estimated_current_distance = array_sensor.mean();
      if (estimated_current_distance>=100)estimated_current_distance=100;
      if (estimated_current_distance<=10)estimated_current_distance=10;
      
      
      filt_data.data=estimated_current_distance;
      pub_filt_data.publish(filt_data);
      
//       std::cout<<"mean 1 : " << array_sensor.mean()<< " mean 2 : "<<  array_sensor.sum()/N<<std::endl;
      N_old = N;
      

      //--------------------------------ADJUST MEASUREMENT-------------------------------
//       for(int cond_i=0;cond_i<num_cond;cond_i++){
// 	      if(cond_i==0){adj_val=0;}
// 	      else if(estimated_current_distance<=cond[cond_i]/2 && estimated_current_distance>cond[cond_i-1]){
// 	adj_val=adj[cond_i-1]/2;
// 	      }else if(estimated_current_distance<=cond[cond_i] && estimated_current_distance>cond[cond_i-1]/2){
// 	adj_val=adj[cond_i-1];
// 	      }
//       }//end for
      
      if ((estimated_current_distance>20) && (estimated_current_distance<100)){
      adj_val=adj_s(estimated_current_distance);}
      
      
      
    adj_estimated_current_distance=estimated_current_distance+adj_val;
   
    
   //-----------------------------------PID------------------------------------------
    
    distance_err = -(desired_distance_ - adj_estimated_current_distance);
    

    
    kp_=0.00;
    P = kp_ * distance_err;
    I += ki_ * (distance_err + anti_wu*1) * (1 / rateHZ);
    D = kd_ * (distance_err - distance_err_old) * rateHZ;
    
 // As function of the error we have to modify the bias of trunk and ankle due to the friction between the robot and the support structure
// hence we have to publish a specific message to a new topic. Here we will set the publisher and in the synergies control file we will set the subscriber which
// will update the bias.

    
    dist_err_vel=distance_err_old-distance_err;
    dist_err << dist_err.tail(dist_err.size() - 1),dist_err_vel;
    dist_err.conservativeResize(N_v);
    err_tilde=dist_err.mean();
//     std::cout<<" err tilde "<<err_tilde<<std::endl;
    err_data.data=err_tilde;
    
    
    distance_err_old = distance_err;
    

//     if (distance_err<=-5) {V0=0;} 
//     else{
//       V0=distance_err*GG+FF_V_0;
//     }
    
      V0=distance_err*GG+FF_V_0;
//     std::cout<<" "<<I_vect[0]<<" "<<I_vect[1]<<std::endl;
//     incremet_speed(adj_estimated_current_distance);
//       std_msgs::Float32 adj_msg;
//       adj_msg.data=I_vect[0];
//       pub_dist_I.publish(adj_msg);
//     
//       I=I_vect[1];
//       
//       std::cout<<" "<<I_vect[0]<<" "<<I_vect[1]<<std::endl;
      
//   std::cout<<__func__<<" array 0"<<std::endl;    
   array_adj_speed << array_adj_speed.tail(array_adj_speed.size() - 1),adj_estimated_current_distance;
  array_adj_speed.conservativeResize(NI);
  adj_mean=array_adj_speed.mean();
//   std::cout<<__func__<<" array 1"<<std::endl;  
  std_msgs::Float32 adj_msg;
  adj_msg.data=adj_mean;
//   std::cout<<__func__<<" array 2"<<std::endl;  
  pub_dist_I.publish(adj_msg);
//   std::cout<<__func__<<" array 3"<<std::endl;  
  
  distance_err_I = -(desired_distance_ - adj_mean);
  
  
  
// // // //   if(distance_err_I>-3 && distance_err_I<=4){N_inc--;} //-3 +5
// // // //   
// // // //   if(distance_err_I>5 && distance_err_I<=8){ //5 10
// // // //     //Nothing
// // // //   }
// // // //   
// // // //   if(distance_err_I>8){N_inc++;} //10
// // // //         
// // // //   if((v_des_out==0) || (distance_err_I<=0) || (GI<=0)){N_inc=0;}
// // // //   
// // // //   inc_I=N_inc*GI;//This should be multiplied by the sampling time of the node distance 
// // // //       
// // // //      I=inc_I; 
     
     if(distance_err_I>30){
       I=I;
    }else{
      if(distance_err_I<-5){
// 	std::cout<<" My I is zero"<<std::endl;
	I=0;
    }else{
 I = I+GI * distance_err_I * (1 / rateHZ);    
    }
    }
    
    if(GI==0){I=0;}
      
    
    v_des = P + I + D+V0;
   
        if(v_des>v_des_max){ 
          v_des_out=v_des_max;  //v_des=v_des_max;
          }else if(v_des<=v_des_min){ 
        v_des_out=v_des_min; //v_des=v_des_min;
	I=0;
          }else{v_des_out=v_des;}   
          
          if(distance_err<=-6.5){v_des_out=0;}
          
          if(v_des_out<=v_des_min){
	    pub_err_tilde.publish(err_data);
	  }
	  pub_err_tilde.publish(err_data);
          
// std::cout<<"V0 "<<V0<<" Vout "<<v_des_out<<std::endl;
//------------------------------------ANTI WIND UP--------------------------------         
	  
	  
          anti_wu=k_wu*(v_des_out-v_des);  
    

//     v_des_out=0.07;
//------------------------------------PUBLISH DESIRED SPEED-----------------------
	if (v_des_out==v_des_min) {flag_err_dist=0;}else{flag_err_dist=1;} 
	
	
// 	v_des_out_q=v_des_out % quant;
	if(activate_quant_flag){
	  
	v_des_out_r=fmod(v_des_out,quant);
	if (v_des_out_r >=quant/2){
	  v_des_out=(v_des_out-v_des_out_r)+quant;}
	else{
	  v_des_out=v_des_out-v_des_out_r;
	}
  }
	
    v_pub.data = v_des_out;
    
    pub_v_des.publish(v_pub);

//     std::cout <<" error: "<<distance_err<< " v_des: "<< v_des_out << " N: "<< N <<std::endl;
    
    all_data_msg.des_speed=v_des_out;
    all_data_msg.dist_des=desired_distance_;
    all_data_msg.distance=adj_estimated_current_distance;
    all_data_msg.dot_error=err_tilde;
    
    pub_all_data_echo.publish(all_data_msg);
    ros::spinOnce();
    r.sleep();
    
        

// As function of the error we have to modify the bias of trunk and ankle due to the friction between the robot and the support structure
// hence we have to publish a specific message to a new topic. Here we will set the publisher and in the synergies control file we will set the subscriber which
// will update the bias.
//     The pubishing rate is 500Hz, i.e. Ts=0.002. I want to compare the distance every 0.02 seconds.
    

      
    
    
//     if(index_distance==0){
//        err_tilde=v_des_out;
//       index_distance++;
//     }
//        
//     if(index_distance<10 && index_distance>0){
//       index_distance+=1; //count
// //       std::cout<<"index "<<index_distance<<std::endl;
//           }
//      else{
//       err_tilde-=v_des_out;
//       err_data.data=err_tilde;
// //       std::cout<<" publish "<<std::endl;
// 	pub_err_tilde.publish(err_data);
// 	index_distance=0; //reset index
// // 	std::cout<<"published err tilde "<<std::endl;
//     }
//     
          
//            std::cout<<"index "<<index_distance<<std::endl;

    
    
        
  }// end while()
  
return 0;
}
