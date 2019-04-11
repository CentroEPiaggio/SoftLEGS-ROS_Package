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
#include <math.h>
#include <phidgets_interface_kit/AnalogArray.h>

# define EV 0.57
# define l 0.3
# define g 9.81
# define Ts 0.005 //the publish step time
# define V0 0.0
# define N_s_max 100        //max number of samples of the second filter
# define N_s_min 1          //min sample number
# define WS_min 1e-3	//min speed value

Eigen::VectorXd sensor_IR_, array_sensor_pre_; 
// TO NOTE: sensor_IR_(0) it means that i'm working with the measurement provided by the sensor 
// connected to the first channel CHECK THIS IF YOU ARE NOT READING SOMETHING


//desired distance
double desired_distance_ = 50.0;                             

// to check if it is the first time that you take the measurement
bool flag_first_meas_ = false;
// PID gains
double kp_, ki_, kd_;  
// Initial value
double sensor_mean_distance_ = 0.0;                     




//Sensor measurement callback, first filter
void callback_sensor_IR(const phidgets_interface_kit::AnalogArray::ConstPtr& msg)
{
    // check NAN
    if(msg->values.size()<8) return;


     if(!isnan(msg->values[0]))
    { //Convert to cm
      sensor_IR_(0) = fabs(9462.0 / (msg->values[0] - 16.92)); 
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

}

//callback update PID
void callback_PID(const geometry_msgs::Vector3::ConstPtr& msg)
{
  kp_ = msg->x;
  ki_ = msg->y;
  kd_ = msg->z;
  
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
    N_s = round(T / Ts);

    if(N_s < N_s_min) N_s = N_s_min;
    if(N_s > N_s_max) N_s = N_s_max;

  }
  else
  {
    N_s = N_s_max;
  }
  
  return N_s;
  
}

//-----------------------To adjust measurement---------------

      double num_cond=8;
      std::vector<double> cond={20,30,40,50,60,70,80,90};
      std::vector<double> adj={0,3,5,5,6,7,8,9};
      double adj_val(0);
      double adj_estimated_current_distance;
      double acc_des;
      double k_wu;
      double anti_wu;


      
//       cond[1]=20;
//       
//       for(int cond_i=1;cond_i<num_cond;cond_i++){
// 	cond[cond_i]=cond[cond_i-1]+10;
//       }
//       
//       adj[0]=0;
//       adj[1]=3;
//       adj[2]=5;
//       adj[3]=5;
//       adj[4]=6;
//       adj[5]=7;
//       adj[6]=8;
//       adj[7]=9;

//-------------------------MAIN---------------------

int main(int argc, char **argv)
{
  double rateHZ = 100;        //frequenza nodo
  
  // first desired speed
  double v_des = 0.0;
  double v_des_out=0.0;
  double v_des_min=1e-4;
  double v_des_max=0.4;
  // first value
  double estimated_current_distance = 0.0;   //sensore filtrato nel main, quindi l'ultimo filtraggio
  // first gains
  double P = 0.0;
  double I = 0.0;
  double D = 0.0;
  // errors
  double distance_err = 0.0;
  double distance_err_old = 0.0;
  //Window size and previous one
  int N, N_old;
  
  // array_sensor is the array composed of sensor data
  Eigen::VectorXd array_sensor, tmp;
  // here I put the velocity reference that have to be published
  std_msgs::Float32 v_pub;

  // I use a PID to calculate the desired speed (maybe acceleration in future) as function of the error on position
  kp_ = 0.010;
  ki_ = 0.000;
  kd_ = 0.0;
//   k_wu=50;
  anti_wu=0;

  //initialize the vector where the phidgets put the measurements
  sensor_IR_ = Eigen::VectorXd::Zero(8);
  //array pre filtering process
  array_sensor_pre_= Eigen::VectorXd::Zero(10);
  //array post filtering process
  array_sensor = Eigen::VectorXd::Zero(N_s_min);
  
  N = array_sensor.size();
  N_old = array_sensor.size();

 

  ros::init(argc, argv, "target_tracking");
  ros::NodeHandle n_;

  ros::Rate r(rateHZ);

  //Topic you want to pub
  ros::Publisher  pub_v_des = n_.advertise<std_msgs::Float32>("synergies_des_speed", 100);

  //Topics to subscribe to
  ros::Subscriber sub_sensor_IR, sub_d_des, sub_PID;
  sub_sensor_IR = n_.subscribe("/cl4_gpio/analog_in", 10, &callback_sensor_IR);
  sub_d_des = n_.subscribe("/d_des", 10, &callback_d_des);
  sub_PID = n_.subscribe("/PID", 10, &callback_PID);


bool N_const=true;

  while(ros::ok())
  {
    
    if(N_const){
     if(flag_first_meas_){
      N=N_s_max;
            for(int i=0;i<array_sensor.size();i++){
      std::cout<< array_sensor(i)<<" ";
      }
      std::cout<<std::endl;
      
      array_sensor << array_sensor.tail(array_sensor.size() - 1),sensor_mean_distance_;
      array_sensor.conservativeResize(N);
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


      estimated_current_distance = array_sensor.mean();
      
      std::cout<<"mean 1 : " << array_sensor.mean()<< " mean 2 : "<<  array_sensor.sum()/N<<std::endl;
      N_old = N;
      

      //--------------------------------ADJUST MEASUREMENT-------------------------------
      for(int cond_i=0;cond_i<num_cond;cond_i++){
	      if(cond_i==0){adj_val=0;}
	      else if(estimated_current_distance<=cond[cond_i]/2 && estimated_current_distance>cond[cond_i-1]){
	adj_val=adj[cond_i-1]/2;
	      }else if(estimated_current_distance<=cond[cond_i] && estimated_current_distance>cond[cond_i-1]/2){
	adj_val=adj[cond_i-1];
	      }
      }//end for
      
      
      
    adj_estimated_current_distance=estimated_current_distance+adj_val;
   
    
    //////////PID///////////////////////////////////////////////////////////////////////
    distance_err = -(desired_distance_ - adj_estimated_current_distance);
    P = kp_ * distance_err;
    I += ki_ * (distance_err + anti_wu*1) * (1 / rateHZ);
    D = kd_ * (distance_err - distance_err_old) * rateHZ;

    distance_err_old = distance_err;

    v_des = P + I + D + V0;
//     acc_des=   P + I + D + V0;
//     v_des-= (acc_des + anti_wu) * (1 / rateHZ);
    
//     if(v_des>v_des_max){ 
//       v_des_out=v_des_max;  acc_des=0;
//       }else if(v_des<v_des_min){ 
//     v_des_out=v_des_min; acc_des=0;
//       }else{v_des_out=v_des; acc_des = P + I + D + V0;}
      
//       v_des_out+= acc_des * (1 / rateHZ);
    
        if(v_des>v_des_max){ 
          v_des_out=v_des_max;  //v_des=v_des_max;
          }else if(v_des<v_des_min){ 
        v_des_out=v_des_min; //v_des=v_des_min;
          }else{v_des_out=v_des;}   //anti wind up  
// 
//          
	  k_wu=50;
          anti_wu=k_wu*(v_des_out-v_des);  
    
    
    
    v_pub.data = v_des_out;
    pub_v_des.publish(v_pub);

//     std::cout <<" distance from sensor: "<< sensor_mean_distance_ <<" filtered distance : "<< estimated_current_distance<< " Adjusted distance : " <<adj_estimated_current_distance<<std::endl<< " v_des: "<< v_des_out << " N: "<< N <<std::endl;

       
    std::cout <<" error: "<<distance_err<< " v_des: "<< v_des_out << " N: "<< N <<std::endl;
    ros::spinOnce();
    r.sleep();
        
  }// end while()
  
return 0;
}
