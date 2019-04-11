// Distance Sensor header

// #include <ros/ros.h>
// #include <ros/rate.h>
// #include <eigen3/Eigen/Eigen>
#include <control_msgs/JointTrajectoryControllerState.h>
// #include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
// #include <math.h>
#include <phidgets_interface_kit/AnalogArray.h>

// # define exp_val 0.57
// # define l 0.3
// # define g 9.81
// # define Ts 0.01
// # define V0 0.0
// # define N_s_max 100        //numero campioni max (nel filtro del main)
// # define N_s_min 1          //numero campioni min (nel filtro del main)
// # define WS_min 1e-3






Eigen::VectorXd sensor_IR_, arr_sensor_pre_;      
double d_des;                             //distanza desiderata, finchè nessuno la setta dall'esterno
bool flag_first_meas_ = false;
double kp_, ki_, kd_;                             //guadagni del PID settabili esternamente nel topic /PID
double sensor_pre_filt = 0.0;                     //sensore prefiltrato nella callback

double P,I,D;


//calback lettura sensore e prefiltraggio (finestra mobile su 10 samples)
void callback_sensor_IR(const phidgets_interface_kit::AnalogArray::ConstPtr& msg)
{

   
    // controlla NAN
    if(msg->values.size()<8) return;


     if(!isnan(msg->values[0]))
    {
      sensor_IR_(0) = fabs(9462.0 / (msg->values[0] - 16.92));     // distanza in cm
      if(sensor_IR_(0) < 20.0) sensor_IR_(0) = 20.0;
      if(sensor_IR_(0) > 100.0) sensor_IR_(0) = 100.0;

      arr_sensor_pre_ << arr_sensor_pre_.tail(arr_sensor_pre_.size() - 1), sensor_IR_(0);
      sensor_pre_filt = arr_sensor_pre_.mean();
      // std::cout<<"sensore prefiltrato (callback): "<<sensor_pre_filt <<std::endl;
    }
    if(!flag_first_meas_)
    {
      // d_des_ = sensor_IR_(0);
      flag_first_meas_ = true;
    }

   
    // std::cout<<"sensore: "<<sensor_IR_(0) <<std::endl;


}

//calback lettura distanza desiderata
void callback_d_des(const std_msgs::Float32::ConstPtr& msg)
{
  d_des = msg->data;     // distanza in cm

}

//returns the number of step that we have to consider
int N_function(double WS)
{
  double N_s;

  if(WS > WS_min)
  {

    double WSN = WS / sqrt(g * l);
    double SLN = pow(WSN, exp_val);
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

double Calculate_des_speed(){
  double des_v;
      if(flag_first_meas_)
    {

      //////////Windows elements number////////////////////////////////////////////////
      int N = N_function(v_des);

      //////////Filter/////////////////////////////////////////////////////////////////
      arr_sensor.conservativeResize(N);
      
      if(N > N_old)
      {
        if((N - N_old) == 1)
        {
          arr_sensor << arr_sensor.head(N_old), sensor_IR_(0);
        }
        else
        {
          tmp = Eigen::VectorXd::Ones(N - N_old - 1);
          arr_sensor << arr_sensor.head(N_old), (tmp * sensor_filt),sensor_IR_(0);
        }
      }
      else
      {
        arr_sensor << arr_sensor.tail(arr_sensor.size() - 1), sensor_IR_(0);
        
      }

      sensor_filt = arr_sensor.mean();
      N_old = N;

    }
    

    //////////PID///////////////////////////////////////////////////////////////////////
    e = d_des_ - sensor_filt;
    P = kp_ * e;
    I += ki_ * e * (1 / rateHZ);
    D = kd_ * (e - e_old) * rateHZ;

    e_old = e;

    v_des = P + I + D + V0;
  
  
  return des_v;
}


// //calback guadagni PID
// void callback_PID(const geometry_msgs::Vector3::ConstPtr& msg)
// {
//   kp_ = msg->x;
//   ki_ = msg->y;
//   kd_ = msg->z;
//   
// }
// 
// // data la velocità desiderata calcola il numero di campioni utilizzati nel filtro della distanza interna al main
// int N_function(double WS)
// {
//   double N_s;
// 
//   if(WS > WS_min)
//   {
// 
//     double WSN = WS / sqrt(g * l);
//     double SLN = pow(WSN, exp_val);
//     double SL = SLN * l;
//     double T = SL / WS;
//     N_s = round(T / Ts);
// 
//     if(N_s < N_s_min) N_s = N_s_min;
//     if(N_s > N_s_max) N_s = N_s_max;
// 
//   }
//   else
//   {
//     N_s = N_s_max;
//   }
// 
// 
//   
// 
//   return N_s;
//   
// }
// 
// //-----------------------------------------------------
// //                                                 main
// //-----------------------------------------------------
// int main(int argc, char **argv)
// {
//   double rateHZ = 100;        //frequenza nodo
//   double v_des = 0.0;
//   double sensor_filt = 0.0;   //sensore filtrato nel main, quindi l'ultimo filtraggio
//   double P = 0.0;
//   double I = 0.0;
//   double D = 0.0;
//   double e = 0.0;
//   double e_old = 0.0;
// 
// 
// 
// 
// 
// 
//   int N, N_old;
//   Eigen::VectorXd arr_sensor, tmp;
//   std_msgs::Float32 v_pub;
// 
//   kp_ = 1.0;
//   ki_ = 0.0;
//   kd_ = 0.0;
// 
// 
//   sensor_IR_ = Eigen::VectorXd::Zero(8);
//   arr_sensor_pre_= Eigen::VectorXd::Zero(10);
//   arr_sensor = Eigen::VectorXd::Zero(N_s_min);
//   
//   N = arr_sensor.size();
//   N_old = arr_sensor.size();
// 
//  
// 
//   ros::init(argc, argv, "target_tracking");
//   ros::NodeHandle n_;
// 
//   ros::Rate r(rateHZ);
//   ros::Subscriber sub_sensor_IR, sub_d_des, sub_PID;
// 
//   //Topic you want to pub
//   ros::Publisher  pub_v_des = n_.advertise<std_msgs::Float32>("des_speed", 100);
// 
//   //Topic you want to subscribe
//   sub_sensor_IR = n_.subscribe("/cl4_gpio/analog_in", 10, &callback_sensor_IR);
//   sub_d_des = n_.subscribe("/d_des", 10, &callback_d_des);
//   sub_PID = n_.subscribe("/PID", 10, &callback_PID);
// 
// 
// 
// 
//   while(ros::ok())
//   {
//     if(flag_first_meas_)
//     {
// 
//       //////////Windows elements number////////////////////////////////////////////////
//       N = N_function(v_des);
// 
//       //////////Filter/////////////////////////////////////////////////////////////////
//       arr_sensor.conservativeResize(N);
//       
//       if(N > N_old)
//       {
//         if((N - N_old) == 1)
//         {
//           arr_sensor << arr_sensor.head(N_old), sensor_IR_(0);
//         }
//         else
//         {
//           tmp = Eigen::VectorXd::Ones(N - N_old - 1);
//           arr_sensor << arr_sensor.head(N_old), (tmp * sensor_filt),sensor_IR_(0);
//         }
//       }
//       else
//       {
//         arr_sensor << arr_sensor.tail(arr_sensor.size() - 1), sensor_IR_(0);
//         
//       }
// 
//       sensor_filt = arr_sensor.mean();
//       N_old = N;
// 
//     }
//     
// 
//     //////////PID///////////////////////////////////////////////////////////////////////
//     e = d_des_ - sensor_filt;
//     P = kp_ * e;
//     I += ki_ * e * (1 / rateHZ);
//     D = kd_ * (e - e_old) * rateHZ;
// 
//     e_old = e;
// 
//     v_des = P + I + D + V0;
//     v_pub.data = v_des;
//     pub_v_des.publish(v_pub);
// 
//     std::cout <<" distanza pre: "<< sensor_pre_filt <<" distanza : "<< sensor_filt << " v_des: "<< v_des << " N: "<< N <<std::endl;
// 
//     
//     ros::spinOnce();
//     r.sleep();
//         
//   }// end while()
// return 0;
// }