// Service to modify sim 
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <iterator>

double Des_Optimal_sim;
double Curr_Optimal_sim;
bool Warm_Start=false;
bool Warm_Stop=true;
std::vector<double> bias={-11,0,25,-3,0,18};
std::vector<double> old_bias(6);

bool setSimCallback(zmp_walk::num_sim_srv::Request& request, zmp_walk::num_sim_srv::Response& response)
{
   ROS_INFO("Desired Sim : %lf ", request.num_sim);
   Des_Optimal_sim=request.num_sim;
   ROS_INFO("Desired Sim : %lf ", request.num_sim);
 
   return true;
}

void DesOptCallback(const std_msgs::Float64::ConstPtr& msg){
   
   Des_Optimal_sim=msg->data;
   ROS_INFO("Desired Sim : %lf ", Des_Optimal_sim);
}

bool setSimStartCallback(zmp_walk::sim_start_srv::Request& request, zmp_walk::sim_start_srv::Response& response)
{
   ROS_INFO("Warm_Start");
   Warm_Start=true;
   
 
   return true;
}

void WarmStartCallback(const std_msgs::Float64::ConstPtr& msg){
     ROS_INFO("Warm_Start");
   Warm_Start=true;
   Warm_Stop=false;
 
   return;
}

void WarmStopCallback(const std_msgs::Float64::ConstPtr& msg){
     ROS_INFO("Warm_Stop");
   Warm_Stop=true;
   Warm_Start=false;
 
   return;
}

bool setSimStopCallback(zmp_walk::sim_stop_srv::Request& request,zmp_walk::sim_stop_srv::Response& response)
{
   ROS_INFO("Warm_Stop");
   Warm_Stop=request.Wsto;

   return true;
}


void BiasCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
   std::cout<<"Current bias = ";
   for(int i=0;i<6;i++){
     std::cout<< bias[i]<<", ";
  }
  std::cout<<std::endl;
  
    std::cout<<"New bias = ";
   for(int i=0;i<6;i++){
     bias[i]=msg->data[i];
     old_bias[i]=bias[i];
     std::cout<< bias[i]<<", ";
  }
  std::cout<<std::endl; 
  
return;
}

//-----------------------------------------------------------------------

Eigen::MatrixXd Interpolate_and_Resample(Eigen::MatrixXd Trajectory, double Traj_Ts, double Pub_Ts){
      
  int Traj_num=Trajectory.rows();// this is 60x6
  int n_joints=Trajectory.cols();
  
//   std::cout<<__func__<<" "<<Traj_num<<" "<<n_joints<<std::endl;
  
  int pub_num=std::round(Traj_num*Traj_Ts/Pub_Ts);
//   std::cout<<"pn "<< pub_num<<" "<<Traj_Ts/Pub_Ts<<std::endl;
  
//   std::cout<<"Traj_dim "<<Traj_num<<" n_joints "<<n_joints<<" pub_num "<<pub_num<<std::endl;
  std::vector<double> X_traj(Traj_num),X_pub(pub_num),Traj_pub(pub_num), Traj(Traj_num);
  
  Eigen::MatrixXd out_Traj(pub_num,n_joints);
    
//     The time vectors old and new
	  	  
	  for(int i=0;i<Traj_num;i++){
	    X_traj[i]=(i+1)*Traj_Ts; // curr_Ts is the sample time of the trajectory that we want to implement
	  }
	  
	  for(int i=0;i<pub_num;i++){
	    X_pub[i]=(i+1)*Pub_Ts; // curr_Ts is the sample time of the trajectory that we want to implement
	  }
	  
	  tk::spline s;
	  for(int i=0;i<n_joints;i++){
	    
	    for(int j=0;j<Traj_num;j++){
	      Traj[j]=Trajectory(j,i);
	    }
	      
	    s.set_points(X_traj,Traj);
	    
	    for(int j=0;j<pub_num;j++){
	    out_Traj(j,i)=s(X_pub[j]);
	    }
	    
	  }
	  
return out_Traj;
    }

   
Eigen::MatrixXd Interpolate_and_Resample_Middle(Eigen::MatrixXd New_Traj, double Ts_new, Eigen::MatrixXd Old_Traj, double Ts_old, int Instant, double Pub_Ts){
     
     int Traj_num=Old_Traj.rows();// this is 60x6 and it is the same of the new
     int n_joints=Old_Traj.cols();
     
     int pub_num=std::round(((Instant-1)*Ts_old+(Traj_num-Instant+1)*Ts_new)/Pub_Ts);
     // -1 'cause we switch the signals at i==Instant, i.e. i==Instant New_signals(i)
     
     std::vector<double> X_traj(Traj_num),X_pub(pub_num),Traj_pub(pub_num), Traj(Traj_num);
    
     Eigen::MatrixXd Middle_Traj_pub(pub_num,n_joints);
     
     for(int i=0;i<Traj_num;i++){
       if(i<Instant){
	    X_traj[i]=(i+1)*Ts_old; // curr_Ts is the sample time of the trajectory that we want to implement
       }else{
	    X_traj[i]=X_traj[Instant-1]+(i-Instant+1)*Ts_new;   
	  }
     }
//      for(int i=0;i<Traj_num;i++){
//      std::cout<<X_traj[i]<<std::endl;
//      }
	  
      for(int i=0;i<pub_num;i++){
	    X_pub[i]=(i+1)*Pub_Ts; // curr_Ts is the sample time of the trajectory that we want to implement
	  }
	  

      tk::spline s; 
      
      for(int i=0;i<n_joints;i++){
	
	for(int j=0;j<Traj_num;j++){
	  if(j<Instant){
	     Traj[j]=Old_Traj(j,i);
	  }else{
	     Traj[j]=New_Traj(j,i);  
	    }  
	}
	
	s.set_points(X_traj,Traj);
	
	for(int j=0;j<pub_num;j++){
	    Middle_Traj_pub(j,i)=s(X_pub[j]);
	    }
	    
	
      }
     
      
     return Middle_Traj_pub;
  }
    