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

#include <trajectory_msgs/JointTrajectory.h>
#include <map>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>

#include <fstream>
#include <ctime>
#include <math.h>

#include <dirent.h>

// #include <optimal_walk/Optimal_Dataset_NO_old.h>      // old gianmaria
#include <optimal_walk/Optimal_Dataset_NO.h>       // new (working)
// #include <optimal_walk/Optimal_Dataset_NO_skip3.h> // new skip 3 (not working)
// #include <optimal_walk/Optimal_Dataset_NO_skip5.h> // new skip 5 (not working0)
#include <optimal_walk/num_sim_srv.h>
#include <optimal_walk/sim_start_srv.h>
#include <optimal_walk/sim_stop_srv.h>

#include <optimal_walk/spline.h>
#include <optimal_walk/Supporting_functions.h>
#include <optimal_walk/dataset_name.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "optimal_walk");
  ros::NodeHandle n;

  bool flag_pub_traj = true;
  bool flag_concatenate_started = false;

  ros::ServiceServer srv_set_Sim = n.advertiseService("optimal_walk/Set_Sim", &setSimCallback);
  ros::ServiceServer srv_sim_start = n.advertiseService("optimal_walk/Sim_Start", &setSimStartCallback);
  ros::ServiceServer srv_sim_stop = n.advertiseService("optimal_walk/Sim_Stop", &setSimStopCallback);

  ros::Subscriber sub_bias  = n.subscribe("optimal_walk/optimal_bias", 1, &BiasCallback);
  ros::Subscriber sub_start = n.subscribe("optimal_walk/start", 1, &WarmStartCallback);
  ros::Subscriber sub_stop  = n.subscribe("optimal_walk/stop", 1, &WarmStopCallback);
  ros::Publisher pub_traj = n.advertise<std_msgs::Float64MultiArray>("optimal_walk/trajectories", 10);
  ros::Publisher qb_legs_pub = n.advertise<geometry_msgs::Pose>("/qb_legs_trajectories", 10);
  ros::Publisher pub_opt_bias_echo = n.advertise<std_msgs::Float64MultiArray>("optimal_walk/bias_echo", 10);
  ros::Subscriber sub_des_opt = n.subscribe("optimal_walk/des_opt", 1, &DesOptCallback);

  ros::Subscriber sub_tele_des_opt = n.subscribe("/logitech_ctrl/cmd", 1, &TeleCallback);

  ros::Publisher pub_dataset_name = n.advertise<optimal_walk::dataset_name>("optimal_walk/dataset_name", 10);

  bool restart_again = true; //this works like an emergency stop and a fast restart.
  // if you go below the min speed your trajectories are set to 0. If you just decrease the
  // min speed to zero due to the stride law the step time increases too much. Hence if you reach
  // min speed trajectories goes to zero then you have to reastart with a warm start
  Warm_Stop = true;
  Warm_Start = false;

  bool first_time = true;

  Initialize_Optimal_Dataset();

  std::cout << " Sims " << std::endl;
  for (int i = 0; i <= sim_name.size() - 1; i++)
  {
    std::cout << sim_name[i] << std::endl;
  }

  optimal_walk::dataset_name opt_sims;
  opt_sims.name_list.resize(n_traj);
  for (int i = 0; i < n_traj; i++)
  {
    opt_sims.name_list[i] = std::to_string(i + 1) + " - " + sim_name[i];
    //     std::cout<<"sims "<<opt_sims.name_list[i]<<" "<<sim_name[i]<<std::endl;
  }

  double pub_Ts = 0.004;
  ros::Rate rate_while = 1 / pub_Ts;

  std::vector<int> ids = {1, 2, 3, 4, 5, 6};
  double qb_val(0);

  std::vector<double> traj_d = {0, 0, 0, 0, 0, 0};
  int Instant(0), Current_Instant(0);

  geometry_msgs::Pose qb_msg;

  Eigen::MatrixXd New_Traj, New_Traj_pub;
  Eigen::MatrixXd Old_Traj, Old_Traj_pub;
  Eigen::MatrixXd Middle_Traj_pub;
  Eigen::MatrixXd Output_Traj;
  //     Eigen::MatrixXd Err_Traj;
  int t_index(0);

  bool one_time = true;
  double interp_Instant = 1;
  bool flag_concatenate_completed = true;
  int pub_num;

  double Ni_start = std::round(200 * 6 * 2); //200 steps with Ts_pub, i.e. 1.2 seconds , *6 due to the code position see below
  double Ni_stop = std::round(200 * 6 * 2);  // was 1 //200 steps with Ts_pub, i.e. 1.2 seconds , *6 due to the code position see below
  double index_w_start = 1;
  double index_w_stop = 1;

  std_msgs::Float64MultiArray sim_traj_msg;
  double index_time(0);
  std_msgs::Float64MultiArray bias_msg_echo;
  bias_msg_echo.data.resize(6);

  while (ros::ok())
  {

    if (Des_Optimal_sim <= 0)
    {
      Des_Optimal_sim = 1;
    }
    if (Curr_Optimal_sim <= 0)
    {
      Curr_Optimal_sim = 1;
    }
    if ((std::round(Des_Optimal_sim)) > n_traj)
    {
      std::cout << "Sim " << Des_Optimal_sim << " does not exist, keep the previous one" << std::endl;
      Des_Optimal_sim = Curr_Optimal_sim;
    }

    if (first_time)
    {
      Des_Optimal_sim = Curr_Optimal_sim;

      std::cout << "Activation sim " << std::to_string(Des_Optimal_sim) << std::endl;
      New_Traj = *(traj_M.at(Des_Optimal_sim - 1));

      // 	std::cout<<" Traj rows "<<New_Traj.rows()<<" Traj cols "<<New_Traj.cols()<<std::endl;

      New_Traj_pub = Interpolate_and_Resample(New_Traj, Ts_traj, pub_Ts);
      Old_Traj_pub = New_Traj_pub;
      // 	std::cout<<" Traj rows "<<New_Traj_pub.rows()<<" Traj cols "<<New_Traj_pub.cols()<<std::endl;
      first_time = false;
      // 	std::cout<<"147"<<std::endl;
    } //end first time
    else
    {
      if (Curr_Optimal_sim != Des_Optimal_sim)
      {

        Warm_Stop = true;
        // 	      std::cout<<"154"<<std::endl;
        if (index_w_stop == Ni_stop && index_w_start == 1)
        {
          // 		std::cout<<"155"<<std::endl;
          New_Traj = *(traj_M.at(Des_Optimal_sim - 1));
          New_Traj_pub = Interpolate_and_Resample(New_Traj, Ts_traj, pub_Ts);
          Old_Traj_pub = New_Traj_pub;
          std::cout << "Sim modified from " << Curr_Optimal_sim << " to " << Des_Optimal_sim << std::endl;
          Curr_Optimal_sim = Des_Optimal_sim;
          Warm_Start = true;
          Warm_Stop = false;
          // 		std::cout<<"163"<<std::endl;
        }
      }
    } // end else if it is first time

    Output_Traj = New_Traj_pub;

    // 	    std::cout<<"170"<<std::endl;
    for (int i = 0; i < 6; i++)
    {

      //--------------------------------------Warm Start & Warm Stop---------------
      traj_d[i] = Output_Traj(t_index, i);

      if (Warm_Start && !Warm_Stop)
      {
        if (index_w_start >= Ni_start)
        {
          index_w_start = Ni_start;
          index_w_stop = 1;
        }
        else
        {
          index_w_start += 1;
        }
        // 		  std::cout<<"188"<<std::endl;
        traj_d[i] = traj_d[i] * index_w_start / Ni_start;
        //     std::cout<<__func__<<" 277 "<<std::endl;
      }
      else if (!Warm_Start && Warm_Stop)
      {

        if (index_w_stop >= Ni_stop)
        {
          // std::cout<<" end "<<std::endl;
          // traj_d[i]=0;
          // index_w=1;
          index_w_stop = Ni_stop;
          index_w_start = 1;
          // first_time=true; //modified 21/07/2017
          // flag_concatenate_started=false;
          // flag_concatenate_completed=true;
          // Warm_Start=false;
          // Warm_Stop=false;
        }
        else
        {
          index_w_stop += 1;
          // 		    std::cout<<((Ni-index_w)/Ni)<<std::endl;
        }

        traj_d[i] = traj_d[i] * (1 - index_w_stop / Ni_stop);
      }
      else if ((!Warm_Start && !Warm_Stop) || (Warm_Start && Warm_Stop))
      {
        Warm_Start = false;
        Warm_Stop = true;
        index_w_stop = 1;
        index_w_start = 1;
      }
    } //end for i

    // std::cout<<"226"<<std::endl;
    //---------------------------------------PUB----------------------------------

    //     std::cout<<std::to_string(New_Traj(0,0))<<std::endl;
    sim_traj_msg.data.resize(traj_d.size());
    for (int i = 0; i < 6; i++)
    {
      sim_traj_msg.data.at(i) = traj_d[i];
    }
    // 	sim_traj_msg.data=traj_d;
    pub_traj.publish(sim_traj_msg);

    //---------------------------------------PUB QB LEGS--------------------------
    index_time += pub_Ts;

    //     synergies::bias bias_msg_echo;
    //     std::cout<<__func__<<" 326 "<<std::endl;
    for (int i = 0; i < 6; i++)
    {
      bias_msg_echo.data.at(i) = bias[i];
    }
    //     std::cout<<__func__<<" 337 "<<std::endl;
    pub_opt_bias_echo.publish(bias_msg_echo);

    // std::cout<<"247"<<std::endl;
    //------------------

    // Here publish to qbs and to other topics

    for (auto id : ids)
    {

      qb_val = traj_d[id - 1] + (bias[id - 1]) * M_PI / 180;
      // 	bias_msg_echo.bias[id-1]=bias[id-1];

      if (id == 1 || id == 2 || id == 3)
      {
        qb_val *= -1;
      }

      if (id == 1)
      {
        qb_msg.position.x = qb_val;
      }
      if (id == 2)
      {
        qb_msg.position.y = qb_val;
      }
      if (id == 3)
      {
        qb_msg.position.z = qb_val;
      }
      if (id == 4)
      {
        qb_msg.orientation.x = qb_val;
      }
      if (id == 5)
      {
        qb_msg.orientation.y = qb_val;
      }
      if (id == 6)
      {
        qb_msg.orientation.z = qb_val;
      }

    } // end for pub

    qb_legs_pub.publish(qb_msg);

    // 	std::cout<<"273"<<std::endl;
    Current_Instant = t_index;
    // 	std::cout<<Current_Instant<<std::endl;
    if (t_index == Output_Traj.rows() - 1)
    {
      t_index = 0;
    }
    else
    {
      t_index += 1;
    }
    // 	std::cout<<"277"<<std::endl;

    pub_dataset_name.publish(opt_sims);
    ros::spinOnce();
    rate_while.sleep();

  } //end while

  return 1;
}
