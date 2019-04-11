#include <ros/ros.h>
#include <ros/rate.h>
#include <math.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <qb_frank_controller/qbmove_communications.h>

#define STIFFNESS 0.4
#define HAND_CLOSURE 18000

std::map<int, std::vector<short int>> cube_pos;
std::map<int, short int> hand_pos;

double tick2rad(short int meas)
{
	return (double(meas)) * (2.0 * M_PI) / 32768.0;
}

short int compute_cube_motor_position(double eq, double stiff, short int& th1, short int& th2)
{
	th1 = (short int)((eq + stiff) * 32768.0 / (2* M_PI));
	th2 = (short int)((eq - stiff) * 32768.0 / (2* M_PI));
}

short int compute_hand_motor_position(double eq, short int& th1)
{
	th1 = (short int)(eq*HAND_CLOSURE);
}

void left_chain_values_callback(const geometry_msgs::Pose& msg)
{
    compute_cube_motor_position(msg.position.x,STIFFNESS,cube_pos.at(1).at(0),cube_pos.at(1).at(1));
	compute_cube_motor_position(msg.position.y,STIFFNESS,cube_pos.at(2).at(0),cube_pos.at(2).at(1));
	compute_cube_motor_position(msg.position.z,STIFFNESS,cube_pos.at(3).at(0),cube_pos.at(3).at(1));
	compute_cube_motor_position(msg.orientation.x,STIFFNESS,cube_pos.at(4).at(0),cube_pos.at(4).at(1));
	compute_cube_motor_position(msg.orientation.y,STIFFNESS,cube_pos.at(5).at(0),cube_pos.at(5).at(1));
}

void right_chain_values_callback(const geometry_msgs::Pose& msg)
{
    compute_cube_motor_position(msg.position.x,STIFFNESS,cube_pos.at(11).at(0),cube_pos.at(11).at(1));
    compute_cube_motor_position(msg.position.y,STIFFNESS,cube_pos.at(12).at(0),cube_pos.at(12).at(1));
    compute_cube_motor_position(msg.position.z,STIFFNESS,cube_pos.at(13).at(0),cube_pos.at(13).at(1));
    compute_cube_motor_position(msg.orientation.x,STIFFNESS,cube_pos.at(14).at(0),cube_pos.at(14).at(1));
    compute_cube_motor_position(msg.orientation.y,STIFFNESS,cube_pos.at(15).at(0),cube_pos.at(15).at(1));
}

void gaze_callback(const geometry_msgs::Vector3& msg)
{
	compute_cube_motor_position(msg.z,STIFFNESS,cube_pos.at(21).at(0),cube_pos.at(21).at(1));
	compute_cube_motor_position(msg.y - 0.3,STIFFNESS,cube_pos.at(22).at(0),cube_pos.at(22).at(1));
}

void left_hand_callback(const std_msgs::Float64& msg)
{
	compute_hand_motor_position(msg.data,hand_pos.at(6));
}

void right_hand_callback(const std_msgs::Float64& msg)
{
	compute_hand_motor_position(msg.data,hand_pos.at(16));
}

void activate(comm_settings* cs, int id)
{
	commActivate(cs, id, 1);
	usleep(1000);
}

void deactivate(comm_settings* cs, int id)
{
	commActivate(cs, id, 0);
	usleep(1000);
}

//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{
	ros::init(argc, argv, "qb_manager");
  	ros::NodeHandle n_;

  	double rateHZ = 100;
	short int inputs[2];
	short int measurements[3];


	ros::Subscriber subl = n_.subscribe("/frank_q_des_left",1,&left_chain_values_callback);
    ros::Subscriber subr = n_.subscribe("/frank_q_des_right",1,&right_chain_values_callback);
    ros::Subscriber subg = n_.subscribe("/gaze",1,&gaze_callback);
    ros::Subscriber subrh = n_.subscribe("/right_hand/hand_remap/hand_position",1,&right_hand_callback);
    ros::Subscriber sublh = n_.subscribe("/left_hand/hand_remap/hand_position",1,&left_hand_callback);

    ros::Publisher  pub_measure_R = n_.advertise<geometry_msgs::Pose>("measure_R", 10);
    ros::Publisher  pub_measure_L = n_.advertise<geometry_msgs::Pose>("measure_L", 10);
    geometry_msgs::Pose msg_R, msg_L;


	comm_settings comm_settings_t;

	openRS485(&comm_settings_t, "/dev/ttyUSB0", 2000000);
	usleep(10000);

	std::vector<int> cube_id = {1,2,3,4,5,11,12,13,14,15,21,22};
	std::vector<int> hand_id = {6,16};

	std::cout<<std::endl;
	std::cout<<"QB Manager"<<std::endl;
	std::cout<<" - Cubes ("<<cube_id.size()<<") = ";

	for(auto id:cube_id)
	{
		cube_pos[id] = {0,0};
		activate(&comm_settings_t,id);
		std::cout<<id<<", ";
	}
	std::cout<<std::endl<<" - Hands ("<<hand_id.size()<<") = ";
	for(auto id:hand_id)
	{
		hand_pos[id]=0;
		activate(&comm_settings_t,id);
		std::cout<<id<<", ";
	}
	std::cout<<std::endl<<std::endl;
	
  	ros::Rate r(rateHZ);

	while(ros::ok())
	{
		for(auto id:cube_id)
		{
			inputs[0] = cube_pos[id].at(0);
			inputs[1] = cube_pos[id].at(1);

			commSetInputs(&comm_settings_t, id, inputs);
			usleep(100);
			commGetMeasurements(&comm_settings_t, id, measurements);
			usleep(100);
			if(id == 1) msg_L.position.x = tick2rad(measurements[2]);
			if(id == 2) msg_L.position.y = tick2rad(measurements[2]); 
			if(id == 3) msg_L.position.z = tick2rad(measurements[2]); 
			if(id == 4) msg_L.orientation.x = tick2rad(measurements[2]); 
			if(id == 5) msg_L.orientation.y = tick2rad(measurements[2]); 
			if(id == 11) msg_R.position.x = tick2rad(measurements[2]); 
			if(id == 12) msg_R.position.y = tick2rad(measurements[2]); 
			if(id == 13) msg_R.position.z = tick2rad(measurements[2]); 
			if(id == 14) msg_R.orientation.x = tick2rad(measurements[2]); 
			if(id == 15) msg_R.orientation.y = tick2rad(measurements[2]); 
			if(id == 21) msg_R.orientation.z = tick2rad(measurements[2]); 
			if(id == 22) msg_L.orientation.z = tick2rad(measurements[2]); 
			
		}
		pub_measure_R.publish(msg_R);
		pub_measure_L.publish(msg_L);


		for(auto id:hand_id)
		{
			inputs[0] = hand_pos[id];
			inputs[1] = 0;

			commSetInputs(&comm_settings_t, id, inputs);
			usleep(100);
			commGetMeasurements(&comm_settings_t, id, measurements);
			usleep(100);
		}

		r.sleep();
		ros::spinOnce();
	}

	for(auto id:cube_id)
		deactivate(&comm_settings_t,id);
	for(auto id:hand_id)
		deactivate(&comm_settings_t,id);

	closeRS485(&comm_settings_t);
	usleep(1000);

	std::cout<<"muoio"<<std::endl;

}