#include <ros/ros.h>
#include <ros/rate.h>
#include <math.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <qb_legs_controller/qbmove_communications.h>

#define STIFFNESS 0.4

std::vector<double> stiffness;

#define HAND_CLOSURE 18000

std::map<int, std::vector<short int>> cube_pos;
std::map<int, short int> hand_pos;
ros::Publisher pub_qb_legs ;
std::vector<double> stiffness_val;

geometry_msgs::Pose curr_pos;



// 	std::vector<int> cube_id = {1,2,3,4,5,6};
	std::vector<int> cube_id = {4};
	int n_cube(cube_id.size());



double tick2rad(short int meas)
{
	return (double(meas)) * (2.0 * M_PI) / 32768.0;
}

short int compute_cube_motor_position(double eq, double stiff, short int& th1, short int& th2) //set cube pos that has to be sent later to move the cube
{
	th1 = (short int)((eq + stiff) * 32768.0 / (2* M_PI));
	th2 = (short int)((eq - stiff) * 32768.0 / (2* M_PI));
// 	std::cout<<"m1 "<<th1<<", m2 "<<th2<<std::endl;
	
}

void qb_legs_callback(const geometry_msgs::Pose& msg) 
{
  
//     std::cout<<" qb_legs_callback "<<std::endl;
    //Stiffness is in degrees 'cause i prefer them however we have to use rad
  for(int i=0;i<6;i++){
    stiffness_val[i]=stiffness[i]*M_PI/180;
//     std::cout<<" "<<stiffness[i]<<std::endl;
    
  }
  
  curr_pos.position.x=msg.position.x;
  curr_pos.position.y=msg.position.y;
  curr_pos.position.z=msg.position.z;
  curr_pos.orientation.x=msg.orientation.x;
  curr_pos.orientation.y=msg.orientation.y;
  curr_pos.orientation.z=msg.orientation.z;
  
    compute_cube_motor_position(msg.position.x,stiffness_val[0],cube_pos.at(1).at(0),cube_pos.at(1).at(1));
    compute_cube_motor_position(msg.position.y,stiffness_val[1],cube_pos.at(2).at(0),cube_pos.at(2).at(1));
    compute_cube_motor_position(msg.position.z,stiffness_val[2],cube_pos.at(3).at(0),cube_pos.at(3).at(1));
    compute_cube_motor_position(msg.orientation.x,stiffness_val[3],cube_pos.at(4).at(0),cube_pos.at(4).at(1));
    compute_cube_motor_position(msg.orientation.y,stiffness_val[4],cube_pos.at(5).at(0),cube_pos.at(5).at(1));
    compute_cube_motor_position(msg.orientation.z,stiffness_val[5],cube_pos.at(6).at(0),cube_pos.at(6).at(1));
    
// std::cout<<" "<<curr_pos.position.x<<" "<<curr_pos.position.y<<" "<<curr_pos.position.z<<" "<<curr_pos.orientation.x<<" "<<curr_pos.orientation.y<<" "<<curr_pos.orientation.z;
//     std::cout<<std::endl;
}

void stiffness_callback(const geometry_msgs::Pose& msg)
{
  std::cout<<" stiffness callback "<<std::endl;
 stiffness[0]=msg.position.x;
 stiffness[1]=msg.position.y;
 stiffness[2]=msg.position.z;
 stiffness[3]=msg.orientation.x;
 stiffness[4]=msg.orientation.y;
 stiffness[5]=msg.orientation.z;
 
 pub_qb_legs.publish(curr_pos);
 
}

void activate(comm_settings* cs, int id)
{
	commActivate(cs, id, 1);
	usleep(1000);
}

void deactivate(comm_settings* cs, int id)
{
	commActivate(cs, id, 0);
	usleep(1000); //1 millisecond
}

int main(int argc, char **argv)
{

	stiffness.resize(6);
	stiffness_val.resize(6);
// 	stiffness={30,30,30,30,30,30};
	stiffness={0,0,0,0,0,0};
	  for(int i=0;i<6;i++){
    stiffness_val[i]=stiffness[i]*M_PI/180;
//     std::cout<<" "<<stiffness[i]<<std::endl;
    
  }
  
	ros::init(argc, argv, "qb_legs_manager");
  	ros::NodeHandle n_;

	
	
  	double rateHZ = 250; //0.004 
	short int inputs[2];
	short int measurements[3];
	short int current_measurements[2];
	
	bool flag_retrieve_currents=true;
	
	ros::Subscriber sub_qb_legs = n_.subscribe("/qb_legs_trajectories",1,&qb_legs_callback);
	 pub_qb_legs = n_.advertise<geometry_msgs::Pose>("/qb_legs_trajectories",10);
	ros::Subscriber sub_qb_stiffness=n_.subscribe("/qb_legs_stiffness",1,&stiffness_callback);
	ros::Publisher  pub_measure_qb_legs = n_.advertise<geometry_msgs::Pose>("/qb_legs_measures", 10);
	ros::Publisher pub_current_qb_legs = n_.advertise<std_msgs::Float64MultiArray>("/qb_legs_currents",10);
	
	geometry_msgs::Pose qb_legs_msg;
	std_msgs::Float64MultiArray qb_legs_curr_msg;
	


	comm_settings comm_settings_t;

	openRS485(&comm_settings_t, "/dev/ttyUSB0", 2000000);
// 	openRS485(&comm_settings_t, "/dev/ttyUSB0", 460800);
	usleep(10000); // 0.01 sec


	
	qb_legs_curr_msg.data.resize(2*cube_id.size());
	
// 	%%%%QUI
// 	std::vector<int> cube_id = {1};

	std::cout<<std::endl;
	std::cout<<"QbLegs Manager"<<std::endl;
	std::cout<<" - Cubes ("<<cube_id.size()<<") = ";

	for(auto id:cube_id)
	{
		cube_pos[id] = {0,0};
		activate(&comm_settings_t,id);
		std::cout<<id<<", ";
	}
	std::cout<<std::endl<<std::endl;
	
  	ros::Rate r(rateHZ);
	
	 pub_qb_legs.publish(curr_pos);
	 int count=0;

	while(ros::ok())
	{
		for(auto id:cube_id)
		{
			inputs[0] = cube_pos[id].at(0);
			inputs[1] = cube_pos[id].at(1);

			commSetInputs(&comm_settings_t, id, inputs); //here set motor positions 
			usleep(100);
			commGetMeasurements(&comm_settings_t, id, measurements); //here get motor measurements
			usleep(100);
			if(id == 1) qb_legs_msg.position.x = tick2rad(measurements[2]);
			if(id == 2) qb_legs_msg.position.y = tick2rad(measurements[2]); 
			if(id == 3) qb_legs_msg.position.z = tick2rad(measurements[2]); 
			if(id == 4) qb_legs_msg.orientation.x = tick2rad(measurements[2]); 
			if(id == 5) qb_legs_msg.orientation.y = tick2rad(measurements[2]);
			if(id == 6) qb_legs_msg.orientation.z = tick2rad(measurements[2]);
			
			if(flag_retrieve_currents){
			  commGetCurrents(&comm_settings_t, id, current_measurements); //here get current motor measurements
			  usleep(100);
			  qb_legs_curr_msg.data.at(count++) = current_measurements[0];
			  qb_legs_curr_msg.data.at(count++) = current_measurements[1];
			}

		}
// 		std::cout<<" meas "<<tick2rad(measurements[0])<<" "<<tick2rad(measurements[1])<<" "<<tick2rad(measurements[2])<<std::endl;
		pub_measure_qb_legs.publish(qb_legs_msg);
		pub_current_qb_legs.publish(qb_legs_curr_msg);

		r.sleep();
		ros::spinOnce();
	}

	for(auto id:cube_id)
		deactivate(&comm_settings_t,id);

	closeRS485(&comm_settings_t);
	usleep(1000);

	std::cout<<" Adios Amigos "<<std::endl;

}