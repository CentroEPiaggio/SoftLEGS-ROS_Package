#include <iostream>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <map>

std::map<int,trajectory_msgs::JointTrajectory> msgs;

void left_chain_values_callback(const geometry_msgs::Pose& msg)
{
    msgs.at(1).points.at(0).positions.at(0) = msg.position.x;
    msgs.at(2).points.at(0).positions.at(0) = msg.position.y;
    msgs.at(3).points.at(0).positions.at(0) = msg.position.z;
    msgs.at(4).points.at(0).positions.at(0) = msg.orientation.x;
    msgs.at(5).points.at(0).positions.at(0) = msg.orientation.y;
}

void right_chain_values_callback(const geometry_msgs::Pose& msg)
{
    msgs.at(11).points.at(0).positions.at(0) = msg.position.x;
    msgs.at(12).points.at(0).positions.at(0) = msg.position.y;
    msgs.at(13).points.at(0).positions.at(0) = msg.position.z;
    msgs.at(14).points.at(0).positions.at(0) = msg.orientation.x;
    msgs.at(15).points.at(0).positions.at(0) = msg.orientation.y;
}

void gaze_callback(const geometry_msgs::Vector3& msg)
{
    msgs.at(21).points.at(0).positions.at(0) = msg.z;
    msgs.at(22).points.at(0).positions.at(0) = msg.y - 0.3;
}

int main(int argc, char** argv)
{
    if(!ros::isInitialized()) ros::init(argc,argv,"qb_frank_controller");
    
    ros::NodeHandle nh;
    
    std::vector<int> ids = {1,2,3,4,5,11,12,13,14,15,21,22};
    
    std::map<int,ros::Publisher> pubs;

    for(auto id:ids)
    {
        std::string topic = "/frank/cube" + std::to_string(id) + "/cube" + std::to_string(id) + "_position_and_preset_trajectory_controller/command";
	pubs[id] = nh.advertise<trajectory_msgs::JointTrajectory>(topic,10);
	
	trajectory_msgs::JointTrajectoryPoint point;
	point.time_from_start.sec=0;
        point.time_from_start.nsec=10000000;
	point.positions.push_back(0.0);
	point.positions.push_back(0.87);
	
	trajectory_msgs::JointTrajectory traj;
	std::string name_pos = "cube" + std::to_string(id) + "_shaft_joint";
	std::string name_stiff = "cube" + std::to_string(id) + "_stiffness_preset_virtual_joint";
	traj.joint_names.push_back(name_pos);
	traj.joint_names.push_back(name_stiff);
	traj.points.push_back(point);

	msgs[id] = traj;
    }
    
    ros::Rate loop(100);
    
    ros::Subscriber subl = nh.subscribe("/frank_q_des_left",1,&left_chain_values_callback);
    ros::Subscriber subr = nh.subscribe("/frank_q_des_right",1,&right_chain_values_callback);
    ros::Subscriber subg = nh.subscribe("/gaze",1,&gaze_callback);
    
    while(ros::ok())
    {
	for(auto id:ids)
	{
	   pubs.at(id).publish(msgs.at(id));
	}

	ros::spinOnce();
	loop.sleep();
    }

    return 0;
}
