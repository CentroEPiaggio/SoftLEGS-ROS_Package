#include <iostream>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <map>

int main(int argc, char** argv)
{
    if(!ros::isInitialized()) ros::init(argc,argv,"qb_frank_controller");
    
    ros::NodeHandle nh;
    
    std::vector<int> ids = {1,2,3,4,5,11,12,13,14,15,21,22};

    double des34 = 1.0;
    double des1 = 0.5;
    double des2 = 0.4;
    double des5 = -0.75;
    double des12 = -0.4;
    
    std::map<int,trajectory_msgs::JointTrajectory> msgs;
    std::map<int,ros::Publisher> pubs;
    std::map<int,double> values = {
    {1,des1},
    {2,des2},
    {3,des34},
    {4,des34},
    {5,des5},
    {11,des1},
    {12,des12},
    {13,des34},
    {14,des34},
    {15,des5},
    {21,des2},
    {22,des2}};

    for(auto id:ids)
    {
        std::string topic = "/frank/cube" + std::to_string(id) + "/cube" + std::to_string(id) + "_position_and_preset_trajectory_controller/command";
	pubs[id] = nh.advertise<trajectory_msgs::JointTrajectory>(topic,10);
	
	trajectory_msgs::JointTrajectoryPoint point;
	point.time_from_start.sec=1;
	point.positions.push_back(0.0);
	point.positions.push_back(0.5);
	
	trajectory_msgs::JointTrajectory traj;
	std::string name_pos = "cube" + std::to_string(id) + "_shaft_joint";
	std::string name_stiff = "cube" + std::to_string(id) + "_stiffness_preset_virtual_joint";
	traj.joint_names.push_back(name_pos);
	traj.joint_names.push_back(name_stiff);
	traj.points.push_back(point);

	msgs[id] = traj;
    }
    
    ros::Time start = ros::Time::now();
    ros::Time stop = ros::Time::now();
    ros::Rate loop(10);
    
    while(ros::ok())
    {
        stop = ros::Time::now();

	if(stop-start > ros::Duration(2.0))
	{
	    start = ros::Time::now();

	    values.at(1) = -values.at(1);
	    values.at(3) = -values.at(3);
	    values.at(4) = -values.at(4);
	    values.at(11) = -values.at(11);
	    values.at(13) = -values.at(13);
	    values.at(14) = -values.at(14);

	    for(auto id:ids)
	    {
		//HACK
		if(id==2 || id==12) msgs.at(id).points.at(0).positions.at(0) = values.at(id) - values.at(3)*0.4;
		else if(id==5 || id==15) msgs.at(id).points.at(0).positions.at(0) = values.at(id) - values.at(3)*0.75;
		else if(id==22) msgs.at(id).points.at(0).positions.at(0) = -0.5 - values.at(3)*0.5;
		else if(id==21) msgs.at(id).points.at(0).positions.at(0) = values.at(3)*1.0;
		else msgs.at(id).points.at(0).positions.at(0) = values.at(id);
	    }
	}

	for(auto id:ids)
	{
	   pubs.at(id).publish(msgs.at(id));
	}

	ros::spinOnce();
	loop.sleep();
    }

    return 0;
}