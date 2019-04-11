#include <iostream>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

int main(int argc, char** argv)
{
    if(!ros::isInitialized()) ros::init(argc,argv,"qb_frank_controller");
    
    ros::NodeHandle nh;
    
    trajectory_msgs::JointTrajectory msg1,msg2,msg3, msg4, msg5, msg6;
    ros::Publisher traj_pub_1 = nh.advertise<trajectory_msgs::JointTrajectory>("/frank_left/cube1/cube1_position_and_preset_trajectory_controller/command",10);
    ros::Publisher traj_pub_2 = nh.advertise<trajectory_msgs::JointTrajectory>("/frank_left/cube2/cube2_position_and_preset_trajectory_controller/command",10);
    ros::Publisher traj_pub_3 = nh.advertise<trajectory_msgs::JointTrajectory>("/frank_left/cube3/cube3_position_and_preset_trajectory_controller/command",10);
    ros::Publisher traj_pub_4 = nh.advertise<trajectory_msgs::JointTrajectory>("/frank_left/cube4/cube4_position_and_preset_trajectory_controller/command",10);
    ros::Publisher traj_pub_5 = nh.advertise<trajectory_msgs::JointTrajectory>("/frank_left/cube5/cube5_position_and_preset_trajectory_controller/command",10);
    ros::Publisher traj_pub_6 = nh.advertise<trajectory_msgs::JointTrajectory>("/frank_left/cube22/cube22_position_and_preset_trajectory_controller/command",10);

    msg1.joint_names.push_back("cube1_shaft_joint");
    msg1.joint_names.push_back("cube1_stiffness_preset_virtual_joint");
    msg2.joint_names.push_back("cube2_shaft_joint");
    msg2.joint_names.push_back("cube2_stiffness_preset_virtual_joint");
    msg3.joint_names.push_back("cube3_shaft_joint");
    msg3.joint_names.push_back("cube3_stiffness_preset_virtual_joint");
    msg4.joint_names.push_back("cube4_shaft_joint");
    msg4.joint_names.push_back("cube4_stiffness_preset_virtual_joint");
    msg5.joint_names.push_back("cube5_shaft_joint");
    msg5.joint_names.push_back("cube5_stiffness_preset_virtual_joint");
    msg6.joint_names.push_back("cube22_shaft_joint");
    msg6.joint_names.push_back("cube22_stiffness_preset_virtual_joint");

    double des34 = 1.0;
    double des1 = 0.5;
    double des2 = 0.4;
    double des5 = -0.75;
    
    trajectory_msgs::JointTrajectoryPoint point1;
    point1.time_from_start.sec=1;
    point1.positions.push_back(0.0);
    point1.positions.push_back(0.5);
    trajectory_msgs::JointTrajectoryPoint point2;
    point2.time_from_start.sec=1;
    point2.positions.push_back(0.0);
    point2.positions.push_back(0.5);
    trajectory_msgs::JointTrajectoryPoint point3;
    point3.time_from_start.sec=1;
    point3.positions.push_back(0.0);
    point3.positions.push_back(0.5);
    trajectory_msgs::JointTrajectoryPoint point4;
    point4.time_from_start.sec=1;
    point4.positions.push_back(0.0);
    point4.positions.push_back(0.5);
    trajectory_msgs::JointTrajectoryPoint point5;
    point5.time_from_start.sec=1;
    point5.positions.push_back(0.0);
    point5.positions.push_back(0.5);
    trajectory_msgs::JointTrajectoryPoint point6;
    point6.time_from_start.sec=1;
    point6.positions.push_back(0.0);
    point6.positions.push_back(0.5);

    msg1.points.push_back(point1);
    msg2.points.push_back(point2);
    msg3.points.push_back(point3);
    msg4.points.push_back(point4);
    msg5.points.push_back(point5);
    msg6.points.push_back(point6);
    
    ros::Time start = ros::Time::now();
    ros::Time stop = ros::Time::now();


    ros::Rate loop(10);
    
    while(ros::ok())
    {
        stop = ros::Time::now();

	if(stop-start > ros::Duration(2.0))
	{
	    start = ros::Time::now();

	    des34=-des34;
	    des1=-des1;

	    point1.positions.clear();
	    point1.positions.push_back(des1);
	    point1.positions.push_back(0.5);
	    point2.positions.clear();
	    point2.positions.push_back(des2-des34*0.4); //HACK
	    point2.positions.push_back(0.5);
	    point3.positions.clear();
	    point3.positions.push_back(des34);
	    point3.positions.push_back(0.5);
	    point4.positions.clear();
	    point4.positions.push_back(des34);
	    point4.positions.push_back(0.5);
	    point5.positions.clear();
	    point5.positions.push_back(des5-des34*0.75); //HACK
	    point5.positions.push_back(0.5);
	    point6.positions.clear();
	    point6.positions.push_back(-0.5-des34*0.5); //HACK
	    point6.positions.push_back(0.5);
    
	    msg1.points.clear();
	    msg2.points.clear();
	    msg3.points.clear();
	    msg4.points.clear();
	    msg5.points.clear();
	    msg6.points.clear();
	    msg1.points.push_back(point1);
	    msg2.points.push_back(point2);
	    msg3.points.push_back(point3);
	    msg4.points.push_back(point4);
	    msg5.points.push_back(point5);
	    msg6.points.push_back(point6);
	}

	traj_pub_1.publish(msg1);
	traj_pub_2.publish(msg2);
	traj_pub_3.publish(msg3);
	traj_pub_4.publish(msg4);
	traj_pub_5.publish(msg5);
	traj_pub_6.publish(msg6);
	ros::spinOnce();
	loop.sleep();
    }

    return 0;
}