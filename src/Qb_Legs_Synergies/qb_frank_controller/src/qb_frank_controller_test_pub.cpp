#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>

int main(int argc, char** argv)
{
    if(!ros::isInitialized()) ros::init(argc,argv,"qb_frank_controller_test_pub");
    
    ros::NodeHandle nh;
    
    ros::Publisher pubg = nh.advertise<geometry_msgs::Vector3>("/gaze",10);
    ros::Publisher publ = nh.advertise<geometry_msgs::Pose>("/frank_q_des_left",10);
    ros::Publisher pubr = nh.advertise<geometry_msgs::Pose>("/frank_q_des_right",10);

    geometry_msgs::Vector3 msgg;
    geometry_msgs::Pose msgl;
    geometry_msgs::Pose msgr;
    
    msgg.x = 0.0;
    msgg.y = 0.5;
    msgg.z = 0.5;
    
    msgl.position.x = 0.5;
    msgl.position.y = 0.5;
    msgl.position.z = 0.5;
    msgl.orientation.x = 0.5;
    msgl.orientation.y = 0.5;
    msgl.orientation.z = 0.0;
    msgl.orientation.w = 0.0;
    
    msgr.position.x = 0.5;
    msgr.position.y = 0.5;
    msgr.position.z = 0.5;
    msgr.orientation.x = 0.5;
    msgr.orientation.y = 0.5;
    msgr.orientation.z = 0.0;
    msgr.orientation.w = 0.0;
    
    ros::Rate loop(50);

    while(ros::ok())
    {
	pubg.publish(msgg);
	publ.publish(msgl);
	pubr.publish(msgr);
	ros::spinOnce();
	loop.sleep();
    }

    return 0;
}