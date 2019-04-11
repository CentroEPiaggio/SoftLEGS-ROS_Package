#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

#include <fstream>
#include <ctime>
#include <dirent.h>

std::vector<std_msgs::Float64MultiArray> vec_allforces;
std_msgs::Float64MultiArray matlab_data;

int matlab_enabled;

void callbackforce(const std_msgs::Float64MultiArray::ConstPtr& msg, int i)
{
    vec_allforces[i].data = msg->data;
}

void callbackTimeMatlab(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    matlab_data.data = msg->data;
    matlab_enabled = 1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "merge_forces");
    ros::NodeHandle nh = ros::NodeHandle("~");
    ROS_INFO("[MergeForces] Node is ready");

    double spin_rate = 1000;
    ros::param::get("~spin_rate", spin_rate);
    ROS_INFO_STREAM( "Spin Rate: " << spin_rate);

    ros::Rate rate(spin_rate);

    /*ros::Subscriber sub_matlab_data = nh.subscribe<std_msgs::Float64MultiArray>( "/matlab_data", 1, callbackTimeMatlab);*/

    XmlRpc::XmlRpcValue my_list;
    nh.getParam("topics", my_list);

    std::string file_results;
    nh.param<std::string>("file_results", file_results, "results.txt");
    ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_INFO_STREAM ("Writing of file: " << file_results.c_str());

//     CREATE A NEW PATH FOR RESULTING FILE
    
    nh.param<std::string>("file_results", file_results, "/home/gian/Documents/Leg_Synergies_models/Optoforce_Sensors_Exp_Results/results.txt");
    
    std::string path_file_results="/home/gian/Documents/Leg_Synergies_models/Optoforce_Sensors_Exp_Results/";
    
//     New coding for path file
    time_t now = time(0);
    tm *ltm = localtime(&now);
    std::string Year=std::to_string( 1900 + ltm->tm_year);
    std::string Month=std::to_string( ltm->tm_mon+1);
    std::string Day=std::to_string( ltm->tm_mday);
    std::string Hour=std::to_string(ltm->tm_hour);
    std::string Min=std::to_string(ltm->tm_min);
    std::string folder="Res_"+Year+"_"+Day+"_"+Month+"_"+Hour+"_"+Min;
    std::string cmd="mkdir "+ path_file_results + folder;
//     ROS_INFO_STREAM("***** FOLDER ******");
    std::string check_fold=path_file_results + folder;
//     ROS_INFO_STREAM("***** FOLDER ******");
    
    DIR* dir = opendir(check_fold.c_str());
if (dir)
{
    /* Directory exists. */
    closedir(dir);
}
else 
{
    /* Directory does not exist. */
    system(cmd.c_str());  
    ROS_INFO_STREAM("*** FOLDER DOES NOT EXIST! ***");
}
    
//     system(cmd.c_str());    
    
    file_results=path_file_results+folder+"/"+file_results;
    
    
    matlab_enabled = 0;
    std::vector<ros::Subscriber> list_subs;
    for ( int i = 0; i < my_list.size(); ++i)
    {
        ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
        ROS_INFO_STREAM("Merging topic: " <<  static_cast<std::string>(my_list[i]).c_str());
        std_msgs::Float64MultiArray new_forces;
        vec_allforces.push_back( new_forces );
        list_subs.push_back( nh.subscribe<std_msgs::Float64MultiArray>( static_cast<std::string>(my_list[i]).c_str(), 1, boost::bind(&callbackforce, _1, i) ) );
	
	std::cout<<"LIST "<<static_cast<std::string>(my_list[i]).c_str()<<std::endl;
    }
    int n;
    
    std::cin>>n;
    ros::Time begin = ros::Time::now();
    ros::Duration elapsed_time;



    std::ofstream myfile;
    myfile.open(file_results.c_str());
    // ROS_INFO_STREAM( "****** open file *******");

//     ros::Subscriber sub_matlab_data = nh.subscribe<std_msgs::Float64MultiArray>( "/matlab_data", 1, callbackTimeMatlab);


    if (vec_allforces.size() > 0)
    {
            // ROS_INFO_STREAM( "****** if size vec all forces *******");
        std::string on_topic;
        nh.param<std::string>("on_topic", on_topic, "allforces");
        ros::Publisher pub_allforces = nh.advertise<std_msgs::Float64MultiArray>( "allforces", 100 );
        // ROS_INFO_STREAM( "****** nh.ok "<<nh.ok()<<" *******");
        while ( nh.ok() )
        {
            std_msgs::Float64MultiArray allforces;
            elapsed_time = ros::Time::now() - begin;
            allforces.data = matlab_data.data;
            allforces.data.push_back(elapsed_time.toSec());
            // allforces.data.push_back(time_matlab.data);
            for (unsigned int i = 0; i < vec_allforces.size(); ++i)
            {
                allforces.data.insert(allforces.data.end(), vec_allforces[i].data.begin(), vec_allforces[i].data.end());
            }
            // ROS_INFO_STREAM( "****** matlab enabled "<<matlab_enabled<<" *******");
//             if (allforces.data.size() > 1 && matlab_enabled == 1)
	    if (allforces.data.size() > 1)
            {
//                 ROS_INFO_STREAM( "****** forces grater than 1 and matlab enabled *******");


                for (unsigned int i = 0; i < allforces.data.size(); i++)
                {
                    myfile << allforces.data[i] << "\t";
                }
                myfile << std::endl;
            }
 
 if(allforces.data.size()<8){
   std::cout<<" One foot lost "<<std::endl; 
 }else{
        std::cout<<"Left: "<<allforces.data[3]<<"   Right: "<<allforces.data[8]<<std::endl;
 }
            
            pub_allforces.publish(allforces);
            ros::spinOnce();
            rate.sleep();
        }
    }

    myfile.close();

    return 0;
}