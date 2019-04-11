#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64MultiArray.h>

#include <iostream>
#include "force_sensor_ati/force_sensor_ati.h"
#include <unistd.h>
#include <string>
#include <std_srvs/Empty.h>

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <ctime>
#include <dirent.h>


// WARNING!! TO PROPERLY USE THIS CODE BE SURE TO HAVE CONNECTED BOTH THE FEET, FIRST THE LEFT ONE

OptoDAQ daq;

bool setZerosCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    daq.zeroAll();
    return true;
}


int main(int argc, char **argv)
{
//   INIT MATRICES
  
Eigen::MatrixXd ML(12,6);
Eigen::MatrixXd MR(12,6);

ML(0,0)= -0.043110;
ML(0,1)= -0.712854;
ML(0,2)= 0.337812;
ML(0,3)= 0.007834;
ML(0,4)= -0.022045;
ML(0,5)= -0.043987;
ML(1,0)= 0.290516;
ML(1,1)= 0.047632;
ML(1,2)= -0.215946;
ML(1,3)= -0.001863;
ML(1,4)= 0.022198;
ML(1,5)= -0.001958;
ML(2,0)= -0.050674;
ML(2,1)= -0.010893;
ML(2,2)= -1.204045;
ML(2,3)= -0.023812;
ML(2,4)= 0.047094;
ML(2,5)= 0.002342;
ML(3,0)= 0.279307;
ML(3,1)= 0.119155;
ML(3,2)= -1.247605;
ML(3,3)= 0.017487;
ML(3,4)= 0.048149;
ML(3,5)= 0.022401;
ML(4,0)= -0.329921;
ML(4,1)= -0.210013;
ML(4,2)= -2.092224;
ML(4,3)= 0.006234;
ML(4,4)= 0.008114;
ML(4,5)= -0.017893;
ML(5,0)= 0.031632;
ML(5,1)= -0.048327;
ML(5,2)= -1.076292;
ML(5,3)= -0.023874;
ML(5,4)= -0.043988;
ML(5,5)= 0.000984;
ML(6,0)= -0.224785;
ML(6,1)= 0.277779;
ML(6,2)= -0.996199;
ML(6,3)= 0.005054;
ML(6,4)= 0.030369;
ML(6,5)= 0.028721;
ML(7,0)= -0.324152;
ML(7,1)= -0.608971;
ML(7,2)= -3.043267;
ML(7,3)= 0.025227;
ML(7,4)= 0.030752;
ML(7,5)= 0.020567;
ML(8,0)= 0.016663;
ML(8,1)= 0.048154;
ML(8,2)= -1.305152;
ML(8,3)= 0.026464;
ML(8,4)= -0.039828;
ML(8,5)= 0.001511;
ML(9,0)= 0.021365;
ML(9,1)= 0.165419;
ML(9,2)= 0.407141;
ML(9,3)= -0.001406;
ML(9,4)= -0.006483;
ML(9,5)= 0.009111;
ML(10,0)= 0.278663;
ML(10,1)= 0.088179;
ML(10,2)= 0.242144;
ML(10,3)= -0.002067;
ML(10,4)= -0.008332;
ML(10,5)= 0.007349;
ML(11,0)= -0.084818;
ML(11,1)= 0.011321;
ML(11,2)= -1.035046;
ML(11,3)= 0.021847;
ML(11,4)= 0.039180;
ML(11,5)= -0.003782;


MR(0,0)= 0.001545;
MR(0,1)= -0.354675;
MR(0,2)= -0.264982;
MR(0,3)= 0.008578;
MR(0,4)= 0.016709;
MR(0,5)= -0.013683;
MR(1,0)= 0.432042;
MR(1,1)= 0.160885;
MR(1,2)= 0.398085;
MR(1,3)= 0.007665;
MR(1,4)= 0.023475;
MR(1,5)= 0.014243;
MR(2,0)= -0.012958;
MR(2,1)= 0.082405;
MR(2,2)= -1.051177;
MR(2,3)= 0.019719;
MR(2,4)= 0.046772;
MR(2,5)= 0.002924;
MR(3,0)= -0.330192;
MR(3,1)= -0.063281;
MR(3,2)= -0.579276;
MR(3,3)= 0.033878;
MR(3,4)= -0.017273;
MR(3,5)= -0.013012;
MR(4,0)= -0.051479;
MR(4,1)= -0.241786;
MR(4,2)= -1.102123;
MR(4,3)= 0.009672;
MR(4,4)= 0.014038;
MR(4,5)= 0.009967;
MR(5,0)= 0.033508;
MR(5,1)= 0.007363;
MR(5,2)= -1.129698;
MR(5,3)= 0.022295;
MR(5,4)= -0.043151;
MR(5,5)= -0.002006;
MR(6,0)= 0.037339;
MR(6,1)= -0.022243;
MR(6,2)= -0.184780;
MR(6,3)= 0.030632;
MR(6,4)= -0.009060;
MR(6,5)= -0.011289;
MR(7,0)= 0.040841;
MR(7,1)= 0.341620;
MR(7,2)= -0.581836;
MR(7,3)= -0.011765;
MR(7,4)= 0.027273;
MR(7,5)= -0.002517;
MR(8,0)= 0.056581;
MR(8,1)= -0.019217;
MR(8,2)= -1.064235;
MR(8,3)= -0.026400;
MR(8,4)= -0.043157;
MR(8,5)= 0.002574;
MR(9,0)= 0.151079;
MR(9,1)= -0.007965;
MR(9,2)= 0.718499;
MR(9,3)= -0.011016;
MR(9,4)= -0.002645;
MR(9,5)= 0.002183;
MR(10,0)= 0.139850;
MR(10,1)= -0.181000;
MR(10,2)= -0.515811;
MR(10,3)= -0.002993;
MR(10,4)= 0.001233;
MR(10,5)= -0.013658;
MR(11,0)= -0.072292;
MR(11,1)= -0.055915;
MR(11,2)= -1.136047;
MR(11,3)= -0.020934;
MR(11,4)= 0.039638;
MR(11,5)= -0.001393;
//   END MATRICES
  
  
  
    ros::init(argc, argv, "force_sensor_ati");
    ros::NodeHandle nh = ros::NodeHandle("~");
    ROS_INFO("[ForceFensorATI] Node is ready");
    
    std::string Foot_Name;
    nh.getParam("Foot_Name",Foot_Name);
    
    Eigen::MatrixXd M(12,6);
    
      if(!strcmp(Foot_Name.c_str(),"left")){
	M=ML;
      }else{
	M=MR;
      }
    
    double spin_rate = 1000;
    ros::param::get("~spin_rate", spin_rate);
    ROS_INFO( "Spin Rate %lf", spin_rate);

    ros::Rate rate(spin_rate);

    int iSpeed = 1000; // Speed in Hz
    int iFilter = 15;  // Filter in Hz
    int port_id = 0;

    std::string sensor_port, on_topic;
    nh.param<int>("port_id", port_id, 0);
    
    //     Check port id
    bool cond1 = !strcmp(Foot_Name.c_str(),"left") && (port_id==1);
    //     bool cond2 = !strcmp(Foot_Name.c_str(),"right") && (port_id==0);
    
    if(cond1){ROS_ERROR_STREAM("*** FOOT INDEX & PORT ID WRONG *** ");return 0;}
    
    
    sensor_port = std::string("/dev/ttyACM") + std::to_string(port_id);
    
    nh.param<std::string>("on_topic", on_topic, "wrench");

    nh.param<int>("filter", iFilter, 15);

    OptoPorts ports;

    sleep(1.0);

    OPort* portlist = ports.listPorts(true);
    
// //     std::cout << "****" << std::string (portlist->name) << std::endl;
// //     std::cout << "**** 0 " << std::string (portlist[0].name) << std::endl;
// //     std::cout << "**** 1 " << std::string (portlist[1].name) << std::endl;
// //     
// //     std::cout << "**** I'm  "<<port_id << " and hence "<< std::string (portlist[port_id].name) << std::endl;

    ROS_INFO_STREAM("Opening sensor on port: " << sensor_port.c_str());
    ROS_INFO_STREAM("Publishing force measurements on topic: " << on_topic.c_str());
    
    if(!strcmp(Foot_Name.c_str(),"left")){port_id=0;}
    else{port_id=1;}
	    
    if (daq.open(portlist[ port_id ]))
//     if (daq.open(portlist[ port_id ]))
        // if (daq.open(portlist[ port_id -1 ])) /old configuration but using 
    {
        ROS_INFO_STREAM("I'm "<< Foot_Name<<" Sensor Initialized on port " << sensor_port.c_str());
    }
    else
    {
        ROS_ERROR_STREAM(Foot_Name<<" No sensor available on port " << sensor_port.c_str() );
        return 0;
        ros::shutdown();
    }

    geometry_msgs::Wrench force_measurements;


    SensorConfig sensorConfig;
    sensorConfig.setSpeed(iSpeed);
    sensorConfig.setFilter(iFilter);

    
//     bool bConfig = false;
//     bConfig = daq.sendConfig(sensorConfig);
//     sleep(2.0);
//     if (bConfig == false) {
//         daq.close();
//         ROS_ERROR_STREAM("Could not set config");
//         ros::shutdown();
//         return 0;
//     }
    
    bool bConfig = false;
    bConfig = daq.sendConfig(sensorConfig);
    sleep(1.0);
      while(bConfig == false) {
	  ROS_ERROR_STREAM("Could not set config");
	  bConfig = daq.sendConfig(sensorConfig);
	  sleep(1.0);
  //         ros::shutdown();
  //         return 0;
      }
      
    ROS_INFO_STREAM("Sensor Initialized with Speed: " << iSpeed << " and Filter: " << iFilter);

    OptoPackage* pack3D = 0;
    int size = daq.readAll(pack3D, false);
    
//     std::cout<<"size read all "<< size<<std::endl;
    int number_of_sensor = daq.getSensorSize();
    
    ROS_INFO_STREAM("Number of Sensor: " << number_of_sensor);

    ros::ServiceServer srv_setZeros = nh.advertiseService("setZeros", &setZerosCallback);

    
// //         //     Create publisher vector to publish each sensor forces retrieved
// //     std::vector<ros::Publisher> pubs_forces;
// //     
// //     for (unsigned i = 0; i < number_of_sensor; ++i) {
// //         pubs_forces.push_back( nh.advertise<geometry_msgs::WrenchStamped>( on_topic.c_str() + std::to_string(i), 100 ));
// //     }
    
//  Create publisher all forces
//     ros::Publisher pub_allforces = nh.advertise<std_msgs::Float64MultiArray>( "allforces", 100 );

   ros::Publisher pub_allforces = nh.advertise<std_msgs::Float64MultiArray>( Foot_Name+"_wrenches", 100 );
 
    
//     Put in da file yo!
    std::ofstream myfile;
     std::string file_results;

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
}
    /*system(cmd.c_str());  */  
    
    file_results=path_file_results+folder+"/RAW_"+Foot_Name+"_"+file_results;

    myfile.open(file_results.c_str());
    ros::Time begin = ros::Time::now();
    ros::Duration elapsed_time;
    
    
    while (nh.ok())
    {
        geometry_msgs::WrenchStamped force_readings;
        std_msgs::Float64MultiArray allforces;
// 	pack3D=0;
	
        size = daq.readAll(pack3D, false);
// 	std::cout<<"SIZE "<<size<<std::endl;
	
        if (size == 0)
            continue;

        for (int k = 0; k < number_of_sensor; k++)
        {
// 	  ONLY LAST PACKAGE, LAST READING, that's why k*size and i<1
            for (int i = 0; i < 1; i++)
            {
//                 force_readings.wrench.force.x  = pack3D[k * size + i].x;
//                 force_readings.wrench.force.y  = pack3D[k * size + i].y;
//                 force_readings.wrench.force.z  = pack3D[k * size + i].z;
// 		
//                 pubs_forces[k].publish(force_readings);
		
                allforces.data.push_back(pack3D[k * size + i].x);
                allforces.data.push_back(pack3D[k * size + i].y);
                allforces.data.push_back(pack3D[k * size + i].z);
            }
        } //end for number of sensor
        
        
//         TRY TO MULTIPLY AND SEND A WRENCH. NEED RESIZE OF allforces AND CASTING

		Eigen::MatrixXd new_forces_eig_M(1,allforces.data.size());
	      
		for(int i=0; i < allforces.data.size(); i++)
                {
		  new_forces_eig_M(0,i)=allforces.data[i];
		  }
		
		Eigen::MatrixXd New_Wrench_L(1,6); //Should be 1x6
				
		New_Wrench_L=new_forces_eig_M*M;
// 		OTHER NECESSARY PARAMETERS
		New_Wrench_L*=9.81*0.001/0.83;

   std_msgs::Float64MultiArray new_allforces;
      for(int i=0;i<New_Wrench_L.rows();i++){
      for(int j=0;j<New_Wrench_L.cols();j++){
	new_allforces.data.push_back(New_Wrench_L(i,j));
      }
//       std::cout<<std::endl;
    }


            elapsed_time = ros::Time::now() - begin;
            
            myfile << (elapsed_time.toSec())<<"\t";
                    for (unsigned int i = 0; i < allforces.data.size(); i++)
                {
                    myfile << allforces.data[i] <<"\t";
                }
                myfile << " " <<Foot_Name.c_str()<<std::endl;
            


        pub_allforces.publish(new_allforces);
        ros::spinOnce();
        rate.sleep();

    }

    sleep(1.0);
    daq.close();
    myfile.close();
    delete[] pack3D;
    return 0;
}
