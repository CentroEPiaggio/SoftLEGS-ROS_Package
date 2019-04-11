#include <optimal_gui.h>
#include <iostream>
#include <QProcess>
#include <QTextLine>
#include <QTableWidget>
// #include <QStringList>
#include <ros/node_handle.h>
#include <QRadioButton>
#include <QFrame>
#include <QVBoxLayout>
#include <QPalette>
#include <string>
#include <string.h>
#include <cstdlib>
#include <QString>
#include <QGroupBox>
#include <QStringList>
#include <QLineEdit>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <iomanip>
#include <QImage>
#include <QPixmap>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>

void qt_example::on_button_clicked_pub_stiffness()
{


    std::cout<<"you clicked! you wanna pub Stiffness!"<<std::endl;
    
    QString S_String = stiffnessLineEdit_write->text();
    
    std::string S_val =(S_String.toStdString());
    
    char * bv=strdup(S_val.c_str());
    char* pch = strtok(bv," ,");
    
//     std_msgs::Float32 msg;
    geometry_msgs::Pose msg;
    
    
    int i=0;
    while (pch != NULL)
    {
     if(i==0){ msg.position.x=(std::stod(pch));}
     if(i==1){ msg.position.y=(std::stod(pch));}
     if(i==2){ msg.position.z=(std::stod(pch));}
     if(i==3){ msg.orientation.x=(std::stod(pch));}
     if(i==4){ msg.orientation.y=(std::stod(pch));}
     if(i==5){ msg.orientation.z=(std::stod(pch));}
      i++;
//       std::cout<<"n "<<i<<" "<<std::stod(pch)<<std::endl;
      pch = strtok (NULL, " ,");
    }
    
    pub_stiff.publish(msg);
  
}



void qt_example::on_button_clicked_pub_start_Opt()
{
  std_msgs::Float64 msg;
  pub_start.publish(msg);

}


void qt_example::on_button_clicked_pub_stop_Opt()
{
  std_msgs::Float64 msg;
  std::cout<<"pub stop"<<std::endl;
  pub_stop.publish(msg);
}


void qt_example::on_button_clicked_set_zero_forces()
{

    QString program = "rosservice";
    QStringList arguments;
    arguments<<"call"<< "/wrenches/setZeros";
    std::cout<<"service set zero"<<std::endl;

    QProcess *myProcess = new QProcess(this);
    myProcess->startDetached(program,  arguments);

}


void qt_example::on_button_clicked_launch_qb()
{
    std::cout<<"you clicked! you wanna launch nodes !"<<std::endl;
    
    
//     std::string cmd="xterm -e roslaunch Distance_Sensor distance.launch";
//     system(cmd.c_str());
    QString program = "roslaunch";
    QStringList arguments;
//     arguments<<"qb_chain_control"<< "qb_legs_control.launch";
    arguments<<"qb_legs_controller"<< "qb_legs_controller.launch";
    

    QProcess *myProcess = new QProcess(this);
    myProcess->startDetached(program,  arguments);
}



void qt_example::on_button_clicked_kill_qb()
{
    std::cout<<"you clicked! you wanna kill nodes!"<<std::endl;
//     std::string cmd="xterm ";
//     system(cmd.c_str());
    
    QString program = "rosnode";
    QStringList arguments;

      arguments<<"kill"<<"/qb_legs_controller";
    
      QProcess *myProcess = new QProcess(this);
      myProcess->startDetached(program,  arguments);    
      myProcess->kill();
        
    
}



void qt_example::on_button_clicked_launch_Opt()
{	
//   roslaunch optimal_walk optimal_walk.launch
    std::cout<<"you clicked! you wanna launch nodes !"<<std::endl;
    
    QString program = "roslaunch";
    QStringList arguments;
    arguments<<"optimal_walk"<< "optimal_walk.launch";
    

    QProcess *myProcess = new QProcess(this);
    myProcess->startDetached(program,  arguments);
    myProcess->kill();
    
    
}


void qt_example::on_button_clicked_kill_Opt()
{
    std::cout<<"you clicked! you wanna kill nodes!"<<std::endl;
//     std::string cmd="xterm ";
//     system(cmd.c_str());
    
    QString program = "rosnode";
    QStringList arguments;
    arguments<<"kill"<<"/optimal_walk_control";
    

    QProcess *myProcess = new QProcess(this);
    myProcess->startDetached(program,  arguments);
    myProcess->kill();
}

// void qt_example::on_button_clicked_launch_zmp()
// {	
// //   roslaunch zmp_walk zmp_walk.launch
//     std::cout<<"you clicked! you wanna launch nodes !"<<std::endl;
//     
//     QString program = "roslaunch";
//     QStringList arguments;
//     arguments<<"zmp_walk"<< "zmp_walk.launch";
//     
// 
//     QProcess *myProcess = new QProcess(this);
//     myProcess->startDetached(program,  arguments);
//     myProcess->kill();
//     
//     
// }
// 
// 
// void qt_example::on_button_clicked_kill_zmp()
// {
//     std::cout<<"you clicked! you wanna kill nodes!"<<std::endl;
// //     std::string cmd="xterm ";
// //     system(cmd.c_str());
//     
//     QString program = "rosnode";
//     QStringList arguments;
//     arguments<<"kill"<<"/zmp_walk_control";
//     
// 
//     QProcess *myProcess = new QProcess(this);
//     myProcess->startDetached(program,  arguments);
//     myProcess->kill();
// }


void qt_example::on_button_clicked_pub_bias()
{
    std::cout<<"you clicked! you wanna pub bias!"<<std::endl;
    
    QString Bias_String = biasLineEdit_write->text();
    
    std::string bias_val =(Bias_String.toStdString());
    
    char * bv=strdup(bias_val.c_str());
    char* pch = strtok(bv," ,");
    std::vector<double> ppp;
//     optimal_walk_gui::bias msg;
    std_msgs::Float64MultiArray msg;
    msg.data.resize(6);
    
    int i=0;
    while (pch != NULL)
    {
      msg.data[i]=(std::stod(pch));
      i++;
//       std::cout<<"n "<<i<<" "<<std::stod(pch)<<std::endl;
      pch = strtok (NULL, " ,");
    }
    
    bias_pub.publish(msg);

return;
}



void qt_example::on_button_clicked_kill_Wrenches()
{
    QString program = "rosnode";
    QStringList arguments;
    arguments<<"kill"<< "/wrenches";
    

    QProcess *myProcess = new QProcess(this);
    myProcess->startDetached(program,  arguments);

}

void qt_example::on_button_clicked_launch_Wrenches()
{

      QString program = "roslaunch";
    QStringList arguments;
    arguments<<"opto_wrenches"<< "opto_wrenches.launch";
    

    QProcess *myProcess = new QProcess(this);
    myProcess->startDetached(program,  arguments);
  
}



void qt_example::on_button_clicked_launch_zmp()
{	
//   roslaunch optimal_walk optimal_walk.launch
    std::cout<<"you clicked! you wanna launch nodes !"<<std::endl;
    
    QString program = "roslaunch";
    QStringList arguments;
    arguments<<"zmp_walk"<< "zmp_walk.launch";
    

    QProcess *myProcess = new QProcess(this);
    myProcess->startDetached(program,  arguments);
    myProcess->kill();
    
    
}

void qt_example::on_button_clicked_kill_zmp()
{
    QString program = "rosnode";
    QStringList arguments;
    arguments<<"kill"<< "/zmp_walk";
    

    QProcess *myProcess = new QProcess(this);
    myProcess->startDetached(program,  arguments);

}


void qt_example::on_button_clicked_pub_N()
{

    std_msgs::Float32 msg_n;
      
    QString N_String = NLineEdit_write->text();
    msg_n.data=N_String.toDouble();
    pub_N.publish(msg_n);

}

void qt_example::on_button_clicked_launch_Dist()
{
    std::cout<<"you clicked! you wanna launch nodes !"<<std::endl;
    
    
//     std::string cmd="xterm -e roslaunch Distance_Sensor distance.launch";
//     system(cmd.c_str());	
    QString program = "roslaunch";
    QStringList arguments;
    arguments<<"Distance_Sensor"<< "distance.launch";
    

    QProcess *myProcess = new QProcess(this);
    myProcess->startDetached(program,  arguments);
}


void qt_example::on_button_clicked_kill_Dist()
{
    std::cout<<"you clicked! you wanna kill nodes!"<<std::endl;
//     std::string cmd="xterm ";
//     system(cmd.c_str());
    
    QString program = "rosnode";
    QStringList arguments;
    arguments<<"kill"<<"/Desired_Velocity_Calculus";
    

    QProcess *myProcess = new QProcess(this);
    myProcess->startDetached(program,  arguments);
    myProcess->kill();
}