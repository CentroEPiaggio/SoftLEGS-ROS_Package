#include <iostream>
#include <thread>
#include <QApplication>
#include "qb_gui.h"
#include <QProcess>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <ros/node_handle.h>
#include <std_msgs/Float32.h>
#include <QStyleFactory>
#include <QStyle>
#include <QWindowsStyle>
#include <QCleanlooksStyle>
#include <QCommonStyle>
#include <qb_legs_gui/bias.h>
#include <qb_legs_gui/data_msg.h>
#include <QImage>
// #include "/home/gian/catkin_ws/src/Qb_Legs_Synergies/synergies/include/synergies/data_in_common.h"


// qt_example* p_gui;




int main(int argc, char** argv)
{  
    ros::init(argc,argv,"qt_example_pr");
    QApplication app(argc,argv);

    qt_example gui;
 
    gui.show();
    gui.setWindowTitle("QbLegs GUI");
    QApplication::setStyle(new QCleanlooksStyle);
    


    

    
    app.exec();
       

    return 0;
}
