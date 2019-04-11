#include <iostream>
#include <thread>
#include <QApplication>
#include "qt_example.h"
#include <QProcess>
#include <vector>
#include </lib/eigen-eigen/Eigen/Dense>
#include <ros/node_handle.h>

#include <qb_legs_gui/bias.h>

qt_example* p_gui;

void Retrieve_bias(const qb_legs_gui::bias& msg)
{
  QString output;
       for(int i=0;i<6;i++){

      output=output+QString::number(msg.bias[i], 'g', 3);
      if(i==5){}else{output=output+", ";}
    }
    
    p_gui->biasLineEdit_read->setText(output);
  
return;
}

int main(int argc, char** argv)
{
  ros::init(argc,argv,"qt_example_pr");
  ros::NodeHandle n;
  
  ros::Subscriber bias_sub=n.subscribe("/synergies_bias_echo",10,&Retrieve_bias);
  
  
  
    QApplication app(argc,argv);

    qt_example gui;
    p_gui=&gui;
    gui.show();
    gui.setWindowTitle("QbLegs GUI");
    
    app.exec();
       

    return 0;
}