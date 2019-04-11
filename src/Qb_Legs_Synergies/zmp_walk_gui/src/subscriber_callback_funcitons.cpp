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

//-------------SUBSCRIBER CALLBACK


void qt_example::Retrieve_Current(const std_msgs::Float64MultiArray::ConstPtr& msg){
  QString Q;
  double c1;
  double c2;
  double C;

    for(int j=0;j<12;j++)
    { c1=msg->data[j++];
      c2=msg->data[j];
//       std::cout<<" "<<(j-1)/2<<std::endl;
//       std::cout<<c1<<" "<<c2<<" "<<std::endl;
//       std::cout<<" "<<(j-1)/2<<std::endl;

      C=fabs(c1+c2)/1000;
    
      Q= QString::number(C,'g', 3);
      m_pTableWidget->item(0,(j-1)/2)->setText(Q);
      if(C<=0.5){
      m_pTableWidget->item(0,(j-1)/2)->setBackgroundColor(QColor(255,255,255));
      };
      if(C>0.5 && C<1.5){
      m_pTableWidget->item(0,(j-1)/2)->setBackgroundColor(QColor(255,255,153));
      };
      if(C>=1.5 && C<2.5){
	m_pTableWidget->item(0,(j-1)/2)->setBackgroundColor(QColor(255,153,51));
      }
      if(C>=2.5){
	m_pTableWidget->item(0,(j-1)/2)->setBackgroundColor(Qt::red);
      };
    }
  
  return;
}

void qt_example::Retrieve_Voltage(const std_msgs::Float64MultiArray::ConstPtr& msg){
  QString Q;
  double V;
  

    for(int j=0;j<6;j++)
    {	
      V=msg->data[j]/1000;
      Q= QString::number(V,'g', 4);
	if(V==0){
      m_pTableWidget->item(1,j)->setBackgroundColor(QColor(192,192,192));
      };
	if(V<0){
      m_pTableWidget->item(1,j)->setBackgroundColor(QColor(64,64,64));
      };
      
      if(V>0){
	m_pTableWidget->item(1,j)->setBackgroundColor(QColor(255,255,255));
      }
      if(V>0 && V<23){
	m_pTableWidget->item(1,j)->setBackgroundColor(QColor(255,153,51));
      }
      
      m_pTableWidget->item(1,j)->setText(Q);
    }

  return;
}




void qt_example::Retrieve_Bias(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  QString output;

       for(int i=0;i<6;i++){

      output=output+QString::number(msg->data[i], 'g', 3);
      if(i==5){}else{output=output+",";}
    }
    
    qt_example::biasLineEdit_read->setText(output);
   
  
return;
}

void qt_example::Retrieve_names_optimal(const zmp_walk_gui::dataset_name& msg)
{
if(msg.name_list.size()<=0 || OptList==NULL){return;}

if(names.size()!=msg.name_list.size()){ 
  //if different size destroy everything
std::cout<<" they are the different by size"<<std::endl;
// std::cout<<"54"<<std::endl;
      dataList=new QListWidgetItem(QString::fromStdString(" v "));
      OptList->addItem(dataList);
//       std::cout<<"59"<<std::endl;
  OptList->clear();
//     std::cout<<"55"<<std::endl;
//   dataList->clear();
  names.resize(msg.name_list.size());

//   QString::FromStdString()
//   std::cout<<"59"<<std::endl;
//   fromStdString(const std::string &str)
  for(int i=0;i<msg.name_list.size();i++){
//     dataList->append(QString::fromStdString(msg.name_list.at(i))); //append
      dataList=new QListWidgetItem(QString::fromStdString(msg.name_list.at(i)));
      OptList->addItem(dataList); //append
    connect(OptList, SIGNAL(itemDoubleClicked(QListWidgetItem*)), this, SLOT(send_opt(QListWidgetItem* )));
    names.at(i)=msg.name_list.at(i);
  }//end for
  
//   OptList->addItems(*dataList);//create
  
  
}else{//end if they are equal
//   std::cout<<"73"<<std::endl;
  int count=0;

  for(int i=0;i<msg.name_list.size();i++){ //check if it is the same dataset
    if(names.at(i)==msg.name_list.at(i)) count++;
  } 

  if(count<msg.name_list.size()){
    std::cout<<" they are the different "<<std::endl;
  
    OptList->clear();
//     dataList->clear();
    names.resize(msg.name_list.size());
  
    for(int i=0;i<msg.name_list.size();i++){
  
// //       dataList->append(QString::fromStdString(msg.name_list.at(i)));
// //       names.at(i)=msg.name_list.at(i);
            dataList=new QListWidgetItem(QString::fromStdString(msg.name_list.at(i)));
      OptList->addItem(dataList); //append
    connect(OptList, SIGNAL(itemDoubleClicked(QListWidgetItem*)), this, SLOT(send_opt(QListWidgetItem*)));
    names.at(i)=msg.name_list.at(i);
    }//end for
// //     OptList->addItems(*dataList);
  
  }else{ // end if

//     std::cout<<" they are the same "<<std::endl;
  }

}


// }
//   
// names.resize(msg.name_list.size());
// 
// for(int i=0;i<msg.name_list.size();i++){
//   if(names.at(i)==msg.name_list.at(i)) count++;
// }
// 
// if(count==
// OptList->clear();
// dataList->clear();

}

void qt_example::Retrieve_names_zmp(const zmp_walk_gui::dataset_name& msg)
{
if(msg.name_list.size()<=0 || OptList==NULL){return;}

if(names.size()!=msg.name_list.size()){ 
  //if different size destroy everything
std::cout<<" they are the different by size"<<std::endl;
// std::cout<<"54"<<std::endl;
      dataList=new QListWidgetItem(QString::fromStdString(" v "));
      OptList->addItem(dataList);
//       std::cout<<"59"<<std::endl;
  OptList->clear();
//     std::cout<<"55"<<std::endl;
//   dataList->clear();
  names.resize(msg.name_list.size());

//   QString::FromStdString()
//   std::cout<<"59"<<std::endl;
//   fromStdString(const std::string &str)
  for(int i=0;i<msg.name_list.size();i++){
//     dataList->append(QString::fromStdString(msg.name_list.at(i))); //append
      dataList=new QListWidgetItem(QString::fromStdString(msg.name_list.at(i)));
      OptList->addItem(dataList); //append
    connect(OptList, SIGNAL(itemDoubleClicked(QListWidgetItem*)), this, SLOT(send_opt(QListWidgetItem* )));
    names.at(i)=msg.name_list.at(i);
  }//end for
  
//   OptList->addItems(*dataList);//create
  
  
}else{//end if they are equal
//   std::cout<<"73"<<std::endl;
  int count=0;

  for(int i=0;i<msg.name_list.size();i++){ //check if it is the same dataset
    if(names.at(i)==msg.name_list.at(i)) count++;
  } 

  if(count<msg.name_list.size()){
    std::cout<<" they are the different "<<std::endl;
  
    OptList->clear();
//     dataList->clear();
    names.resize(msg.name_list.size());
  
    for(int i=0;i<msg.name_list.size();i++){
  
// //       dataList->append(QString::fromStdString(msg.name_list.at(i)));
// //       names.at(i)=msg.name_list.at(i);
            dataList=new QListWidgetItem(QString::fromStdString(msg.name_list.at(i)));
      OptList->addItem(dataList); //append
    connect(OptList, SIGNAL(itemDoubleClicked(QListWidgetItem*)), this, SLOT(send_opt(QListWidgetItem*)));
    names.at(i)=msg.name_list.at(i);
    }//end for
// //     OptList->addItems(*dataList);
  
  }else{ // end if

//     std::cout<<" they are the same "<<std::endl;
  }

}


// }
//   
// names.resize(msg.name_list.size());
// 
// for(int i=0;i<msg.name_list.size();i++){
//   if(names.at(i)==msg.name_list.at(i)) count++;
// }
// 
// if(count==
// OptList->clear();
// dataList->clear();

}


void qt_example::Retrieve_L_Wrench(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
QString output;
std::vector<double> v;

       for(int i=0;i<3;i++){
      output=output+QString::number(msg->data[i], 'g', 3);
      if(i==2){}else{output=output+",";}
    }
qt_example::LFLineEdit_read->setText(output);
}

void qt_example::Retrieve_R_Wrench(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
QString output;
std::vector<double> v;

       for(int i=0;i<3;i++){
      output=output+QString::number(msg->data[i], 'g', 3);
      if(i==2){}else{output=output+",";}
    }
qt_example::RFLineEdit_read->setText(output);
}

void qt_example::Retrieve_N_Sensor(const std_msgs::Float32::ConstPtr& msg)
{
QString output;

      output=QString::number(msg->data, 'g', 3);
      
qt_example::NSLineEdit_read->setText(output);
  
}


void qt_example::Retrieve_Distance(const std_msgs::Float32::ConstPtr& msg)
{
  QString output;
  

    output=QString::number(msg->data, 'g', 3);
    
    qt_example::DistanceLineEdit_read->setText(output);
  
return;
}

void qt_example::Retrieve_Distance_I(const std_msgs::Float32::ConstPtr& msg)
{
  QString output;
  

    output=QString::number(msg->data, 'g', 3);
    
    qt_example::DistanceLineEdit_I_read->setText(output);
  
return;
}

