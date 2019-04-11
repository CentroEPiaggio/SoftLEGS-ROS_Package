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

// #include "src/on_button_click_functions.cpp"
// #include "src/subscriber_callback_funcitons.cpp"
// #include <QtQml>



//----------CONSTRUCTOR

qt_example::qt_example(): QWidget()
{	
  
  //------------------------
  
  QString program = "rosnode";
    QStringList arguments;
    arguments<<"kill"<< "/zmp_walk_control";
    

    QProcess *myProcess = new QProcess(this);
    myProcess->startDetached(program,  arguments);

  
  
  
  //------------------------
  pub_flag_gui=n.advertise<std_msgs::Bool>("/zmp_gui/flag_opt",10);
  std_msgs::Bool msg_b;
  msg_b.data=true;
//   std::cout<<msg_b.data<<std::endl;
  pub_flag_gui.publish(msg_b);
  dist_sub=n.subscribe("/filtered_distance",10,&qt_example::Retrieve_Distance,this);
//   speed_sub=n.subscribe("/synergies_des_speed",10,&qt_example::Retrieve_Speed,this);
//   err_tilde_sub=n.subscribe("/bias_trunk_ankles",10,&qt_example::Retrieve_Err_Tilde,this);
  L_wrench_sub=n.subscribe("/wrenches/left_wrenches",10,&qt_example::Retrieve_L_Wrench,this);
  R_wrench_sub=n.subscribe("/wrenches/right_wrenches",10,&qt_example::Retrieve_R_Wrench,this);
  dataset_name_sub_1=n.subscribe("/optimal_walk/dataset_name",10,&qt_example::Retrieve_names_optimal,this);
  dataset_name_sub_2=n.subscribe("/zmp_walk/dataset_name",10,&qt_example::Retrieve_names_zmp,this);
//   N_wrench_sub=n.subscribe("/wrenches/N",10,&qt_example::Retrieve_N_Sensor,this);
  dist_I_sub=n.subscribe("/distance_filtered_I",10,&qt_example::Retrieve_Distance_I,this);
  
//   pub_VmM=n.advertise<std_msgs::Float64MultiArray>("/update_min_max_v",10);
  pub_N=n.advertise<std_msgs::Float32>("/update_N",10);
//   pub_N_I=n.advertise<std_msgs::Float64MultiArray>("/update_NI_I",10);
//   pub_quant=n.advertise<std_msgs::Float32>("/update_quant",10);
//   pub_activate_quant=n.advertise<std_msgs::Bool>("/activate_quant",10);
  
  
  
  bias_pub=n.advertise<std_msgs::Float64MultiArray>("/zmp_walk/optimal_bias",10);
  bias_sub=n.subscribe("/zmp_walk/bias_echo",10,&qt_example::Retrieve_Bias,this);
  
//   bias_stop_pub=n.advertise<zmp_walk_gui::bias>("/synergies_bias_stop",10);
//   BT_pub=n.advertise<std_msgs::Float32>("/update_BT",10);
//   AT_pub=n.advertise<zmp_walk_gui::gain_bias_err>("/gain_bias_trunk_ankles",10);
  
//   pub_bias_stop_flag=n.advertise<std_msgs::Bool>("/enable_bias_stop",10);
  pub_ankle_trunk_gain_flag=n.advertise<std_msgs::Bool>("/enable_gain_ankle_trunk",10);
//   pub_GG=n.advertise<zmp_walk_gui::GG_msg>("/update_G",10);
//   pub_ddes=n.advertise<std_msgs::Float32>("/update_d_des",10);
  pub_stiff=n.advertise<geometry_msgs::Pose>("/qb_legs_stiffness",10);
  pub_des_opt=n.advertise<std_msgs::Float64>("/zmp_walk/des_opt",10);
  
  pub_start=n.advertise<std_msgs::Float64>("/zmp_walk/start",10);
    pub_stop=n.advertise<std_msgs::Float64>("/zmp_walk/stop",10);

      QImage myImage;
    myImage.load("/home/gian/catkin_ws/src/Qb_Legs_Synergies/zmp_walk_gui/Soft_Legs_Gui.png");

    QLabel * myLabel= new QLabel(this);
    myLabel->setPixmap(QPixmap::fromImage(myImage));
    QPixmap p=QPixmap::fromImage(myImage); // load pixmap
    
    
    
    
// get label dimensions
int w = myLabel->width();
int h = myLabel->height();
// std::cout<<w<<" "<<h<<std::endl;

// set a scaled pixmap to a w x h window keeping its aspect ratio 
myLabel->setPixmap(p.scaled(w*9,h*9,Qt::KeepAspectRatio));
//     myLabel->setPixmap(pix);
  
  QGridLayout * grid= new QGridLayout;
  
  QGridLayout * grid_1= new QGridLayout;
  QGridLayout * grid_2= new QGridLayout;
  QGridLayout * n_grid_2= new QGridLayout;
  
  
  grid->addLayout(grid_1,0,0);
  grid->addLayout(grid_2,1,0);

    int count=0;
    int count_r=0;
    
    grid_1->addWidget(myLabel,count_r,count++);
    grid_1->addWidget(createNodeGroup(),count_r,count);
    count=0;
    count_r=0;

  
grid_2->addWidget(createDistanceGroup(),count_r,count++);
grid_2->addWidget(createBiasGroup(),count_r,count++);
  grid_2->addLayout(n_grid_2,count_r,count++);
  
  
  n_grid_2->addWidget(createStiffGroup(),0,0);
  n_grid_2->addWidget(createForceGroup(),1,0);
  
  grid_2->addWidget(createOptGroup(),count_r,count++);
  
  
  
// // //     grid_1->addWidget(createNodeGroup(),count_r,count);
// // //   count=0;
// // //   count_r=0;
// // // //   grid->addWidget(&myFrame_Node,count_r,count++);
// // //   grid_2->addWidget(createDistanceGroup(),count_r,count++);
// // //   
// // //   grid_2->addWidget(createBiasGroup(),count_r,count++);
// // // //   grid_2->addWidget(createStopBiasGroup(),count_r,count++);
// // //   grid_2->addLayout(n_grid_2,count_r,count++);
// // //   
// // //   n_grid_2->addWidget(createStiffGroup());
// // //   n_grid_2->addWidget(createForceGroup());
// // //   
// // // //   grid_2->addWidget(createStiffGroup(),count_r,count++);
// // //   grid_2->addWidget(createMiscGroup(),count_r,count++);
  
  
  

  
  setLayout(grid);
  setWindowTitle(tr("SoftLegs W/R Interface"));
  
//   quantLineEdit_write->setDisabled(1);

    
}


void qt_example::send_opt(QListWidgetItem* Item)
{
std::string sim=(Item->text()).toStdString();
// std::string myText("some-text-to-tokenize");
    std::istringstream iss(sim);
    std::string token;
    std::getline(iss, token, ' ');

  std_msgs::Float64 msg;
  msg.data=atof(token.c_str());
  std::cout<<"i am going to publish "<<msg.data<<std::endl;
    pub_des_opt.publish(msg);
  std::cout<<"i 've published "<<msg.data<<std::endl;
  
}


QGroupBox* qt_example::createOptGroup(){
  QGroupBox* groupBox = new QGroupBox(tr("ZMP Walk"));
  QGridLayout * n_grid= new QGridLayout;
  
//   OptList= new QListWidget();
  
  
  OptList=new QListWidget();

    
    button_start_Opt.setText("Start Sim");
    button_stop_Opt.setText("Stop Sim");
    
    button_start_Opt.sizeHint();
    connect(&button_start_Opt,SIGNAL(clicked(bool)),this,SLOT(on_button_clicked_pub_start_Opt()));
    button_stop_Opt.sizeHint();
    
    
    connect(&button_stop_Opt,SIGNAL(clicked(bool)),this,SLOT(on_button_clicked_pub_stop_Opt()));  
    n_grid->addWidget(OptList,0,0);
    QGridLayout* n_grid_2=new QGridLayout;
    n_grid->addLayout(n_grid_2,1,0);
    n_grid_2->addWidget(&button_start_Opt,0,0);
    n_grid_2->addWidget(&button_stop_Opt,0,1);

    
    groupBox->setLayout(n_grid);
    
  return groupBox;
}




QGroupBox* qt_example::createForceGroup()
{
  QGroupBox* groupBox= new QGroupBox(tr("Opto Forces"));
   QGridLayout * n_grid= new QGridLayout;
    
    
    QLabel * LFLabel=new QLabel(tr("Left Foot"));
    QLabel * RFLabel=new QLabel(tr("Right Foot"));
    QLabel * NSLabel=new QLabel(tr("Sensors 'on' "));
    
    LFLineEdit_read = new QLineEdit();
    LFLineEdit_read->setText("0,0,0");
    LFLineEdit_read->setCursorPosition(0);
    LFLineEdit_read->setReadOnly(true);
    LFLineEdit_read->setFocus();
    LFLineEdit_read->sizeHint();
    
    RFLineEdit_read = new QLineEdit();
    RFLineEdit_read->setText("0,0,0");
    RFLineEdit_read->setCursorPosition(0);
    RFLineEdit_read->setReadOnly(true);
    RFLineEdit_read->setFocus();
    RFLineEdit_read->sizeHint();
    
    NSLineEdit_read = new QLineEdit();
    NSLineEdit_read->setText("0");
    NSLineEdit_read->setCursorPosition(0);
    NSLineEdit_read->setReadOnly(true);
    NSLineEdit_read->setFocus();
    NSLineEdit_read->setAlignment(Qt::AlignRight);
    NSLineEdit_read->setMaximumWidth(20);
    
    QGridLayout * n_n_grid= new QGridLayout;
    n_grid->addLayout(n_n_grid,0,1);
    
    n_n_grid->addWidget(NSLineEdit_read,0,1);
    n_n_grid->addWidget(NSLabel,0,0);
    
    button_force_bias.setText("Bias Forces");
    connect(&button_force_bias,SIGNAL(clicked(bool)),this,SLOT(on_button_clicked_set_zero_forces()));  
    n_grid->addWidget(&button_force_bias,0,0);
    n_grid->addWidget(LFLabel,1,0);
    n_grid->addWidget(RFLabel,1,1);
    n_grid->addWidget(LFLineEdit_read,2,0);
    n_grid->addWidget(RFLineEdit_read,2,1);
  groupBox->setLayout(n_grid);
  return groupBox;
  
}



QGroupBox* qt_example::createStiffGroup()
{
QGroupBox* groupBox= new QGroupBox(tr("Stiffness"));


    QGridLayout * n_grid= new QGridLayout;
    QGridLayout * n_grid_1= new QGridLayout;
    QGridLayout * n_grid_2= new QGridLayout;
    n_grid->addLayout(n_grid_1,0,0);
    n_grid->addLayout(n_grid_2,1,0);
    
   
    
    stiffnessLineEdit_write = new QLineEdit();
    stiffnessLineEdit_write->setText("35,35,35,35,35,35");
    stiffnessLineEdit_write->setCursorPosition(0);
    stiffnessLineEdit_write->setFocus();
    stiffnessLineEdit_write->sizeHint();

     button_pub_stiffness.setText("Pub Stiffness");
     button_pub_stiffness.sizeHint();
    connect(& button_pub_stiffness,SIGNAL(clicked(bool)),this,SLOT(on_button_clicked_pub_stiffness()));  
    n_grid->addWidget(stiffnessLineEdit_write);
    n_grid->addWidget(&button_pub_stiffness);
  
  groupBox->setLayout(n_grid);
  
  return groupBox;

}


QGroupBox* qt_example::createNodeGroup()
{

  QGroupBox *groupBox = new QGroupBox(tr("Node Management"));
  
  
//     QRadioButton *radio1 = new QRadioButton(tr("&Radio button 1"));
//     QRadioButton *radio2 = new QRadioButton(tr("R&adio button 2"));
//     QRadioButton *radio3 = new QRadioButton(tr("Ra&dio button 3"));
// 
//     radio1->setChecked(true);
//     
//     QVBoxLayout *vbox = new QVBoxLayout;
//     vbox->addWidget(radio1);
//     vbox->addWidget(radio2);
//     vbox->addWidget(radio3);
//     vbox->addStretch(1);
//     groupBox->setLayout(vbox);
    
    
    button_1.setText("Launch Distance S.");
    button_2.setText("Kill Distance S.");
    button_3.setText("Launch ZMP");
    button_4.setText("Kill ZMP");
    button_5.setText("Launch qb");
    button_6.setText("Kill qb");
//     button_7.setText("Launch ZMP");
//     button_8.setText("Kill ZMP");
    
    button_launch_Wrenches.setText("Launch Opto");
    button_kill_Wrenches.setText("Kill Opto");
    
    
    
    connect(&button_1,SIGNAL(clicked(bool)),this,SLOT(on_button_clicked_launch_Dist()));
    connect(&button_2,SIGNAL(clicked(bool)),this,SLOT(on_button_clicked_kill_Dist()));
    connect(&button_3,SIGNAL(clicked(bool)),this,SLOT(on_button_clicked_launch_Opt()));
    connect(&button_4,SIGNAL(clicked(bool)),this,SLOT(on_button_clicked_kill_Opt()));    
    connect(&button_5,SIGNAL(clicked(bool)),this,SLOT(on_button_clicked_launch_qb()));
    connect(&button_6,SIGNAL(clicked(bool)),this,SLOT(on_button_clicked_kill_qb()));
    
//     connect(&button_7,SIGNAL(clicked(bool)),this,SLOT(on_button_clicked_launch_zmp()));
//     connect(&button_8,SIGNAL(clicked(bool)),this,SLOT(on_button_clicked_kill_zmp())); 
    
    connect(&button_launch_Wrenches,SIGNAL(clicked(bool)),this,SLOT(on_button_clicked_launch_Wrenches())); 
    connect(&button_kill_Wrenches,SIGNAL(clicked(bool)),this,SLOT(on_button_clicked_kill_Wrenches())); 
    
    QGridLayout * n_grid = new QGridLayout;

    
    n_grid->addWidget(&button_1,0,0);
    n_grid->addWidget(&button_2,0,1);
//     n_grid->addWidget(&button_7,1,0);
//     n_grid->addWidget(&button_8,1,1);
    n_grid->addWidget(&button_3,1,0);
    n_grid->addWidget(&button_4,1,1);
    n_grid->addWidget(&button_5,2,0);
    n_grid->addWidget(&button_6,2,1);
    n_grid->addWidget(&button_launch_Wrenches,3,0);
    n_grid->addWidget(&button_kill_Wrenches,3,1);
    
    
    

    
    QRadioButton *radio1 = new QRadioButton(tr("&Save Signals"));

    radio1->setChecked(false);
    connect(radio1,SIGNAL(toggled(bool)),this,SLOT(itwasclicked(bool)));
    n_grid->addWidget(radio1,1,3);
    
    
//     vbox->addWidget(&button_1);
//     vbox->addWidget(&button_2);
//     vbox->addWidget(&button_3);
//     vbox->addWidget(&button_4);
//     vbox->addWidget(&button_5);
//     vbox->addWidget(&button_6);
    


groupBox->setLayout(n_grid);

return groupBox;
  
}

QGroupBox * qt_example::createBiasGroup()
{
  
      
    QGroupBox *groupBox= new QGroupBox(tr("Bias"));
    QLabel *BiasLabel_C = new QLabel(tr(" C. Bias:"));
    QLabel *BiasLabel_D = new QLabel(tr(" D. Bias:"));
    
    
    biasLineEdit_read = new QLineEdit;
    biasLineEdit_read->setPlaceholderText("Placeholder Text");
    biasLineEdit_read->setReadOnly(true);
    biasLineEdit_read->setFocus();
    
    biasLineEdit_write = new QLineEdit;
//     biasLineEdit_write->setInputMask("xxx,xxx,xxx,xxx,xxx,xxx");
    biasLineEdit_write->setText("-11,0,25,-3,0,18"); //-1,0,15,-3,0,16
    biasLineEdit_write->setCursorPosition(0);
    biasLineEdit_write->setFocus();
    
    button_pub_bias.setText("Pub Des Bias");
    connect(&button_pub_bias,SIGNAL(clicked(bool)),this,SLOT(on_button_clicked_pub_bias()));
    

    QGridLayout * n_grid= new QGridLayout;

    QGridLayout * n_grid_1= new QGridLayout;
    QGridLayout * n_grid_2= new QGridLayout;
    QGridLayout * n_grid_3= new QGridLayout;
    n_grid->addLayout(n_grid_1,0,0);
    n_grid->addLayout(n_grid_2,1,0);
    
//     n_grid_2->addWidget(createStiffGroup());
//     n_grid_2->addWidget(createStopBiasGroup());
    
    n_grid_1->addWidget(BiasLabel_C);
    n_grid_1->addWidget(biasLineEdit_read); 
    n_grid_1->addWidget(BiasLabel_D);
    n_grid_1->addWidget(biasLineEdit_write);
    n_grid_1->addWidget(&button_pub_bias);
    
    
    timer =new QTimer(this);
    connect(timer,SIGNAL(timeout()),this,SLOT(timer_bias()));
    timer->start(10);
    timer_bias();
  
    groupBox->setLayout(n_grid);
    
  return groupBox;
}



QGroupBox * qt_example::createDistanceGroup()
{
  QGroupBox *groupBox= new QGroupBox(tr("Distance"));
  
  QLabel *Label_D = new QLabel(tr(" Dist: "));
  QLabel *Label_S = new QLabel(tr(" Speed: "));
  
  DistanceLineEdit_read = new QLineEdit;
  DistanceLineEdit_read->setPlaceholderText("Placeholder Text");
  DistanceLineEdit_read->setReadOnly(true);
  DistanceLineEdit_read->setFocus();
  DistanceLineEdit_read->setMaximumWidth(60);
  
  DistanceLineEdit_I_read = new QLineEdit;
  DistanceLineEdit_I_read->setPlaceholderText("Placeholder Text");
  DistanceLineEdit_I_read->setReadOnly(true);
  DistanceLineEdit_I_read->setFocus();
  DistanceLineEdit_I_read->setMaximumWidth(60);
  
//       QLineEdit * DistanceLineEdit_I_read;
//     QLineEdit * NILineEdit_write;
  
  NILineEdit_write = new QLineEdit;
  NILineEdit_write->setPlaceholderText("1");
  NILineEdit_write->setFocus();
  NILineEdit_write->setMaximumWidth(40);
  
  
/*  
  SpeedLineEdit_read = new QLineEdit;
  SpeedLineEdit_read->setPlaceholderText("Placeholder Text");
  SpeedLineEdit_read->setReadOnly(true);
  SpeedLineEdit_read->setFocus();
  SpeedLineEdit_read->setMaximumWidth(80);*/
  
  
  QGridLayout * n_grid= new QGridLayout;
  
  QGridLayout * n_grid_3= new QGridLayout;
  QGridLayout * n_grid_4= new QGridLayout;
  
  QLabel *Label_D_I = new QLabel(tr(" Dist I: "));
  n_grid_3->addWidget(Label_D,0,0);
  n_grid_3->addWidget(DistanceLineEdit_read,0,1);
//   n_grid_3->addWidget(Label_D_I,0,2);
//   n_grid_3->addWidget(DistanceLineEdit_I_read,0,3);
  
//   n_grid_4->addWidget(Label_S,0,0);
//   n_grid_4->addWidget(SpeedLineEdit_read,0,1);
  QGridLayout * n_grid_1=new QGridLayout;
  QGridLayout * n_grid_2=new QGridLayout;
  
  QLabel * Label_N = new QLabel(tr(" N: "));
  QLabel * Label_N_I = new QLabel(tr(" NI: "));
  
  NLineEdit_write= new QLineEdit;
  NLineEdit_write->setPlaceholderText("30");
  NLineEdit_write->setFocus();
  NLineEdit_write->setMaximumWidth(40);
  
  QLabel * Label_VmM = new QLabel(tr(" Vmin, Vmax: "));
  
  VLineEdit_write= new QLineEdit;
  VLineEdit_write->setPlaceholderText("0.065,0.18");

  VLineEdit_write->setMaximumWidth(80);
  VLineEdit_write->setCursorPosition(0);
    VLineEdit_write->setFocus();
  
  n_grid_1->addWidget(Label_N,0,0);
  n_grid_1->addWidget(NLineEdit_write,0,1);
//   n_grid_1->addWidget(Label_N_I,0,2);
//   n_grid_1->addWidget(NILineEdit_write,0,3);
  
//   n_grid_2->addWidget(Label_VmM,0,0);
//   n_grid_2->addWidget(VLineEdit_write,0,1);
//     n_grid_4->addWidget(Label_VmM,1,0);
//   n_grid_4->addWidget(VLineEdit_write,1,1);
  
//   n_grid_2->addWdiget(,0,0);
//   n_grid_2->addWidget(,0,1);

  QGridLayout * n_grid_5=new QGridLayout;
 
      button_pub_N_VmM.setText("Pub N");
    connect(&button_pub_N_VmM,SIGNAL(clicked(bool)),this,SLOT(on_button_clicked_pub_N()));
    
     n_grid_5->addWidget(&button_pub_N_VmM);
      
  QGridLayout * n_grid_6=new QGridLayout; 
     
//   quantLineEdit_write= new QLineEdit;
//   quantLineEdit_write->setPlaceholderText("0.005");
//   quantLineEdit_write->setFocus();
//   quantLineEdit_write->setMaximumWidth(80);
  
//   connect(quantLineEdit_write, SIGNAL(returnPressed()),this,SLOT(change_quant()));
  
//        QRadioButton *quant_radio = new QRadioButton(tr("&Activate quant"));
//        quant_radio->setChecked(false);
  
//   connect(quant_radio,SIGNAL(toggled(bool)),this,SLOT(itwasclicked_quant(bool)));

//   n_grid_6->addWidget(quant_radio,0,0);
//   n_grid_6->addWidget(quantLineEdit_write,0,1);
// 
    n_grid->addLayout(n_grid_3,1,0);
  n_grid->addLayout(n_grid_4,3,0);
  n_grid->addLayout(n_grid_1,2,0);
  n_grid->addLayout(n_grid_2,3,1);
   n_grid->addLayout(n_grid_5,4,0);
//    n_grid->addLayout(n_grid_6,5,0);
   
   
  
  groupBox->setLayout(n_grid);
    
  return groupBox;
}


void qt_example::timer_bias()
{	

    ros::spinOnce();


return;
}

void qt_example::itwasclicked(bool checked)
{
if(checked)
{
    QString program = "roslaunch";
    QStringList arguments;
    arguments<<"save_qb_legs"<< "save_qb_legs.launch";
    

    QProcess *myProcess = new QProcess(this);
    myProcess->startDetached(program,  arguments);
}  else{
    QString program = "rosnode";
    QStringList arguments;
    arguments<<"kill"<< "/save_qb_legs";
    

    QProcess *myProcess = new QProcess(this);
    myProcess->startDetached(program,  arguments);
}
}




qt_example::~qt_example()
{

}

