#include <qt_example.h>
#include <iostream>
#include <QProcess>
#include <QTextLine>
#include <QTableWidget>
// #include <QStringList>
#include <ros/node_handle.h>


#include <iomanip>
// #include "/home/gian/catkin_ws/src/Qb_Legs_Synergies/synergies/include/synergies/data_in_common.h"

// file:///home/gian/catkin_ws/src/Qb_Legs_Synergies/synergies/include/synergies/synergy_values_2.h

qt_example::qt_example(): QWidget()
{	
  bias_in_common={0,0,0,0,0,0};

    setLayout(&main_layout_1);
    main_layout_1.addLayout(&main_layout_2);
    main_layout_1.addLayout(&main_layout_3);
  
    button_1.setText("Launch Distance Sensor");
    
    main_layout_2.addWidget(&button_1);
//     setLayout(&main_layout_2);
    
    connect(&button_1,SIGNAL(clicked(bool)),this,SLOT(on_button_clicked_launch_Dist()));
    
    button_2.setText("Kill Distance Sensor");
    main_layout_3.addWidget(&button_2);
//     setLayout(&main_layout_1);
    
    connect(&button_2,SIGNAL(clicked(bool)),this,SLOT(on_button_clicked_kill_Dist()));
    
    button_3.setText("Launch Synergies");
    main_layout_2.addWidget(&button_3);
//     setLayout(&main_layout_2);
    
    connect(&button_3,SIGNAL(clicked(bool)),this,SLOT(on_button_clicked_launch_Syn()));
    
    button_4.setText("Kill Synergies");
    main_layout_3.addWidget(&button_4);
//     setLayout(&main_layout_2);
    
    connect(&button_4,SIGNAL(clicked(bool)),this,SLOT(on_button_clicked_kill_Syn()));
    
    button_5.setText("Launch qb");
    main_layout_2.addWidget(&button_5);
//     setLayout(&main_layout_2);
    
    connect(&button_5,SIGNAL(clicked(bool)),this,SLOT(on_button_clicked_launch_qb()));
    
    button_6.setText("Kill qb");
    main_layout_3.addWidget(&button_6);
//     setLayout(&main_layout_2);
    
    connect(&button_6,SIGNAL(clicked(bool)),this,SLOT(on_button_clicked_kill_qb()));
    
    QGroupBox *biasGroup= new QGroupBox(tr("Bias"));
    QLabel *BiasLabel_C = new QLabel(tr(" C. Bias:"));
    QLabel *BiasLabel_D = new QLabel(tr(" D. Bias:"));
    
    
    biasLineEdit_read = new QLineEdit;
    biasLineEdit_read->setPlaceholderText("Placeholder Text");
    biasLineEdit_read->setReadOnly(true);

    
    biasLineEdit_read->setFocus();
    biasLineEdit_write = new QLineEdit;

    biasLineEdit_write->setInputMask("00,00,00,00,00,00");
    biasLineEdit_write->setText("00,00,00,00,00,00");
    biasLineEdit_write->setCursorPosition(0);
// 	std::cout<<"cane"<<std::endl;
    biasLineEdit_write->setFocus();
    
    button_pub_bias.setText("Pub Des Bias");
    connect(&button_pub_bias,SIGNAL(clicked(bool)),this,SLOT(on_button_clicked_pub_bias()));
    
//     connect()
    QGridLayout * BiasLayout= new QGridLayout;
    main_layout_1.addLayout(BiasLayout);
    
    BiasLayout->addWidget(BiasLabel_C);
    BiasLayout->addWidget(biasLineEdit_read);
    
    BiasLayout->addWidget(BiasLabel_D);
    BiasLayout->addWidget(biasLineEdit_write);
    BiasLayout->addWidget(&button_pub_bias);
    biasGroup->setWindowTitle(tr("Bias S"));
    biasGroup->setLayout(BiasLayout);
    
//     BiasLayout->setWindowTitle(tr("Bias S"));
    
    
    timer =new QTimer(this);
    connect(timer,SIGNAL(timeout()),this,SLOT(timer_bias()));
    timer->start(1000);
    timer_bias();
    

    
//     main_layout_1.addWidget(BiasLabel_D);
//     main_layout_1.addWidget(biasLineEdit);
    


    
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


void qt_example::on_button_clicked_launch_qb()
{
    std::cout<<"you clicked! you wanna launch nodes !"<<std::endl;
    
    
//     std::string cmd="xterm -e roslaunch Distance_Sensor distance.launch";
//     system(cmd.c_str());
    QString program = "roslaunch";
    QStringList arguments;
    arguments<<"qb_chain_control"<< "qb_legs_control.launch";
    

    QProcess *myProcess = new QProcess(this);
    myProcess->startDetached(program,  arguments);
}

// /qb_device_communication_handler
// /qb_legs/cube1/controller_spawner
// /qb_legs/cube1/qb_move_control
// /qb_legs/cube2/controller_spawner
// /qb_legs/cube2/qb_move_control
// /qb_legs/cube3/controller_spawner
// /qb_legs/cube3/qb_move_control
// /qb_legs/cube4/controller_spawner
// /qb_legs/cube4/qb_move_control
// /qb_legs/cube5/controller_spawner
// /qb_legs/cube5/qb_move_control
// /qb_legs/cube6/controller_spawner
// /qb_legs/cube6/qb_move_control


void qt_example::on_button_clicked_kill_qb()
{
    std::cout<<"you clicked! you wanna kill nodes!"<<std::endl;
//     std::string cmd="xterm ";
//     system(cmd.c_str());
    
    QString program = "rosnode";
    QStringList arguments;
    
    
    arguments<<"kill"<<"/qb_device_communication_handler";
    

    QProcess *myProcess = new QProcess(this);
    myProcess->startDetached(program,  arguments);
    myProcess->kill();
    std::string str;
    
    for(int j=0;j<3;j++){
    for(int i=0;i<6;i++){
      str="qb_legs/cube"+std::to_string(i+1)+"/controller_spawner";
      
      arguments<<"kill"<<str.c_str();
//     QProcess *myProcess = new QProcess(this);
      myProcess->startDetached(program,  arguments);
      myProcess->kill();
    
      str="qb_legs/cube"+std::to_string(i+1)+"/qb_move_control";
      arguments<<"kill"<<str.c_str();
    
      myProcess->startDetached(program,  arguments);
      myProcess->kill();
    
    }//end for
    }//end for 2
    
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

void qt_example::on_button_clicked_launch_Syn()
{
    std::cout<<"you clicked! you wanna launch nodes !"<<std::endl;
    
    
//     std::string cmd="xterm -e roslaunch Distance_Sensor distance.launch";
//     system(cmd.c_str());
    QString program = "roslaunch";
    QStringList arguments;
    arguments<<"synergies"<< "synergies.launch";
    

    QProcess *myProcess = new QProcess(this);
    myProcess->startDetached(program,  arguments);
    
    
   
    
    
    
}


void qt_example::on_button_clicked_kill_Syn()
{
    std::cout<<"you clicked! you wanna kill nodes!"<<std::endl;
//     std::string cmd="xterm ";
//     system(cmd.c_str());
    
    QString program = "rosnode";
    QStringList arguments;
    arguments<<"kill"<<"/synergies_qb_legs_control_backup";
    

    QProcess *myProcess = new QProcess(this);
    myProcess->startDetached(program,  arguments);
    myProcess->kill();
}

void qt_example::on_button_clicked_pub_bias()
{
    std::cout<<"you clicked! you wanna pub bias!"<<std::endl;
//     std::string cmd="xterm ";
//     system(cmd.c_str());
    
    QString Bias_String = biasLineEdit_write->text();
    std::string bias_val =(Bias_String.toStdString());
    std::cout<<bias_val<<std::endl;
    

    std::string prova="rostopic pub -1 /synergies_bias synergies/bias '{bias: ["+bias_val+"]}'";
//     prova
    system(prova.c_str());

//     QString output;
//     std::string num;
//     std::string cc;
//     for(int i=0;i<6;i++){
// 
//       output=output+QString::number(bias_in_common[i], 'g', 3);
//       if(i==5){}else{output=output+", ";}
//     
//     std::cout<<output.toStdString()<<num<<std::endl;
      
//     }

//     output=num.c_str();
//     biasLineEdit_read->setText(output);
//     myProcess->kill();
//     std::cout<<output.toStdString()<<std::endl;
//     myProcess->kill();
}

void qt_example::timer_bias()
{	
//   QString output;

//   QTime time = QTime::currentTime();
//   QString text = time.toString("hh:mm");
//   if ((time.second() % 2) == 0){}
//   else{
    ros::spinOnce();
//     std::cout<<text.toStdString()<<std::endl;
// //      for(int i=0;i<6;i++){
// // 
// //       output=output+QString::number(bias_in_common[i], 'g', 3);
// //       if(i==5){}else{output=output+", ";}
// //     }
// //     
// //     biasLineEdit_read->setText(output);
//     std::cout<<output.toStdString()<<std::endl;
    
//   }

return;
}

qt_example::~qt_example()
{

}




