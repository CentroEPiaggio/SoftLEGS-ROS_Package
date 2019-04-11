#include <QWidget>
#include <QPushButton>
#include <QHBoxLayout>
#include <string>
#include <QLineEdit>
#include <QComboBox>
#include <QGroupBox>
#include <QLabel>
#include <QGridLayout>
#include <QString>
#include <QStringList>
#include <QTimer>
#include <QTime>
#include <vector>
#include <eigen3/Eigen/Dense>

#include <vector>
#include <QFrame>
#include <ros/node_handle.h>

#include <qb_legs_gui/bias.h>
#include <qb_legs_gui/data_msg.h>
#include <qb_legs_gui/data_corr_AT.h>
#include <qb_legs_gui/gain_bias_err.h>
#include <std_msgs/Float32.h>
#include <qb_legs_gui/GG_msg.h>

#include <std_msgs/Float64MultiArray.h>
// #include "/home/gian/catkin_ws/src/Qb_Legs_Synergies/synergies/include/synergies/data_in_common.h"


class qt_example: public QWidget
{
Q_OBJECT
public:
    qt_example();
    ~qt_example();
    void inputBias();
    
    void Retrieve_Bias(const qb_legs_gui::bias& msg);
    void Retrieve_Distance(const std_msgs::Float32::ConstPtr& msg);
    void Retrieve_Speed(const std_msgs::Float32::ConstPtr& msg);
    void Retrieve_Err_Tilde(const std_msgs::Float32::ConstPtr& msg);
    void Retrieve_L_Wrench(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void Retrieve_R_Wrench(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void Retrieve_N_Sensor(const std_msgs::Float32::ConstPtr& msg);
    void Retrieve_Distance_I(const std_msgs::Float32::ConstPtr& msg);
    
    QLineEdit * biasLineEdit_read;
    QLineEdit * DistanceLineEdit_read; 
    QLineEdit * SpeedLineEdit_read; 
    
    std::vector<double> bias_in_common;

private Q_SLOTS:
    void on_button_clicked_launch_Syn();
    void on_button_clicked_kill_Syn();
    void on_button_clicked_launch_Dist();
    void on_button_clicked_kill_Dist();
    void on_button_clicked_launch_qb();
    void on_button_clicked_kill_qb();
    void on_button_clicked_pub_bias();
    void on_button_clicked_pub_bias_stop();
    void timer_bias();
    void itwasclicked(bool);
    void itwasclicked_stop(bool);
    void itwasclicked_AT(bool);
    void itwasclicked_quant(bool);
    void on_button_clicked_pub_bias_AT();
    void on_button_clicked_pub_bias_BT();
    void on_button_clicked_pub_pub_GG_ddes();
    void on_button_clicked_pub_stiffness();
    void on_button_clicked_kill_Wrenches();
    void on_button_clicked_launch_Wrenches();
    void on_button_clicked_set_zero_forces();
    void on_button_clicked_pub_N_VmM();
    void change_quant();
    

private:
    QPushButton button_1;
    QHBoxLayout main_layout_1;
    QVBoxLayout main_layout_2;
    QVBoxLayout main_layout_3;
    QPushButton button_2;
    QPushButton button_3;
    QPushButton button_4;
    QPushButton button_5;
    QPushButton button_6;
    
    QPushButton button_force_bias;
    
    QPushButton button_launch_Wrenches;
    QPushButton button_kill_Wrenches;
    
    QPushButton button_AT;
    QPushButton button_BT;
    
    QPushButton button_pub_bias;
    QPushButton button_pub_stopbias;
    
    QPushButton button_pub_GG_ddes;
    
    QPushButton button_pub_stiffness;
    
    QPushButton button_pub_N_VmM;
    
    QLineEdit * LWLineEdit_read;
    QLineEdit * LRLineEdit_read;
    
    QLineEdit * stiffnessLineEdit_write;
    
    QLineEdit * biasLineEdit_write;
    QLineEdit * biasstopLineEdit_write;
    
    QLineEdit * LFLineEdit_read;
    QLineEdit * RFLineEdit_read;
    QLineEdit * NSLineEdit_read;
    
    QLineEdit * ATLineEdit_write;
    QLineEdit * ATLineEdit_1_write;
    QLineEdit * ATLineEdit_2_write;
    
    QLineEdit * BTLineEdit_write;
    
    QLineEdit * errLineEdit_read;
    QLineEdit * ddesLineEdit_write;
    QLineEdit * GGLineEdit_write;
    QLineEdit * IGLineEdit_write;
    
    QLineEdit * NLineEdit_write;
    QLineEdit * VLineEdit_write;
    
    QLineEdit * quantLineEdit_write;
    
//     QLineEdit * biasLineEdit_read;
    QLineEdit * echoLineEdit_1;
    QLineEdit * echoLineEdit_2;
    
    QLineEdit * DistanceLineEdit_I_read;
    QLineEdit * NILineEdit_write;
    
    QTimer *timer;
    
    QGroupBox * createBiasGroup();
    QGroupBox * createNodeGroup();
    QGroupBox * createDistanceGroup();
    QGroupBox * createMiscGroup();
    QGroupBox * createStopBiasGroup();
    QGroupBox * createStiffGroup();
    QGroupBox * createForceGroup();
    
    QFrame myFrame_Bias;
    QFrame myFrame_Node;
    
    
  ros::NodeHandle n;
  
  ros::Subscriber bias_sub;
  ros::Subscriber dist_sub;
  ros::Subscriber speed_sub;
  ros::Subscriber err_tilde_sub;
  ros::Subscriber L_wrench_sub;
  ros::Subscriber R_wrench_sub;
  ros::Subscriber N_wrench_sub;
  ros::Subscriber dist_I_sub;
  
  ros::Publisher bias_pub;
  ros::Publisher bias_stop_pub;
  ros::Publisher BT_pub;
  ros::Publisher AT_pub;
  ros::Publisher pub_bias_stop_flag;
  ros::Publisher pub_ankle_trunk_gain_flag;
  ros::Publisher pub_GG;
  ros::Publisher pub_ddes;
  ros::Publisher pub_stiff;
  ros::Publisher pub_N;
  ros::Publisher pub_N_I;
  ros::Publisher pub_VmM;
  ros::Publisher pub_quant;
  ros::Publisher pub_activate_quant;
  
  
  std::vector<double> bias_mat_R;
std::vector<double> bias_mat_val_R;
int count_R;
bool set_bias_flag_R;
  
//   "rostopic pub -1 /synergies_bias synergies/bias '{bias: ["+bias_val+"]}'";
//   ros::Publisher bias_pub=n.advertise<qb_legs_gui::bias>("/synergies_bias",10);
//   ros::Publisher bias_stop_pub=n.advertise<qb_legs_gui::bias>("/synergies_bias_stop",10);
};


// qt_example * p_gui;



