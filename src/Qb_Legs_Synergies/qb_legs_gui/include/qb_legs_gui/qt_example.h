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
#include </lib/eigen-eigen/Eigen/Dense>
#include <vector>
// #include "/home/gian/catkin_ws/src/Qb_Legs_Synergies/synergies/include/synergies/data_in_common.h"

class qt_example: public QWidget
{
Q_OBJECT
public:
    qt_example();
    ~qt_example();
    void inputBias();
    int cane;
    QLineEdit * biasLineEdit_read;
    
    std::vector<double> bias_in_common;

private Q_SLOTS:
    void on_button_clicked_launch_Syn();
    void on_button_clicked_kill_Syn();
    void on_button_clicked_launch_Dist();
    void on_button_clicked_kill_Dist();
    void on_button_clicked_launch_qb();
    void on_button_clicked_kill_qb();
    void on_button_clicked_pub_bias();
    void timer_bias();
    

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
    QPushButton button_pub_bias;
    QLineEdit * biasLineEdit_write;
//     QLineEdit * biasLineEdit_read;
    QLineEdit * echoLineEdit_1;
    QLineEdit * echoLineEdit_2;
    QTimer *timer;
//     std::string str1,str2;
};