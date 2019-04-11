#include <qb_gui.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <QProcess>
#include <QTextLine>
#include <QTableWidget>

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
#include <iomanip>
#include <QImage>
#include <QPixmap>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <ros/package.h>

// ------------- SUBSCRIBER CALLBACK -------------
void qt_example::Retrieve_Bias(const qb_legs_gui::bias &msg)
{
  QString output;

  for (int i = 0; i < 6; i++)
  {

    output = output + QString::number(msg.bias[i], 'g', 3);
    if (i == 5)
    {
    }
    else
    {
      output = output + ",";
    }
  }

  qt_example::biasLineEdit_read->setText(output);

  return;
}

void qt_example::Retrieve_Distance(const std_msgs::Float32::ConstPtr &msg)
{
  QString output;

  output = QString::number(msg->data, 'g', 3);

  qt_example::DistanceLineEdit_read->setText(output);

  return;
}

void qt_example::Retrieve_Distance_I(const std_msgs::Float32::ConstPtr &msg)
{
  QString output;

  output = QString::number(msg->data, 'g', 3);

  qt_example::DistanceLineEdit_I_read->setText(output);

  return;
}

void qt_example::Retrieve_Speed(const std_msgs::Float32::ConstPtr &msg)
{
  QString output;

  output = QString::number(msg->data, 'g', 4);

  qt_example::SpeedLineEdit_read->setText(output);

  return;
}

void qt_example::Retrieve_Err_Tilde(const std_msgs::Float32::ConstPtr &msg)
{
  QString output;
  output = QString::number(msg->data, 'g', 4);
  qt_example::errLineEdit_read->setText(output);
}

void qt_example::Retrieve_L_Wrench(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  QString output;
  std::vector<double> v;

  for (int i = 0; i < 3; i++)
  {
    output = output + QString::number(msg->data[i], 'g', 3);
    if (i == 2)
    {
    }
    else
    {
      output = output + ",";
    }
  }
  qt_example::LFLineEdit_read->setText(output);
}

void qt_example::Retrieve_R_Wrench(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  QString output;
  std::vector<double> v;

  for (int i = 0; i < 3; i++)
  {
    output = output + QString::number(msg->data[i], 'g', 3);
    if (i == 2)
    {
    }
    else
    {
      output = output + ",";
    }
  }
  qt_example::RFLineEdit_read->setText(output);
}

void qt_example::Retrieve_N_Sensor(const std_msgs::Float32::ConstPtr &msg)
{
  QString output;

  output = QString::number(msg->data, 'g', 3);

  qt_example::NSLineEdit_read->setText(output);
}

void qt_example::on_button_clicked_pub_stiffness()
{
  std::cout << "you clicked! you wanna pub Stiffness!" << std::endl;

  QString S_String = stiffnessLineEdit_write->text();

  std::string S_val = (S_String.toStdString());

  char *bv = strdup(S_val.c_str());
  char *pch = strtok(bv, " ,");

  //     std_msgs::Float32 msg;
  geometry_msgs::Pose msg;

  int i = 0;
  while (pch != NULL)
  {
    if (i == 0)
    {
      msg.position.x = (std::stod(pch));
    }
    if (i == 1)
    {
      msg.position.y = (std::stod(pch));
    }
    if (i == 2)
    {
      msg.position.z = (std::stod(pch));
    }
    if (i == 3)
    {
      msg.orientation.x = (std::stod(pch));
    }
    if (i == 4)
    {
      msg.orientation.y = (std::stod(pch));
    }
    if (i == 5)
    {
      msg.orientation.z = (std::stod(pch));
    }
    i++;

    pch = strtok(NULL, " ,");
  }

  pub_stiff.publish(msg);
}

// ---------- CONSTRUCTOR ----------

qt_example::qt_example() : QWidget()
{
  bias_sub      = n.subscribe("/synergies_bias_echo", 10, &qt_example::Retrieve_Bias, this);
  dist_sub      = n.subscribe("/filtered_distance", 10, &qt_example::Retrieve_Distance, this);
  speed_sub     = n.subscribe("/synergies_des_speed", 10, &qt_example::Retrieve_Speed, this);
  err_tilde_sub = n.subscribe("/bias_trunk_ankles", 10, &qt_example::Retrieve_Err_Tilde, this);
  L_wrench_sub  = n.subscribe("/wrenches/left_wrenches", 10, &qt_example::Retrieve_L_Wrench, this);
  R_wrench_sub  = n.subscribe("/wrenches/right_wrenches", 10, &qt_example::Retrieve_R_Wrench, this);
  N_wrench_sub  = n.subscribe("/wrenches/N", 10, &qt_example::Retrieve_N_Sensor, this);
  dist_I_sub    = n.subscribe("/distance_filtered_I", 10, &qt_example::Retrieve_Distance_I, this);

  pub_VmM       = n.advertise<std_msgs::Float64MultiArray>("/update_min_max_v", 10);
  pub_N         = n.advertise<std_msgs::Float32>("/update_N", 10);
  pub_N_I       = n.advertise<std_msgs::Float64MultiArray>("/update_NI_I", 10);
  pub_quant     = n.advertise<std_msgs::Float32>("/update_quant", 10);
  pub_activate_quant = n.advertise<std_msgs::Bool>("/activate_quant", 10);

  bias_pub      = n.advertise<qb_legs_gui::bias>("/synergies_bias", 10);
  bias_stop_pub = n.advertise<qb_legs_gui::bias>("/synergies_bias_stop", 10);
  BT_pub        = n.advertise<std_msgs::Float32>("/update_BT", 10);
  AT_pub        = n.advertise<qb_legs_gui::gain_bias_err>("/gain_bias_trunk_ankles", 10);

  pub_bias_stop_flag = n.advertise<std_msgs::Bool>("/enable_bias_stop", 10);
  pub_ankle_trunk_gain_flag = n.advertise<std_msgs::Bool>("/enable_gain_ankle_trunk", 10);
  pub_GG        = n.advertise<qb_legs_gui::GG_msg>("/update_G", 10);
  pub_ddes      = n.advertise<std_msgs::Float32>("/update_d_des", 10);
  pub_stiff     = n.advertise<geometry_msgs::Pose>("/qb_legs_stiffness", 10);

  QImage myImage;
  myImage.load(QString::fromStdString(ros::package::getPath("qb_legs_gui") + "/qb3.png"));

  QLabel *myLabel = new QLabel(this);
  myLabel->setPixmap(QPixmap::fromImage(myImage));
  QPixmap p = QPixmap::fromImage(myImage); // load pixmap

  // get label dimensions
  int w = myLabel->width();
  int h = myLabel->height();

  // set a scaled pixmap to a w x h window keeping its aspect ratio
  myLabel->setPixmap(p.scaled(w * 9, h * 9, Qt::KeepAspectRatio));

  QGridLayout *grid = new QGridLayout;

  QGridLayout *grid_1 = new QGridLayout;
  QGridLayout *grid_2 = new QGridLayout;
  QGridLayout *n_grid_2 = new QGridLayout;

  grid->addLayout(grid_1, 0, 0);
  grid->addLayout(grid_2, 1, 0);

  int count = 0;
  int count_r = 0;
  grid_1->addWidget(myLabel, count_r, count++);
  grid_1->addWidget(createNodeGroup(), count_r, count);
  count = 0;
  count_r = 0;
  grid_2->addWidget(createDistanceGroup(), count_r, count++);

  grid_2->addWidget(createBiasGroup(), count_r, count++);
  grid_2->addLayout(n_grid_2, count_r, count++);

  n_grid_2->addWidget(createStiffGroup());
  n_grid_2->addWidget(createForceGroup());

  grid_2->addWidget(createMiscGroup(), count_r, count++);

  setLayout(grid);
  setWindowTitle(tr("QbLegs Interface"));

  quantLineEdit_write->setDisabled(1);
}

QGroupBox *qt_example::createForceGroup()
{
  QGroupBox *groupBox = new QGroupBox(tr("Opto Forces"));
  QGridLayout *n_grid = new QGridLayout;

  QLabel *LFLabel = new QLabel(tr("Left Foot"));
  QLabel *RFLabel = new QLabel(tr("Right Foot"));
  QLabel *NSLabel = new QLabel(tr("Sensors 'on' "));

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

  QGridLayout *n_n_grid = new QGridLayout;
  n_grid->addLayout(n_n_grid, 0, 1);

  n_n_grid->addWidget(NSLineEdit_read, 0, 1);
  n_n_grid->addWidget(NSLabel, 0, 0);

  button_force_bias.setText("Bias Forces");
  connect(&button_force_bias, SIGNAL(clicked(bool)), this, SLOT(on_button_clicked_set_zero_forces()));
  n_grid->addWidget(&button_force_bias, 0, 0);
  n_grid->addWidget(LFLabel, 1, 0);
  n_grid->addWidget(RFLabel, 1, 1);
  n_grid->addWidget(LFLineEdit_read, 2, 0);
  n_grid->addWidget(RFLineEdit_read, 2, 1);
  groupBox->setLayout(n_grid);
  return groupBox;
}

void qt_example::on_button_clicked_set_zero_forces()
{

  QString program = "rosservice";
  QStringList arguments;
  arguments << "call"
            << "/wrenches/setZeros";
  std::cout << "service set zero" << std::endl;

  QProcess *myProcess = new QProcess(this);
  myProcess->startDetached(program, arguments);
}

QGroupBox *qt_example::createStiffGroup()
{
  QGroupBox *groupBox = new QGroupBox(tr("Stiffness"));

  QGridLayout *n_grid = new QGridLayout;
  QGridLayout *n_grid_1 = new QGridLayout;
  QGridLayout *n_grid_2 = new QGridLayout;
  n_grid->addLayout(n_grid_1, 0, 0);
  n_grid->addLayout(n_grid_2, 1, 0);

  stiffnessLineEdit_write = new QLineEdit();
  stiffnessLineEdit_write->setText("35,35,35,35,35,35");
  stiffnessLineEdit_write->setCursorPosition(0);
  stiffnessLineEdit_write->setFocus();
  stiffnessLineEdit_write->sizeHint();

  button_pub_stiffness.setText("Pub Stiffness");
  button_pub_stiffness.sizeHint();
  connect(&button_pub_stiffness, SIGNAL(clicked(bool)), this, SLOT(on_button_clicked_pub_stiffness()));
  n_grid->addWidget(stiffnessLineEdit_write);
  n_grid->addWidget(&button_pub_stiffness);

  groupBox->setLayout(n_grid);

  return groupBox;
}

QGroupBox *qt_example::createStopBiasGroup()
{
  QGroupBox *groupBox = new QGroupBox(tr("Stop Bias"));

  QRadioButton *Stop_radio = new QRadioButton(tr("&Stop Bias"));

  Stop_radio->setChecked(false);
  connect(Stop_radio, SIGNAL(toggled(bool)), this, SLOT(itwasclicked_stop(bool)));

  biasstopLineEdit_write = new QLineEdit(tr("Stop Bias"));
  biasstopLineEdit_write->setText("0,0,17,-2,0,19");
  biasstopLineEdit_write->setCursorPosition(0);
  biasstopLineEdit_write->setFocus();

  button_pub_stopbias.setText("Pub Stop Bias");
  connect(&button_pub_stopbias, SIGNAL(clicked(bool)), this, SLOT(on_button_clicked_pub_bias_stop()));

  biasstopLineEdit_write->setDisabled(1);

  button_pub_stopbias.setEnabled(0);

  QGridLayout *n_grid = new QGridLayout;
  QLabel *Label_B = new QLabel(tr(" Stop Bias "));

  n_grid->addWidget(Stop_radio, 0, 0);
  n_grid->addWidget(biasstopLineEdit_write, 1, 0);
  n_grid->addWidget(&button_pub_stopbias, 2, 0);

  groupBox->setLayout(n_grid);

  return groupBox;
}

QGroupBox *qt_example::createNodeGroup()
{
  QGroupBox *groupBox = new QGroupBox(tr("Node Management"));

  button_1.setText("Launch Distance Sensor");
  button_2.setText("Kill Distance Sensor");
  button_3.setText("Launch Synergies");
  button_4.setText("Kill Synergies");
  button_5.setText("Launch qb");
  button_6.setText("Kill qb");

  button_launch_Wrenches.setText("Launch Opto");
  button_kill_Wrenches.setText("Kill Opto");

  connect(&button_1, SIGNAL(clicked(bool)), this, SLOT(on_button_clicked_launch_Dist()));
  connect(&button_2, SIGNAL(clicked(bool)), this, SLOT(on_button_clicked_kill_Dist()));
  connect(&button_3, SIGNAL(clicked(bool)), this, SLOT(on_button_clicked_launch_Syn()));
  connect(&button_4, SIGNAL(clicked(bool)), this, SLOT(on_button_clicked_kill_Syn()));
  connect(&button_5, SIGNAL(clicked(bool)), this, SLOT(on_button_clicked_launch_qb()));
  connect(&button_6, SIGNAL(clicked(bool)), this, SLOT(on_button_clicked_kill_qb()));

  connect(&button_launch_Wrenches, SIGNAL(clicked(bool)), this, SLOT(on_button_clicked_launch_Wrenches()));
  connect(&button_kill_Wrenches, SIGNAL(clicked(bool)), this, SLOT(on_button_clicked_kill_Wrenches()));

  QGridLayout *n_grid = new QGridLayout;

  n_grid->addWidget(&button_1, 0, 0);
  n_grid->addWidget(&button_2, 0, 1);
  n_grid->addWidget(&button_3, 1, 0);
  n_grid->addWidget(&button_4, 1, 1);
  n_grid->addWidget(&button_5, 2, 0);
  n_grid->addWidget(&button_6, 2, 1);
  n_grid->addWidget(&button_launch_Wrenches, 3, 0);
  n_grid->addWidget(&button_kill_Wrenches, 3, 1);

  QRadioButton *radio1 = new QRadioButton(tr("&Save Signals"));

  radio1->setChecked(false);
  connect(radio1, SIGNAL(toggled(bool)), this, SLOT(itwasclicked(bool)));
  n_grid->addWidget(radio1, 1, 3);

  groupBox->setLayout(n_grid);

  return groupBox;
}

QGroupBox *qt_example::createBiasGroup()
{

  QGroupBox *groupBox = new QGroupBox(tr("Bias"));
  QLabel *BiasLabel_C = new QLabel(tr(" C. Bias:"));
  QLabel *BiasLabel_D = new QLabel(tr(" D. Bias:"));

  biasLineEdit_read = new QLineEdit;
  biasLineEdit_read->setPlaceholderText("Placeholder Text");
  biasLineEdit_read->setReadOnly(true);
  biasLineEdit_read->setFocus();

  biasLineEdit_write = new QLineEdit;
  //     biasLineEdit_write->setInputMask("xxx,xxx,xxx,xxx,xxx,xxx");
  biasLineEdit_write->setText("-1,0,15,-3,0,16");
  biasLineEdit_write->setCursorPosition(0);
  biasLineEdit_write->setFocus();

  button_pub_bias.setText("Pub Des Bias");
  connect(&button_pub_bias, SIGNAL(clicked(bool)), this, SLOT(on_button_clicked_pub_bias()));

  QGridLayout *n_grid = new QGridLayout;

  QGridLayout *n_grid_1 = new QGridLayout;
  QGridLayout *n_grid_2 = new QGridLayout;
  QGridLayout *n_grid_3 = new QGridLayout;
  n_grid->addLayout(n_grid_1, 0, 0);
  n_grid->addLayout(n_grid_2, 1, 0);

  //     n_grid_2->addWidget(createStiffGroup());
  n_grid_2->addWidget(createStopBiasGroup());

  n_grid_1->addWidget(BiasLabel_C);
  n_grid_1->addWidget(biasLineEdit_read);
  n_grid_1->addWidget(BiasLabel_D);
  n_grid_1->addWidget(biasLineEdit_write);
  n_grid_1->addWidget(&button_pub_bias);

  timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(timer_bias()));
  timer->start(10);
  timer_bias();

  groupBox->setLayout(n_grid);

  return groupBox;
}

QGroupBox *qt_example::createDistanceGroup()
{
  QGroupBox *groupBox = new QGroupBox(tr("Distance"));

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

  SpeedLineEdit_read = new QLineEdit;
  SpeedLineEdit_read->setPlaceholderText("Placeholder Text");
  SpeedLineEdit_read->setReadOnly(true);
  SpeedLineEdit_read->setFocus();
  SpeedLineEdit_read->setMaximumWidth(80);

  QGridLayout *n_grid = new QGridLayout;

  QGridLayout *n_grid_3 = new QGridLayout;
  QGridLayout *n_grid_4 = new QGridLayout;

  QLabel *Label_D_I = new QLabel(tr(" Dist I: "));
  n_grid_3->addWidget(Label_D, 0, 0);
  n_grid_3->addWidget(DistanceLineEdit_read, 0, 1);
  n_grid_3->addWidget(Label_D_I, 0, 2);
  n_grid_3->addWidget(DistanceLineEdit_I_read, 0, 3);

  n_grid_4->addWidget(Label_S, 0, 0);
  n_grid_4->addWidget(SpeedLineEdit_read, 0, 1);
  QGridLayout *n_grid_1 = new QGridLayout;
  QGridLayout *n_grid_2 = new QGridLayout;

  QLabel *Label_N = new QLabel(tr(" N: "));
  QLabel *Label_N_I = new QLabel(tr(" NI: "));

  NLineEdit_write = new QLineEdit;
  NLineEdit_write->setPlaceholderText("30");
  NLineEdit_write->setFocus();
  NLineEdit_write->setMaximumWidth(40);

  QLabel *Label_VmM = new QLabel(tr(" Vmin, Vmax: "));

  VLineEdit_write = new QLineEdit;
  VLineEdit_write->setPlaceholderText("0.065,0.18");

  VLineEdit_write->setMaximumWidth(80);
  VLineEdit_write->setCursorPosition(0);
  VLineEdit_write->setFocus();

  n_grid_1->addWidget(Label_N, 0, 0);
  n_grid_1->addWidget(NLineEdit_write, 0, 1);
  n_grid_1->addWidget(Label_N_I, 0, 2);
  n_grid_1->addWidget(NILineEdit_write, 0, 3);

  n_grid_4->addWidget(Label_VmM, 1, 0);
  n_grid_4->addWidget(VLineEdit_write, 1, 1);

  QGridLayout *n_grid_5 = new QGridLayout;

  button_pub_N_VmM.setText("Pub N minV maxV");
  connect(&button_pub_N_VmM, SIGNAL(clicked(bool)), this, SLOT(on_button_clicked_pub_N_VmM()));

  n_grid_5->addWidget(&button_pub_N_VmM);

  QGridLayout *n_grid_6 = new QGridLayout;

  quantLineEdit_write = new QLineEdit;
  quantLineEdit_write->setPlaceholderText("0.005");
  quantLineEdit_write->setFocus();
  quantLineEdit_write->setMaximumWidth(80);

  connect(quantLineEdit_write, SIGNAL(returnPressed()), this, SLOT(change_quant()));

  QRadioButton *quant_radio = new QRadioButton(tr("&Activate quant"));
  quant_radio->setChecked(false);

  connect(quant_radio, SIGNAL(toggled(bool)), this, SLOT(itwasclicked_quant(bool)));

  n_grid_6->addWidget(quant_radio, 0, 0);
  n_grid_6->addWidget(quantLineEdit_write, 0, 1);
  //
  n_grid->addLayout(n_grid_3, 1, 0);
  n_grid->addLayout(n_grid_4, 3, 0);
  n_grid->addLayout(n_grid_1, 2, 0);
  n_grid->addLayout(n_grid_2, 3, 1);
  n_grid->addLayout(n_grid_5, 4, 0);
  n_grid->addLayout(n_grid_6, 5, 0);

  groupBox->setLayout(n_grid);

  return groupBox;
}

QGroupBox *qt_example::createMiscGroup()
{
  QGroupBox *groupBox = new QGroupBox(tr("Miscellaneous"));

  QRadioButton *AT_radio = new QRadioButton(tr("&Ankle Trunk"));

  AT_radio->setChecked(false);

  connect(AT_radio, SIGNAL(toggled(bool)), this, SLOT(itwasclicked_AT(bool)));

  QLabel *Label_A_1 = new QLabel(tr(" T_g, T_M, T_m:"));
  QLabel *Label_A_2 = new QLabel(tr(" A_g, A_M, A_m:"));
  QLabel *Label_S = new QLabel(tr(" BT:"));
  QLabel *Label_err = new QLabel(tr(" Instant Speed: "));
  QLabel *Label_d = new QLabel(tr(" Desired distance: "));
  QLabel *Label_G = new QLabel(tr(" Distance/Robot_Speed Gain: "));

  QLabel *Label_P = new QLabel(tr(" PG:"));
  QLabel *Label_I = new QLabel(tr(" IG:"));

  ATLineEdit_1_write = new QLineEdit;
  ATLineEdit_1_write->setText("-1,0,17");
  ATLineEdit_1_write->setCursorPosition(0);
  ATLineEdit_1_write->setFocus();
  ATLineEdit_1_write->setDisabled(1);

  ATLineEdit_2_write = new QLineEdit;
  ATLineEdit_2_write->setText("-3,0,18");
  ATLineEdit_2_write->setCursorPosition(0);
  ATLineEdit_2_write->setFocus();
  ATLineEdit_2_write->setDisabled(1);

  BTLineEdit_write = new QLineEdit;
  BTLineEdit_write->setText("0");
  BTLineEdit_write->setCursorPosition(0);
  BTLineEdit_write->setFocus();

  errLineEdit_read = new QLineEdit;
  errLineEdit_read->setPlaceholderText("Placeholder Text");
  errLineEdit_read->setReadOnly(true);
  errLineEdit_read->setFocus();

  ddesLineEdit_write = new QLineEdit;
  ddesLineEdit_write->setText("40");
  ddesLineEdit_write->setCursorPosition(0);
  ddesLineEdit_write->setFocus();

  GGLineEdit_write = new QLineEdit;
  GGLineEdit_write->setText("0.0024");
  GGLineEdit_write->setCursorPosition(0);
  GGLineEdit_write->setFocus();
  GGLineEdit_write->setMaximumWidth(80);

  IGLineEdit_write = new QLineEdit;
  IGLineEdit_write->setText("0.0");
  IGLineEdit_write->setCursorPosition(0);
  IGLineEdit_write->setFocus();
  IGLineEdit_write->setMaximumWidth(80);

  button_pub_GG_ddes.setText("Pub Gain and des dist");
  connect(&button_pub_GG_ddes, SIGNAL(clicked(bool)), this, SLOT(on_button_clicked_pub_pub_GG_ddes()));

  button_AT.setText("Pub Ankle Trunk gain");
  button_AT.setEnabled(0);
  connect(&button_BT, SIGNAL(clicked(bool)), this, SLOT(on_button_clicked_pub_bias_BT()));
  connect(&button_AT, SIGNAL(clicked(bool)), this, SLOT(on_button_clicked_pub_bias_AT()));

  QGridLayout *n_grid = new QGridLayout;
  QGridLayout *n_sub_grid_1 = new QGridLayout;
  QGridLayout *n_sub_grid_2_1 = new QGridLayout;
  QGridLayout *n_sub_grid_2 = new QGridLayout;
  QGridLayout *n_sub_grid_2_2 = new QGridLayout;
  QGridLayout *n_sub_grid_2_0 = new QGridLayout;

  n_grid->addLayout(n_sub_grid_1, 0, 0);
  n_grid->addLayout(n_sub_grid_2, 0, 1);

  n_sub_grid_2->addLayout(n_sub_grid_2_1, 1, 0);
  n_sub_grid_2->addLayout(n_sub_grid_2_2, 2, 0);
  n_sub_grid_2->addLayout(n_sub_grid_2_0, 0, 0);

  int count_r = 0;
  int count_c = 0;
  n_sub_grid_1->addWidget(AT_radio, count_r++, count_c);
  n_sub_grid_1->addWidget(Label_A_1, count_r++, count_c);
  n_sub_grid_1->addWidget(ATLineEdit_1_write, count_r++, count_c);
  n_sub_grid_1->addWidget(Label_A_2, count_r++, count_c);
  n_sub_grid_1->addWidget(ATLineEdit_2_write, count_r++, count_c);
  n_sub_grid_1->addWidget(&button_AT, count_r++, count_c);

  count_r = 0;
  count_c = 0;
  n_sub_grid_2_0->addWidget(Label_err, count_r++, count_c);
  n_sub_grid_2_0->addWidget(errLineEdit_read, count_r - 1, count_c + 1);
  n_sub_grid_2_0->addWidget(Label_d, count_r++, count_c);
  n_sub_grid_2_0->addWidget(ddesLineEdit_write, count_r - 1, count_c + 1);
  //   n_sub_grid_2_0->addWidget(Label_G,count_r++,count_c);

  n_sub_grid_2_1->addWidget(Label_P, 0, 0);
  n_sub_grid_2_1->addWidget(GGLineEdit_write, 0, 1);
  n_sub_grid_2_1->addWidget(Label_I, 0, 2);
  n_sub_grid_2_1->addWidget(IGLineEdit_write, 0, 3);

  n_sub_grid_2_2->addWidget(&button_pub_GG_ddes, count_r++, count_c);

  groupBox->setLayout(n_grid);
  return groupBox;
}

void qt_example::on_button_clicked_launch_Dist()
{
  std::cout << "you clicked! you wanna launch nodes !" << std::endl;

  QString program = "roslaunch";
  QStringList arguments;
  arguments << "Distance_Sensor"
            << "distance.launch";

  QProcess *myProcess = new QProcess(this);
  myProcess->startDetached(program, arguments);
}

void qt_example::on_button_clicked_launch_qb()
{
  std::cout << "you clicked! you wanna launch nodes !" << std::endl;

  QString program = "roslaunch";
  QStringList arguments;

  arguments << "qb_legs_controller"
            << "qb_legs_controller.launch";

  QProcess *myProcess = new QProcess(this);
  myProcess->startDetached(program, arguments);
}

void qt_example::on_button_clicked_kill_qb()
{
  std::cout << "you clicked! you wanna kill nodes!" << std::endl;

  QString program = "rosnode";
  QStringList arguments;

  arguments << "kill"
            << "/qb_legs_controller";

  QProcess *myProcess = new QProcess(this);
  myProcess->startDetached(program, arguments);
  myProcess->kill();
}

void qt_example::on_button_clicked_kill_Dist()
{
  std::cout << "you clicked! you wanna kill nodes!" << std::endl;
  //     std::string cmd="xterm ";
  //     system(cmd.c_str());

  QString program = "rosnode";
  QStringList arguments;
  arguments << "kill"
            << "/Desired_Velocity_Calculus";

  QProcess *myProcess = new QProcess(this);
  myProcess->startDetached(program, arguments);
  myProcess->kill();
}

void qt_example::on_button_clicked_launch_Syn()
{
  std::cout << "you clicked! you wanna launch nodes !" << std::endl;

  QString program = "roslaunch";
  QStringList arguments;
  arguments << "synergies"
            << "synergies.launch";

  QProcess *myProcess = new QProcess(this);
  myProcess->startDetached(program, arguments);
  myProcess->kill();
}

void qt_example::on_button_clicked_kill_Syn()
{
  std::cout << "you clicked! you wanna kill nodes!" << std::endl;
  //     std::string cmd="xterm ";
  //     system(cmd.c_str());

  QString program = "rosnode";
  QStringList arguments;
  arguments << "kill"
            << "/synergies_qb_legs_control";

  QProcess *myProcess = new QProcess(this);
  myProcess->startDetached(program, arguments);
  myProcess->kill();
}

void qt_example::on_button_clicked_pub_bias()
{
  std::cout << "you clicked! you wanna pub bias!" << std::endl;

  QString Bias_String = biasLineEdit_write->text();

  std::string bias_val = (Bias_String.toStdString());

  char *bv = strdup(bias_val.c_str());
  char *pch = strtok(bv, " ,");
  std::vector<double> ppp;
  qb_legs_gui::bias msg;

  int i = 0;
  while (pch != NULL)
  {
    msg.bias[i] = (std::stod(pch));
    i++;
    //       std::cout<<"n "<<i<<" "<<std::stod(pch)<<std::endl;
    pch = strtok(NULL, " ,");
  }

  bias_pub.publish(msg);

  return;
}

void qt_example::on_button_clicked_pub_bias_stop()
{
  std::cout << "you clicked! you wanna pub stop bias!" << std::endl;

  QString Bias_String = biasLineEdit_write->text();

  std::string bias_val = (Bias_String.toStdString());

  char *bv = strdup(bias_val.c_str());
  char *pch = strtok(bv, " ,");
  std::vector<double> ppp;
  qb_legs_gui::bias msg;

  int i = 0;
  while (pch != NULL)
  {
    msg.bias[i] = (std::stod(pch));
    i++;
    //       std::cout<<"n "<<i<<" "<<std::stod(pch)<<std::endl;
    pch = strtok(NULL, " ,");
  }

  bias_stop_pub.publish(msg);
}

void qt_example::timer_bias()
{

  ros::spinOnce();

  return;
}

void qt_example::itwasclicked(bool checked)
{
  if (checked)
  {
    QString program = "roslaunch";
    QStringList arguments;
    arguments << "save_qb_legs"
              << "save_qb_legs.launch";

    QProcess *myProcess = new QProcess(this);
    myProcess->startDetached(program, arguments);
  }
  else
  {
    QString program = "rosnode";
    QStringList arguments;
    arguments << "kill"
              << "/save_qb_legs";

    QProcess *myProcess = new QProcess(this);
    myProcess->startDetached(program, arguments);
  }
}

void qt_example::on_button_clicked_pub_bias_AT()
{

  std::cout << "you clicked! you wanna pub AT!" << std::endl;

  QString AT_String_1 = ATLineEdit_1_write->text();
  QString AT_String_2 = ATLineEdit_2_write->text();
  std::string AT_val = (AT_String_1.toStdString()) + ", " + (AT_String_2.toStdString());
  //     std::string AT_val =(AT_String_1.toStdString());
  char *bv = strdup(AT_val.c_str());
  char *pch = strtok(bv, " ,");

  qb_legs_gui::gain_bias_err msg;
  msg.gain_bias_err.resize(6);

  int i = 0;
  while (pch != NULL)
  {
    //       msg.gain_bias_err[i]=(std::stod(pch));
    //       ppp[i]=std::stod(pch);
    msg.gain_bias_err[i] = std::stod(pch);
    //       std::cout<<std::to_string(msg.gain_bias_err[i])<<std::endl;
    i++;
    pch = strtok(NULL, " ,");
  }

  AT_pub.publish(msg);
}

void qt_example::on_button_clicked_pub_bias_BT()
{

  std::cout << "you clicked! you wanna pub BT!" << std::endl;

  QString BT_String = BTLineEdit_write->text();

  std::string BT_val = (BT_String.toStdString());

  char *bv = strdup(BT_val.c_str());
  char *pch = strtok(bv, " ,");

  std_msgs::Float32 msg;

  int i = 0;
  while (pch != NULL)
  {
    msg.data = (std::stod(pch));
    i++;
    //       std::cout<<"n "<<i<<" "<<std::stod(pch)<<std::endl;
    pch = strtok(NULL, " ,");
  }

  BT_pub.publish(msg);
}

void qt_example::on_button_clicked_pub_pub_GG_ddes()
{
  QString GG_String = GGLineEdit_write->text();
  std::string GG_val = (GG_String.toStdString());
  double GG = GG_String.toDouble();
  //   std::cout<<GG_val<<std::endl;
  //   GG_val+=" , ";
  //    std::cout<<GG_val<<" "<<GG<<std::endl;
  char *bv = strdup(GG_val.c_str());
  char *pch = strtok(bv, " ,");
  //
  qb_legs_gui::GG_msg msg;
  //
  int i = 0;

  msg.gain = GG;
  //   std::cout<<" "<<msg.gain<<std::endl;
  pub_GG.publish(msg);

  QString dd_String = ddesLineEdit_write->text();
  std::string dd_val = (dd_String.toStdString());

  bv = strdup(dd_val.c_str());
  pch = strtok(bv, " ,");

  std_msgs::Float32 msg_2;
  i = 0;
  while (pch != NULL)
  {
    msg_2.data = (std::stod(pch));
    i++;
    pch = strtok(NULL, " ,");
  }

  pub_ddes.publish(msg_2);

  QString N_I_String = NILineEdit_write->text();
  std_msgs::Float64MultiArray msg_n_I;
  msg_n_I.data.resize(2);
  msg_n_I.data[0] = N_I_String.toDouble();

  QString GI_String = IGLineEdit_write->text();
  msg_n_I.data[1] = GI_String.toDouble();

  pub_N_I.publish(msg_n_I);
}

void qt_example::itwasclicked_stop(bool val)
{
  std_msgs::Bool msg;

  if (val)
  {
    msg.data = true;
  }
  else
  {
    msg.data = false;
  }
  pub_bias_stop_flag.publish(msg);
  ros::spinOnce();

  if (val)
  {
    biasstopLineEdit_write->setDisabled(0);

    button_pub_stopbias.setEnabled(1);
  }
  else
  {
    biasstopLineEdit_write->setDisabled(1);

    button_pub_stopbias.setEnabled(0);
  }
}

void qt_example::itwasclicked_quant(bool val)
{
  std_msgs::Bool msg;
  msg.data = val;

  if (val)
  {
    quantLineEdit_write->setDisabled(0);
  }
  else
  {
    quantLineEdit_write->setDisabled(1);
  }

  pub_activate_quant.publish(msg);
}

void qt_example::itwasclicked_AT(bool val)
{
  std_msgs::Bool msg;

  if (val)
  {
    msg.data = true;
    //     std::cout<<" gui TRUE "<<std::endl;
  }
  else
  {
    msg.data = false;
  }
  //   std::cout<<" flag "<<val<<std::endl;
  pub_ankle_trunk_gain_flag.publish(msg);
  ros::spinOnce();

  if (val)
  {
    ATLineEdit_2_write->setDisabled(0);
    ATLineEdit_1_write->setDisabled(0);

    button_AT.setEnabled(1);
  }
  else
  {
    ATLineEdit_2_write->setDisabled(1);
    ATLineEdit_1_write->setDisabled(1);

    button_AT.setEnabled(0);
  }
}

void qt_example::on_button_clicked_kill_Wrenches()
{
  QString program = "rosnode";
  QStringList arguments;
  arguments << "kill"
            << "/wrenches";

  QProcess *myProcess = new QProcess(this);
  myProcess->startDetached(program, arguments);
}

void qt_example::on_button_clicked_launch_Wrenches()
{

  QString program = "roslaunch";
  QStringList arguments;
  arguments << "opto_wrenches"
            << "opto_wrenches.launch";

  QProcess *myProcess = new QProcess(this);
  myProcess->startDetached(program, arguments);
}

void qt_example::on_button_clicked_pub_N_VmM()
{

  QString V_String = VLineEdit_write->text();

  std::string V_val = (V_String.toStdString());
  std::cout << V_val << V_String.toDouble() << std::endl;

  std::string::size_type sz;
  char *bv = strdup(V_val.c_str());
  char *pch = strtok(bv, " ,");

  std_msgs::Float64MultiArray msg_v;
  std_msgs::Float32 msg_n;
  msg_v.data.resize(2);
  int i = 0;

  QString q;
  while (pch != NULL)
  { 
    q = pch;
    msg_v.data[i] = q.toDouble();
    i++;
    pch = strtok(NULL, " ,");
  }

  QString N_String = NLineEdit_write->text();
  msg_n.data = N_String.toDouble();

  std::cout << msg_n.data << " pre" << std::endl;
  QString N_I_String = NILineEdit_write->text();
  std_msgs::Float64MultiArray msg_n_I;
  msg_n_I.data.resize(2);
  msg_n_I.data[0] = N_I_String.toDouble();

  QString GI_String = IGLineEdit_write->text();
  msg_n_I.data[1] = GI_String.toDouble();
  std::cout << " post" << std::endl;

  std::cout << " post post" << std::endl;

  pub_N.publish(msg_n);
}

void qt_example::change_quant()
{

  QString quantString = quantLineEdit_write->text();
  std_msgs::Float32 msg;
  msg.data = quantString.toDouble();
  std::cout << " wanna change quant " << msg.data << std::endl;
  pub_quant.publish(msg);
}

qt_example::~qt_example()
{ }