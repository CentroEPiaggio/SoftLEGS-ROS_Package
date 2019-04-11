#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <ros/time.h>
#include <std_msgs/Char.h>
#include <fcntl.h>
#include <stdlib.h>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_SPEEDDOWN 0x35
#define KEYCODE_SPEEDUP 0x36
#define KEYCODE_START 0x1B
// #define KEYCODE_STOP 0x2E
#define KEYCODE_UNSET 0x2E


#include <unistd.h>
#include <termios.h>

  char cmd;
ros::Publisher cmd_pub_;
/*
class RemoteCtrl
{
public:
  RemoteCtrl();
  void keyLoop();


  
};

RemoteCtrl::RemoteCtrl()
{
  cmd_pub_ = nh_.advertise<std_msgs::Char>("/logitech_ctrl/cmd", 1);
}*/

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

void keyLoop()
{
  char c;
  bool dirty=false;
  std::vector<char> c_v;
  c_v.resize(5);


  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

//   puts("Reading from keyboard");
//   puts("---------------------------");
//   puts("Use arrow keys to move the turtle.");
int count=0;
int number_of_char(0);



int flags = fcntl(kfd, F_GETFL, 0);
fcntl(kfd, F_SETFL, flags | O_NONBLOCK);
int c_2;
std_msgs::Char cmd_msgs;
  for(;;)
  {

    count=0;
    c_2=0;
    c=0x10;
    c_v={c,c,c,c,c};
    
    
    while(number_of_char<=0){
      number_of_char=read(kfd, &c,1);
    }
    
    while(number_of_char==1){
      
      if(count==0) {c_v[count]=c; 
// 	ROS_INFO("qui value: 0x%02X\n", c_v[count]); 
	count++;}
      else{
      if(c_v[count-1]!=c){
       c_v[count]=c;
//        ROS_INFO("qui value: 0x%02X\n", c_v[count]);
       count++;
      
      }
      }
//        std::cout<<"N "<<count<<std::endl;
number_of_char=read(kfd, &c,1);
       
    }
//     std::cout<<"N "<<count<<std::endl;


      switch(count)
      {
	case 5:
	  std::cout<<"count "<<count<<" STOP "<<std::endl;
	  count=0;
	   c_v.clear();
	   cmd='S';
	  break;
	  
	case 4:
// 	  ROS_INFO("value: 0x%02X\n", c_v[2]);
	  switch(c_v[2])
	  {
	    case KEYCODE_SPEEDUP:
	    std::cout<<"count "<<count<<" Speed UP"<<std::endl;
	    count=0;
	     c_v.clear();
	     cmd='U';
	    break;
	    case KEYCODE_SPEEDDOWN:
	    std::cout<<"count "<<count<<" Speed DOWN"<<std::endl;
	    count=0;
	     c_v.clear();
	     cmd='D';
	    break;
	  }
	  count=0;
	   c_v.clear();
	  break;
	case 1:
// 	  ROS_INFO("value: 0x%02X\n", c_v[0]);
	  switch(c_v[0])
	  {
	    case KEYCODE_UNSET:
	      std::cout<<"count "<<count<<" Unset"<<std::endl;
	      count=0;
	       c_v.clear();
	      break;
	    case KEYCODE_START:
	      std::cout<<"count "<<count<<" START"<<std::endl;
	      count=0;
	       c_v.clear();
	       cmd='G';
	      break;
	  }
	  count=0;
	   c_v.clear();
	  break;
      }
      
      count=0;
      c_v.clear();
      
	cmd_msgs.data=cmd;
cmd_pub_.publish(cmd_msgs);  
  }

  return;
}


int main(int argc, char** argv)
{
  

  ros::init(argc, argv, "logitech_ctrl");
    ros::NodeHandle nh_;
    
//   RemoteCtrl logitech_ctrl;
//   ros::publisher p
  cmd_pub_ = nh_.advertise<std_msgs::Char>("/logitech_ctrl/cmd", 1);

  signal(SIGINT,quit);

  keyLoop();
  
  return(0);
}





