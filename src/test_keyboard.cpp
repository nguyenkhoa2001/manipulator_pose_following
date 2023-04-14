#include <ros/ros.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

//Function Declaration
double Convert(double degree)
{
    double pi = 3.14159265359;
    return (degree * (pi / 180));
}

char getch_async();

int g_rate = 40;
double g_velocity_jogging=0.2;
double g_vel_cart=0.5;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_keyboard");
  ros::NodeHandle nh;
  nh.getParam("keyboard_cart/rate",g_rate);
  nh.getParam("keyboard_cart/velocity_jogging",g_velocity_jogging);
  nh.getParam("keyboard_cart/velocity_cart",g_vel_cart);
  if(g_vel_cart>0.5)
    g_vel_cart=0.5;
  ros::Publisher pub=nh.advertise<sensor_msgs::JointState>("pose_vel",10);
  ros::Rate loop_rate(g_rate);
  ROS_INFO("This is tester for keyboard moving!");
  ROS_INFO("Keyboard jogging online.");
  ROS_INFO_STREAM("\n"
                  << "\n"
                  << "Key assignments:\n"
                  << "\n"
                  << "Translation: +x: \"1\" | +y: \"2\" | +z: \"3\" |+rx: \"4\" | +ry: \"5\" | +rz: \"6\"\n"
                  << "             -x: \"q\" | -y: \"w\" | -z: \"e\" |-rx: \"r\" | -ry: \"t\" | -rz: \"y\"\n"
                  << "\n"
                  << "Velocity:     +: \"0\"\n"
                  << "              -: \"p\"\n"
                  << "Idle state:    : \"a\"\n"
                  << "Stop state:    : \"s\"\n"
                  << "Stop:            \"x\"");

  bool stop = false;
  system("/bin/stty raw");   // Raw mode (send all keystrokes directly to stdin)
  system("/bin/stty -echo"); // Turn off echo
  bool sstop=false;
  sensor_msgs::JointState jointVel;
  jointVel.velocity.resize(6);
  while(ros::ok() &&!stop)
  {  

    char command = getch_async();
        switch (command) {
        case '1':
          jointVel.velocity.at(0)= g_vel_cart*g_velocity_jogging;
          break;
        case '2':
         jointVel.velocity.at(1)=g_vel_cart*g_velocity_jogging;
          break;
        case '3':
          jointVel.velocity.at(2)=g_vel_cart*g_velocity_jogging;
          break;
        case '4':
          jointVel.velocity.at(3)=g_vel_cart*g_velocity_jogging;
          break;
        case '5':
          jointVel.velocity.at(4)=g_vel_cart*g_velocity_jogging;
          break;
        case '6':
          jointVel.velocity.at(5)=g_vel_cart*g_velocity_jogging;
          break;
        case 'q':
         jointVel.velocity.at(0)=-g_vel_cart*g_velocity_jogging;
          break;
        case 'w':
         jointVel.velocity.at(1)=-g_vel_cart*g_velocity_jogging;
          break;
        case 'e':
          jointVel.velocity.at(2)=-g_vel_cart*g_velocity_jogging;
          break;
        case 'r':
          jointVel.velocity.at(3)=-g_vel_cart*g_velocity_jogging;
          break;
        case 't':
          jointVel.velocity.at(4)=-g_vel_cart*g_velocity_jogging;
          break;
        case 'y':
           jointVel.velocity.at(5)=-g_vel_cart*g_velocity_jogging;
          break;
        case '0':
           g_velocity_jogging+=0.05;
           if (g_velocity_jogging>0.5)
             g_velocity_jogging=0.5;
          ROS_INFO("Jogging velocity factor: %1.5lf\r", g_velocity_jogging);
          break;
        case 'p':
          g_velocity_jogging-=0.05;
          if (g_velocity_jogging<0)
            g_velocity_jogging=0;
          ROS_INFO("Jogging velocity factor: %1.5lf\r", g_velocity_jogging);
          break;
        case 'x':
          stop=true;
          break;
        case 'a':
          sstop=false;
          break;
        case 's':
   
          sstop=true;
          break;
        default:

          break;
        }
       pub.publish(jointVel);
        loop_rate.sleep();

  }
  system("/bin/stty cooked");
  system("/bin/stty echo");
  return 0;
}

char getch_async() {
  // https://answers.ros.org/question/63491/keyboard-key-pressed/
  fd_set set;
  struct timeval timeout;
  int rv;
  char buff = 0;
  int len = 1;
  int filedesc = 0;
  FD_ZERO(&set);
  FD_SET(filedesc, &set);

  timeout.tv_sec = 0;
  timeout.tv_usec = g_rate*1000;

  rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

  struct termios oldattr = {0};
  struct termios newattr = {0};
  if (tcgetattr(filedesc, &oldattr) < 0)
    printf("tcsetattr()");

  newattr = oldattr;
  newattr.c_lflag &= ~ICANON;
  newattr.c_lflag &= ~ECHO;
  newattr.c_lflag |= ECHONL;
  newattr.c_lflag |= (OCRNL | ONLCR);
  newattr.c_cc[VMIN] = 1;
  newattr.c_cc[VTIME] = 0;
  if (tcsetattr(filedesc, TCSANOW, &newattr) < 0)
    printf("tcsetattr ICANON");

  if (rv == -1)
    printf("select");
  else if (rv == 0)
    buff = -1;
  else
    read(filedesc, &buff, len);

  if (tcsetattr(filedesc, TCSADRAIN, &oldattr) < 0)
    printf("tcsetattr ~ICANON");
  return (buff);
}
