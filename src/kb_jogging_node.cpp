
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

// Globals
double g_jogging_velocity;
int g_rate_hz = 40;

// --- Function declarations
char getch_async(); // Non-blocking getch_async()

int main(int argc, char **argv) {

  ros::init(argc, argv, "kb_jogging");
  ros::NodeHandle n;

  ros::Publisher cmd_vel_fct_pub =
      n.advertise<geometry_msgs::PoseStamped>("pose_following/pose", 1);

  // --- Get params from parameter server
  g_jogging_velocity = 0.001;

  ros::Rate loop_rate_hz(g_rate_hz);
  const double max_cart_translation_vel = .5;
  double cart_translation_vel = max_cart_translation_vel;

  if (cart_translation_vel > max_cart_translation_vel) {
    cart_translation_vel = max_cart_translation_vel;
  }

  const double max_cart_rotation_vel = 1;
  double cart_rotation_vel = max_cart_rotation_vel;
  n.getParam("kb_jogging/max_rot_vel", cart_rotation_vel);
  if (cart_rotation_vel > max_cart_rotation_vel) {
    cart_rotation_vel = max_cart_rotation_vel;
  }

  ROS_INFO("Keyboard jogging online.");
  ROS_INFO_STREAM("\n"
                  << "\n"
                  << "Key assignments:\n"
                  << "\n"
                  << "Translation: +x: \"1\" | +y: \"2\" | +z: \"3\"\n"
                  << "             -x: \"q\" | -y: \"w\" | -z: \"e\"\n"
                  << "\n"
                  << "Rotation:    +x: \"4\" | +y: \"5\" | +z: \"6\"\n"
                  << "             -x: \"r\" | -y: \"t\" | -z: \"y\"\n"
                  << "\n"
                  << "Velocity:     +: \"0\"\n"
                  << "              -: \"p\"\n"
                  << "\n"
                  << "Stop:            \"x\"");

  bool stop = false;

  geometry_msgs::Pose sending_pose;
  geometry_msgs::PoseStamped pose_ref;
  sending_pose.position.x = 0.185;
  sending_pose.position.y = 0.0;
  sending_pose.position.z = 0.228;
  sending_pose.orientation.x = 1.0;
  sending_pose.orientation.y = 0.0;
  sending_pose.orientation.z = 0.0;
  sending_pose.orientation.w = 0.0;

  system("/bin/stty raw");   // Raw mode (send all keystrokes directly to stdin)
  system("/bin/stty -echo"); // Turn off echo

  while (ros::ok() && !stop) {

    char command = getch_async();
    switch (command) {
    case '1':
      sending_pose.position.x += g_jogging_velocity * cart_translation_vel;
      break;
    case '2':
      sending_pose.position.y += g_jogging_velocity * cart_translation_vel;
      break;
    case '3':
      sending_pose.position.z += g_jogging_velocity * cart_translation_vel;
      break;
    case '4':
      break;
    case '5':
      break;
    case '6':
      break;
    case 'q':
      sending_pose.position.x -= g_jogging_velocity * cart_translation_vel;
      break;
    case 'w':
      sending_pose.position.y -= g_jogging_velocity * cart_translation_vel;
      break;
    case 'e':
      sending_pose.position.z -= g_jogging_velocity * cart_translation_vel;
      break;
    case 'r':
      break;
    case 't':
      break;
    case 'y':
      break;
    case '0':
      g_jogging_velocity += 0.001;
      if (g_jogging_velocity > 0.01) {
        g_jogging_velocity = 0.01;
      }
      ROS_INFO("Jogging velocity factor: %1.1f\r", g_jogging_velocity);
      break;
    case 'p':
      g_jogging_velocity -= 0.001;
      if (g_jogging_velocity < 0) {
        g_jogging_velocity = 0;
      }
      ROS_INFO("Jogging velocity factor: %1.1f\r", g_jogging_velocity);
      break;
    case 'x':
      stop = true;
      break;
    default:
      break;
    }
    pose_ref.header.stamp = ros::Time::now();
    pose_ref.header.frame_id = "/imu_angle";
    pose_ref.pose.orientation = sending_pose.orientation;
    pose_ref.pose.position.x = sending_pose.position.x;
    pose_ref.pose.position.y = sending_pose.position.y;
    pose_ref.pose.position.z = sending_pose.position.z;
    cmd_vel_fct_pub.publish(pose_ref);
    loop_rate_hz.sleep();
  }
  // use system call to set terminal behaviour to more normal behaviour
  system("/bin/stty cooked");
  system("/bin/stty echo");
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
  timeout.tv_usec = g_rate_hz*1000;

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
