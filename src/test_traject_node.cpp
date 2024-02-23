#define NODE_LOOP_RATE 100

#include <cmath>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#define _USE_MATH_DEFINES

const int TRAJ_1 = 0;
const int TRAJ_2 = 1;
const int TRAJ_STABLE = 2;

int main(int argc, char **argv) {

  ros::init(argc, argv, "test_traject");
  ros::NodeHandle node_handle;

  ros::Publisher pub_pose = node_handle.advertise<geometry_msgs::PoseStamped>(
      "pose_following/pose", 1);

  // --- Obtain parameters ---
  int rate_hz = NODE_LOOP_RATE;
  node_handle.getParam("test_traject/rate", rate_hz);
  ros::Rate loop_rate(rate_hz);
  
  geometry_msgs::Pose pose_init;
  pose_init.position.x = 0.185;
  pose_init.position.y = 0.0;
  pose_init.position.z = 0.125;
  pose_init.orientation.x = 180;
  pose_init.orientation.y = 0.5;
  pose_init.orientation.z = 0.5;
  pose_init.orientation.w = 0.0;
  ros::Time t_init = ros::Time::now();
  double a = 0.25;
  double a_ref[2] = {0, 0};
  double v_ref[2] = {0, 0};
  double pos_ref[2] = {0, 0};
  double t_period = 30;
  bool first_time = true;
  double amp = 0.08;
  int traj = TRAJ_1;
  double dt = 0;
  double t_last_d = 0;

  std::cout << "Input velocity (mm/s): ";
  //TODO: make input velocity available to process test procedure
  double vel = 0.0;
  std::string f_user_input;
  std::getline(std::cin, f_user_input);
  std::stringstream(f_user_input) >> vel;


  std::cout << "Press a key to start the planning traject!\n";
  std::cin.get();
  long int counter = 0;
  while (ros::ok()) {
    double t_now_d = (ros::Time::now() - t_init).toSec();
    dt = t_now_d - t_last_d;
    t_last_d = t_now_d;
    ROS_DEBUG_NAMED("stream_dbg", "dt = %2.2f", dt);

    switch (traj) {
    case TRAJ_1:
      
      
      //pos_ref[1] = amp * sin(2 * M_PI * t_now_d / t_period);
      //pos_ref[1] += amp * sin(2 * M_PI * t_now_d / t_period);

      /* counter ++;
      if (counter > NODE_LOOP_RATE * 10)
      {
        if (first_time)
        {
          t_init = ros::Time::now();
          first_time = false;
          continue;
        }
        pos_ref[0] = amp * cos(2 * M_PI * t_now_d / t_period + M_PI/2);
      } */

      counter ++;
      if (pos_ref[0] < 0.081 && counter > NODE_LOOP_RATE * 8)
      {
        pos_ref[0] = pos_ref[0] + (vel/1000) * dt;
      }

      break;
    case TRAJ_2:
      if (t_now_d < 1) {
        a_ref[0] = a;
        ROS_DEBUG("Accelerating: a = %1.0f", a_ref[0]);
      } else if (t_now_d >= 1 && t_now_d < 2) {
        ROS_DEBUG("Constant velocity. a = %1.0f", a_ref[0]);
        a_ref[0] = 0;
      } else if (t_now_d >= 2 && t_now_d < 3) {
        a_ref[0] = -a;
        ROS_DEBUG("Decelerating: a = %1.0f", a_ref[0]);
      } else if (t_now_d >= 3) {
        a_ref[0] = 0;
        ROS_DEBUG("Stopped");
      }
      v_ref[0] = v_ref[0] + a_ref[0] * dt;
      pos_ref[0] = pos_ref[0] + v_ref[0] * dt;
      ROS_DEBUG("a_ref[0] = %2.3f", a_ref[0]);
      ROS_DEBUG("v_ref[0] = %2.3f", v_ref[0]);
      ROS_DEBUG("pos_ref[0] = %2.3f", pos_ref[0]);
      break;
    case TRAJ_STABLE:
      break;
    default:
      break;
    }

    geometry_msgs::PoseStamped pose_ref;
    pose_ref.header.stamp = ros::Time::now();
    pose_ref.header.frame_id = "/base";
    pose_ref.pose.orientation = pose_init.orientation;
    pose_ref.pose.position.x = pose_init.position.x + pos_ref[0];
    pose_ref.pose.position.y = pose_init.position.y;
    pose_ref.pose.position.z = pose_init.position.z;

    pub_pose.publish(pose_ref);

    loop_rate.sleep();
  }
}
