// File nay cu lam roi
//Chức năng: chuyển giá trị đọc được (vận tốc) của IMU để di chuyển robot thông qua topic joint_command
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/console.h>

#include <tf/transform_datatypes.h>

#include <math.h>
#include <iostream>

#define NUMBER_OF_JOINT 6

//Global variable
sensor_msgs::JointState g_current_joints;
ros::Time g_t_start, g_t_last, g_t_last_cb;

geometry_msgs::Twist g_vel_from_imu;

//Callback function
void cb_joint_state(sensor_msgs::JointState msg) {
  g_current_joints = msg;
}

void cb_angle_vel(geometry_msgs::Twist msg) {
  g_vel_from_imu = msg;
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "test_send_command_node");
  ros::NodeHandle nh_pose_following;

  ros::Publisher streaming_pub = nh_pose_following.advertise<trajectory_msgs::JointTrajectory>("joint_command",10);
  ros::Subscriber sub_joint = nh_pose_following.subscribe("/joint_states", 1, cb_joint_state);

  //doc van toc tu imu
  ros::Subscriber vel_sub = nh_pose_following.subscribe("turtle1/cmd_vel", 1, cb_angle_vel);

  int rate_hz = 60;
  ros::Rate loop_rate(rate_hz);

  g_t_start = ros::Time::now();

  trajectory_msgs::JointTrajectory dummy_traj;
  trajectory_msgs::JointTrajectoryPoint point;

  Eigen::VectorXd theta_d(6);
  theta_d.setZero();
  
  bool first_time = true;
  bool second_time = false;
  while (ros::ok()){

    if (first_time == true) {
      ros::Duration(1.0).sleep();
      ros::spinOnce();

      dummy_traj.joint_names = g_current_joints.name;
      point.time_from_start = ros::Duration(0.0);

      for (unsigned int joint = 0; joint < NUMBER_OF_JOINT; joint++) {
        point.positions.push_back(g_current_joints.position.at(joint));
        point.velocities.push_back(g_current_joints.velocity.at(joint));
      }
      dummy_traj.points.push_back(point);
      dummy_traj.points.at(0) = point;
      dummy_traj.header.stamp = ros::Time::now();
      streaming_pub.publish(dummy_traj);
      g_t_last = ros::Time::now();
      first_time = false;
      second_time = true;
      continue;
    }
    
    if (second_time == true)
    {
      second_time = false;
      ros::spinOnce();
      dummy_traj.joint_names = g_current_joints.name;
      for (unsigned int joint = 0; joint < NUMBER_OF_JOINT; joint++) {
        point.positions[joint] = g_current_joints.position.at(joint);
        point.velocities[joint] =  g_current_joints.velocity.at(joint);
      }
      
      point.time_from_start = ros::Time::now() - g_t_start;
      g_t_last = ros::Time::now();

      dummy_traj.points.at(0) = point;
      dummy_traj.header.stamp = ros::Time::now();
      streaming_pub.publish(dummy_traj);
      continue;
    }

    if (first_time == false && second_time == false)
    {
      theta_d[0] = 3.0;
  
      double dt = (ros::Time::now() - g_t_last).toSec();
      for (unsigned int j = 0; j < NUMBER_OF_JOINT; j++) {
        point.positions.at(j) = g_current_joints.position.at(j) + theta_d[j] * dt;
        point.velocities.at(j) = theta_d[j];
      }
      g_t_last = ros::Time::now();

      dummy_traj.points.at(0) = point;
      dummy_traj.header.stamp = ros::Time::now();
      streaming_pub.publish(dummy_traj);
    }
    //theta_d[0] = g_vel_from_imu.angular.z;
    // theta_d[0] -= 0.001;
    // if (theta_d[0] <= -3.0) theta_d[0] = 0;
    // theta_d[0] = 3.0;
  
    // double dt = (ros::Time::now() - g_t_last).toSec();
    // for (unsigned int j = 0; j < NUMBER_OF_JOINT; j++) {
    //   point.positions.at(j) = g_current_joints.position.at(j) + theta_d[j] * dt;
    //   point.velocities.at(j) = theta_d[j];
    // }
    // g_t_last = ros::Time::now();

    // dummy_traj.points.at(0) = point;
    // dummy_traj.header.stamp = ros::Time::now();
    // streaming_pub.publish(dummy_traj);
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}