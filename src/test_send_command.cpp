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
//  - to contain callback value when get current joints para from joint_states topic
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

  //Set-up publisher:
  //  - streaming_pub: deliveries trajectory to "joint_command" topic. 
  ros::Publisher streaming_pub = nh_pose_following.advertise<trajectory_msgs::JointTrajectory>("joint_command",10);

  //Set-up subcribers:
  //  - sub_joint: get feeback about current robot joints
  //  - vel_sub: get angle velocity from the imu-read node
  ros::Subscriber sub_joint = nh_pose_following.subscribe("/joint_states", 1, cb_joint_state);
  ros::Subscriber vel_sub = nh_pose_following.subscribe("imu_reader/cmd_vel", 1, cb_angle_vel);

  //Set-up control rate, this will affect on the `dt` and orther time variable in while(ros::ok()) loop
  int rate_hz = 35;
  ros::Rate loop_rate(rate_hz);

  Eigen::VectorXd theta_d(NUMBER_OF_JOINT);
  theta_d.setZero();
  double theta_d_limit = 5.0;

  // --- Initialize jogging start
  // Before start to initialize jogging start, wait 2.0s for the callback func
  //    of "/joint_states" topic from the robot to be called at least one time
  trajectory_msgs::JointTrajectory dummy_traj;
  trajectory_msgs::JointTrajectoryPoint point;
  do {
    ROS_INFO("Waiting for the callback from subcriber init 2.0s ...\n");
    // make the callback at least do one time in 2.0s duration
    ros::Duration(2.0).sleep();
    ros::spinOnce();
    //the callback will modify the g_current_joints
    if (g_current_joints.name.size() == NUMBER_OF_JOINT)
    {
      std::cout << "Satisfy the init condition, now going to init path ---" << std::endl;
      dummy_traj.joint_names = g_current_joints.name;
      //init the starting position for the first time for 6 Joints in motomini robot
      for (size_t joint = 0; joint < NUMBER_OF_JOINT; joint++)
      {
        point.positions.push_back(g_current_joints.position.at(joint));
        point.velocities.push_back(0);
      }
      // init state so it will have t = 0
      point.time_from_start = ros::Duration(0.0);
      dummy_traj.points.push_back(point);
      streaming_pub.publish(dummy_traj);
      // after already config, set the last_time to thís state of time
      g_t_last = ros::Time::now();
      g_t_start = ros::Time::now();
    }
    else
    {
      ROS_ERROR("Cannot receive msg from \"joint_state\" topic- Trying again !\n");
    }

  } while (g_current_joints.name.size() != NUMBER_OF_JOINT);

  double dt = 0.1;

  theta_d[0] = 2.0;
  g_vel_from_imu.angular.z = 0.0;

  long int counter = 0;

  while (ros::ok()){
    //Time since last point
    dt = (ros::Time::now() - g_t_last).toSec();
    g_t_last = ros::Time::now(); // calculated dt, now g_t_last will be updated
    std::cout << "Step into path planning, #point = " << counter++ << std::endl;
    
    theta_d[0] = g_vel_from_imu.angular.z;
    theta_d[1] = g_vel_from_imu.angular.y;
    theta_d[3] = g_vel_from_imu.angular.x;
    
    for (size_t joint = 0; joint < NUMBER_OF_JOINT; joint++)
    {
      if (fabs(theta_d[joint]) > theta_d_limit)
      {
        ROS_WARN("Angular velocity of joint %d exceeding %2.2f rad/s.",(int)joint,theta_d_limit);
        theta_d[joint] = 0;
      }
      point.positions.at(joint) = point.positions.at(joint) + theta_d[joint] * dt;
      point.velocities.at(joint) = theta_d[joint];
    }
    point.time_from_start = ros::Time::now() - g_t_start;
    dummy_traj.points.at(0) = point;
    dummy_traj.header.stamp = ros::Time::now();
    streaming_pub.publish(dummy_traj);

    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}