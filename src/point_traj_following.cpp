/*
* NODE's PACKAGE: point_traj_following
* NODE EXE NAME:  point_traj_following
* NODE INIT NAME: point_traj_following
* Subcribe topic: 
* NODE function: 
*/
#define NODE_RATE 50

/* --- INCLUDE BEGIN --- */ 
//Bare Cpp and OS include
#include <math.h>
#include <iostream>
#include <string>
//ROS include file
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

//MoveIt include file
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

//ROS message for topic (or service)
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

//User-define message for service
#include <manipulator_pose_following/DeltaPoseRPY.h>
#include <manipulator_pose_following/ReplyInt.h>
#include <manipulator_pose_following/InitPoint.h>
#include "manipulator_pose_following/PlannedPath.h"
/* --- INCLUDE END --- */ 

/* --- BEGIN ROBOT HARDWARE LIMITATION DEFINE --- */
// JOINTS LIMIT (6 REVOLUTE)
// _DEG: in degree // do i use these values ?
#define JOINT_1_S_UPPER_LIMIT_DEG     170
#define JOINT_1_S_LOWER_LIMIT_DEG     -170
#define JOINT_2_L_UPPER_LIMIT_DEG     90
#define JOINT_2_L_LOWER_LIMIT_DEG     -85
#define JOINT_3_U_UPPER_LIMIT_DEG     120
#define JOINT_3_U_LOWER_LIMIT_DEG     -175
#define JOINT_4_R_UPPER_LIMIT_DEG     140
#define JOINT_4_R_LOWER_LIMIT_DEG     -140
#define JOINT_5_B_UPPER_LIMIT_DEG     210
#define JOINT_5_B_LOWER_LIMIT_DEG     -30
#define JOINT_6_T_UPPER_LIMIT_DEG     360
#define JOINT_6_T_LOWER_LIMIT_DEG     -360
// _RAD: in radian
#define JOINT_1_S_UPPER_LIMIT_RAD     170.0*M_PI/180.0
#define JOINT_1_S_LOWER_LIMIT_RAD     -170.0*M_PI/180.0
#define JOINT_2_L_UPPER_LIMIT_RAD     90.0*M_PI/180.0
#define JOINT_2_L_LOWER_LIMIT_RAD     -85.0*M_PI/180.0
#define JOINT_3_U_UPPER_LIMIT_RAD     120.0*M_PI/180.0
#define JOINT_3_U_LOWER_LIMIT_RAD     -175.0*M_PI/180.0
#define JOINT_4_R_UPPER_LIMIT_RAD     140.0*M_PI/180.0
#define JOINT_4_R_LOWER_LIMIT_RAD     -140.0*M_PI/180.0
#define JOINT_5_B_UPPER_LIMIT_RAD     210.0*M_PI/180.00
#define JOINT_5_B_LOWER_LIMIT_RAD     -30.0*M_PI/180.0
#define JOINT_6_T_UPPER_LIMIT_RAD     360.0*M_PI/180.00
#define JOINT_6_T_LOWER_LIMIT_RAD     -360.0*M_PI/180.0

// JOINTS VELOCITY LIMIT (6 REVOLUTE) (315 - 315 - 420 - 600 - 600 - 600)
#define JOINT_1_S_VEL_LIMIT_RADSEC    M_PI*7.0/4.0
#define JOINT_2_L_VEL_LIMIT_RADSEC    M_PI*7.0/4.0
#define JOINT_3_U_VEL_LIMIT_RADSEC    M_PI*7.0/3.0
#define JOINT_4_R_VEL_LIMIT_RADSEC    M_PI*10.0/3.0
#define JOINT_5_B_VEL_LIMIT_RADSEC    M_PI*10.0/3.0
#define JOINT_6_T_VEL_LIMIT_RADSEC    M_PI*10.0/3.0
#define SAFETY_VELOCITY_ALPHA         0.65

/* --- END ROBOT HARDWARE LIMITATION DEFINE --- */

/* --- BEGIN DEFINE SPECIFICATIONS FOR ROBOT AND CONTROL RULE --- */
#define NUMBER_OF_JOINT     6

// --- Globals

const int STATE_IDLE = 0;
const int STATE_POSE_FOLLOW = 1;
const int STATE_STOP = 2;
const int STATE_INIT = 3;

bool g_is_init_done = false;

int g_state = STATE_IDLE;
int g_last_state = g_state;
sensor_msgs::JointState g_current_joints;
ros::Time g_t_start, g_t_last, g_t_last_cb;
geometry_msgs::PoseStamped g_pose, g_pose_last;
geometry_msgs::PoseStamped g_pose_init;

Eigen::Matrix3d K_p = Eigen::Matrix3d::Identity();
Eigen::Matrix3d K_o = Eigen::Matrix3d::Identity();
float Kp_elastic = 0.0, Ko_elastic = 0.0;
float Kp_max = 3.5, Ko_max = 2.5;
float Kdp = 0.25;
float Kdo = 0.25;

Eigen::MatrixXd e_p(3,1); // linear error
Eigen::MatrixXd e_p_last(3,1);
Eigen::MatrixXd e_o(3,1); //angular error
Eigen::MatrixXd e_o_last(3,1);
Eigen::MatrixXd p_and_o_quat_vel(6,1);

/* --- BEGIN FUNCTION PROTOTYPE --- */
Eigen::MatrixXd calc_p_inverse(Eigen::MatrixXd J);
Eigen::MatrixXd calc_sr_inverse(Eigen::MatrixXd J, double w, double w0, double k0);

void cb_joint_state(sensor_msgs::JointState msg);
bool cb_start_pose_following(manipulator_pose_following::ReplyInt::Request &req, manipulator_pose_following::ReplyInt::Response &res);
bool cb_stop_pose_following(manipulator_pose_following::ReplyInt::Request &req, manipulator_pose_following::ReplyInt::Response &res);
bool cb_init_point_accept(manipulator_pose_following::InitPoint::Request &req, manipulator_pose_following::InitPoint::Response &res);

manipulator_pose_following::DeltaPoseRPY calc_dpose(geometry_msgs::PoseStamped pose);
void cb_pose_quat(const manipulator_pose_following::PlannedPath::ConstPtr &msg);

bool uCheck_JointsLimit(trajectory_msgs::JointTrajectoryPoint &point);
bool uCheck_JointsVelocityLimit(Eigen::VectorXd &delta_theta);
void uModify_JacobianParameter(float &Kp, float &Ko, double &w0, double &k0, float &Kdp, float &Kdo);
void handleIdleState(trajectory_msgs::JointTrajectoryPoint &point);
void doInitSteps();
/* --- END FUNCTION PROTOTYPE --- */

int main(int argc, char **argv) {

  // --- Initializations
  ros::init(argc, argv, "point_trajectory_following");

  // --- Setup node handles for
  //     - pose_following callbacks
  //     - start and stop service callbacks
  ros::NodeHandle nh_pose_following;
  ros::NodeHandle nh_startstop;
  ros::NodeHandle nh_pose_quat;

  // --- Setup custom callback queues for
  //     - start_pose_following and stop_pose_following callbacks
  //     - pose callback
  ros::CallbackQueue queue_startstop;
  nh_startstop.setCallbackQueue(&queue_startstop);
  ros::CallbackQueue queue_pose_quat;
  nh_pose_quat.setCallbackQueue(&queue_pose_quat);

  // --- Start an AsyncSpinner with ONE thread for calls to services
  //     - 'start_pose_following'
  //     - 'stop_pose_following'.
  // TODO does this thread need to be stopped when main() is left?
  ros::AsyncSpinner spin_startstop(1, &queue_startstop);
  spin_startstop.start();
  ros::AsyncSpinner spin_pose_quat(1, &queue_pose_quat);
  spin_pose_quat.start();

  // --- Setup publishers
  ros::Publisher streaming_pub =
      nh_pose_following.advertise<trajectory_msgs::JointTrajectory>("joint_command", 10);

  // --- Setup topic subscriptions
  ros::Subscriber sub_pose = nh_pose_quat.subscribe(
      "/user_defined/desired_path_point", 1, cb_pose_quat);
  ros::Subscriber sub_joint = nh_pose_following.subscribe(
      "joint_states", 1, cb_joint_state);

  // --- Advertise services
  //     - start: Changes state to STATE_POSE_FOLLOW
  //     - stop: Changes state to STATE_IDLE
  ros::ServiceServer srv_start = nh_startstop.advertiseService(
      "/pose_following/start", cb_start_pose_following);
  ros::ServiceServer srv_init = nh_startstop.advertiseService(
      "/pose_following/init_point", cb_init_point_accept);
  ros::ServiceServer srv_stop = nh_pose_following.advertiseService(
      "/pose_following/stop", cb_stop_pose_following);

  // --- Obtain parameters
  int rate_hz = NODE_RATE;
  nh_pose_following.getParam("pose_following/rate", rate_hz);
  ros::Rate loop_rate(rate_hz);

  std::string group_st = "arm";
  nh_pose_following.getParam("pose_following/group", group_st);

  double w0 = 0.01;
  nh_pose_following.getParam("pose_following/w0", w0);
  double k0 = 0.01; //modifiable 0.02
  nh_pose_following.getParam("pose_following/k0", k0);

  double theta_d_limit = 3.14;
  nh_pose_following.getParam("pose_following/theta_d_lim", theta_d_limit);

  double dt_pose_lim = 3.0;
  nh_pose_following.getParam("pose_following/dt_pose_lim", dt_pose_lim);

  uModify_JacobianParameter(Kp_max, Ko_max, w0, k0, Kdp, Kdo);
  std::cout << "---------\n" 
            << "Kp_max = " << Kp_max 
            << "\nKo_max = " << Ko_max 
            << "\nw0 = " << w0 
            << "\nk0 = " << k0 
            << "\nKdp = " << Kdp 
            <<  "\nKdo = " << Kdo 
            << "\n---------" << std::endl; 

  // --- Setup MoveIt interface
  moveit::planning_interface::MoveGroupInterface arm(group_st);

  Eigen::MatrixXd J(6, 6);
  Eigen::VectorXd theta_d(NUMBER_OF_JOINT);
  Eigen::Vector3d ref_point(0.0, 0.0, 0.0);

  theta_d.setZero();
  double dt = 0.1;
  double J_cond = 1;
  double J_cond_nakamura = 1;
  double w = 1; // Determinat-based jacobian condition

  // --- Get set up for kinematics:
  robot_model_loader::RobotModelLoader model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = model_loader.getModel();
  printf("Model Frame: %s", kinematic_model->getModelFrame().c_str()); // printf instead of ROS_INFO

  moveit::core::RobotState kinematic_state(kinematic_model);
  kinematic_state.setToDefaultValues();
  const moveit::core::JointModelGroup *joint_model_group_p;
  joint_model_group_p = kinematic_model->getJointModelGroup(group_st);

  const std::vector<std::string> &joint_names =
      joint_model_group_p->getJointModelNames();
  std::vector<double> joint_values;
  kinematic_state.copyJointGroupPositions(joint_model_group_p, joint_values);

  //init first point to be send
  ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");
  trajectory_msgs::JointTrajectory dummy_traj;
  dummy_traj.joint_names = g_current_joints.name;
  trajectory_msgs::JointTrajectoryPoint point, last_point;
  g_t_last = ros::Time::now();
  printf("Waiting for the callback from subscriber init 2.0s ...\n");
  ros::Duration(2.0).sleep();
  ros::spinOnce();
  //the callback will modify the g_current_joints
  if (g_current_joints.name.size() == NUMBER_OF_JOINT)
  {
    std::cout << "Satisfy the init condition, now going to init path ---" << std::endl;
    dummy_traj.joint_names = g_current_joints.name;
    //init the starting position for the first time for 6 Joints in motomini robot
    // make the velocity is 0 rad/s by default
    for (size_t joint = 0; joint < NUMBER_OF_JOINT; joint++)
    {
      point.positions.push_back(g_current_joints.position.at(joint));
      point.velocities.push_back(0);
    }
    // init state so it will have t = 0
    point.time_from_start = ros::Duration(0.0);
    dummy_traj.points.push_back(point);
    // streaming_pub.publish(dummy_traj);
    // after already config, set the last_time to thís state of time
    g_t_last = ros::Time::now();
    g_t_start = ros::Time::now();
  }
  else
  {
    ROS_ERROR("Cannot receive msg from \"joint_state\" topic- Trying again !\n");
  }

  while (ros::ok()) {
    switch (g_state) {

    case STATE_IDLE: // IDLE
      handleIdleState(point);
      break;

    case STATE_INIT:
    {
    if (g_last_state != g_state)
      {
        Kp_elastic = 0.0;
        Ko_elastic = 0.0;
        std::cout << "STATE change to INIT, start to send first zero velocity" << std::endl;
        for (unsigned int joint = 0; joint < NUMBER_OF_JOINT; joint++) {
        point.positions.at(joint) = g_current_joints.position.at(joint);
        point.velocities.at(joint) = 0;
        } 
        point.time_from_start = ros::Duration(0.0);
        dummy_traj.points.at(0) = point;
        dummy_traj.header.stamp = ros::Time::now();
        streaming_pub.publish(dummy_traj);

        g_t_last = ros::Time::now();
        g_last_state = g_state;
        continue;
      }
      // Time since last point:
      // ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");
      dt = (ros::Time::now() - g_t_last).toSec();
      g_t_last = ros::Time::now();
      kinematic_state.setVariableValues(g_current_joints);
      std::cout << "[DEBUG][INIT] joint state update:" << g_current_joints.position[0] << ";"
                                                       << g_current_joints.position[1] << ";"
                                                       << g_current_joints.position[2] << ";"
                                                       << g_current_joints.position[3] << ";"
                                                       << g_current_joints.position[4] << ";"
                                                       << g_current_joints.position[5] << ";"
                                                       << std::endl;
      
      Eigen::Affine3d end_effector_state = kinematic_state.getGlobalLinkTransform("tool0");
      Eigen::Matrix4d homo_mat = end_effector_state.matrix();
      
      // Get the Jacobian
      kinematic_state.getJacobian(
          joint_model_group_p,
          kinematic_state.getLinkModel(
              joint_model_group_p->getLinkModelNames().back()),
          ref_point, J);
      
      //e_p = p_destination - p_end_effector
      e_p(0,0) = g_pose_init.pose.position.x - homo_mat(0,3);
      e_p(1,0) = g_pose_init.pose.position.y - homo_mat(1,3);
      e_p(2,0) = g_pose_init.pose.position.z - homo_mat(2,3);
      std::cout << "[DEBUG][INIT] error in position: e_x=" <<  e_p(0,0) << " e_y=" << e_p(1,0) << " e_z=" << e_p(2,0) << std::endl;
      Eigen::Matrix3d endEffect_rotateMat = homo_mat.block<3,3>(0,0);

      //Check error < 0.5mm = 0.0005m
      const double position_error_up_limit = 0.0005;
      if(e_p(0,0) < position_error_up_limit &&
         e_p(1,0) < position_error_up_limit &&
         e_p(2,0) < position_error_up_limit)
      {
        std::cout << "Done go to init point" << std::endl;
        g_is_init_done = true;
        g_state = STATE_POSE_FOLLOW;
        continue;
      }

      auto roll_d = 179.0;
      auto pitch_d = 0.1;
      auto yaw_d = 0.1;
      //qxd qyd qzd giờ đây đã không còn là quaternion mà đang dùng dưới danh nghĩa roll-pitch-yaw
      Eigen::Matrix3d des_rotateMat;
      des_rotateMat = Eigen::AngleAxisd((roll_d) * M_PI / 180, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd((pitch_d) * M_PI / 180,  Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(yaw_d * M_PI / 180, Eigen::Vector3d::UnitZ());
      
      Eigen::Matrix3d error_rotateMat = des_rotateMat * endEffect_rotateMat.transpose();
      //tính θ xoay quanh r
      double angle_rotate_error = acos(0.5*(error_rotateMat(0,0) + error_rotateMat(1,1) + error_rotateMat(2,2) -1));
      //tinh 3 vector don vi rx ry rz
      double r_unitx = (error_rotateMat(2,1) - error_rotateMat(1,2))/(2*sin(angle_rotate_error));
      double r_unity = (error_rotateMat(0,2) - error_rotateMat(2,0))/(2*sin(angle_rotate_error));
      double r_unitz = (error_rotateMat(1,0) - error_rotateMat(0,1))/(2*sin(angle_rotate_error));
      
      //Tính error
      e_o(0,0) = angle_rotate_error*r_unitx;
      e_o(1,0) = angle_rotate_error*r_unity;
      e_o(2,0) = angle_rotate_error*r_unitz;

      if(Kp_elastic < Kp_max)
      {
        // MAX / (second*NodeRate)
        Kp_elastic += Kp_max / (10 * NODE_RATE);
      }
      if(Ko_elastic < Ko_max)
      {
        // MAX / (second*NodeRate)
        Ko_elastic += Ko_max / (10 * NODE_RATE);
      }
      //p_and_o_quat_vel << Kp_elastic * K_p * e_p ,  Ko_elastic * K_o * e_o;
      p_and_o_quat_vel << Kp_elastic * K_p * e_p + Kdp * (e_p - e_p_last) ,  Ko_elastic * K_o * e_o + Kdo * (e_o - e_o_last);
      e_p_last = e_p;
      e_o_last = e_o;
      //theta_d = J.inverse() * (p_and_o_quat_vel);

      w = pow((J * J.transpose()).determinant(), 0.5);
      Eigen::MatrixXd sr_inv_J = calc_sr_inverse(J, w, w0, k0);
      theta_d = sr_inv_J  * (p_and_o_quat_vel);
      //theta_d = calc_p_inverse(J)  * (p_and_o_quat_vel);

      // check condition number:
      J_cond = J.norm() * calc_p_inverse(J).norm();
      J_cond_nakamura = J.norm() * sr_inv_J.norm();
      if (w <= w0) {
        //ROS_WARN_NAMED("stream_J_cnd", "Close to singularity (w = %1.6f).", w);
      }

      // Check if joints velocities exceed velocity limit for each joint
      if (uCheck_JointsVelocityLimit(theta_d) != true)
      {
          ROS_WARN("Angular velocity exceed vel limit");
          ROS_WARN("Changing state to STATE_STOP.");
          g_state = STATE_STOP;
          continue; // not calculate and block sending new joint command when fall to this condition
      }
      
      for (unsigned int j = 0; j < NUMBER_OF_JOINT; j++) {
        point.positions.at(j) = point.positions.at(j) + theta_d[j] * dt;
        point.velocities.at(j) = theta_d[j];
      }
      //Check if joint variables (with safe padding ) exceed Joint Limit
      if (uCheck_JointsLimit(point) != true)
      {
        point = last_point;
        for (unsigned int j = 0; j < NUMBER_OF_JOINT; j++)
        {
          point.velocities.at(j) = 0;
        }
      }
      point.time_from_start = ros::Time::now() - g_t_start;
      std::cout << "[INIT_POINT] time_from_start: " << point.time_from_start << std::endl;
      dummy_traj.points.at(0) = point;
      dummy_traj.header.stamp = ros::Time::now();
      last_point = point;
      streaming_pub.publish(dummy_traj);
      break;
    } //END STATE_INIT

    case STATE_STOP: // STOP
      if (g_last_state != g_state)
      {
        std::cout << "Falling into stop state" << std::endl;
        g_last_state = g_state; // update last state = this state if spot the diff
      } 
      // --- Keep track of the joint state (positions)
      for (unsigned int joint = 0; joint < NUMBER_OF_JOINT; joint++) {
        point.positions.at(joint) = g_current_joints.position.at(joint);
        point.velocities.at(joint) = 0;
      }
      point.time_from_start = ros::Duration(0.0);
      break;

    case STATE_POSE_FOLLOW: // POSE_FOLLOW
      /* --- BEGIN STATE POSE FOLLOWING --- */
      //first time to move, send zero velocity command
      if (g_last_state != g_state)
      {
        std::cout << "STATE change to pose following, start to send first zero velocity" << std::endl;
        std::cout << "Accept point and state from publisher" << std::endl;

        // for (unsigned int joint = 0; joint < NUMBER_OF_JOINT; joint++) {
        // point.positions.at(joint) = g_current_joints.position.at(joint);
        // point.velocities.at(joint) = 0;
        // } 
        // point.time_from_start = ros::Time::now() - g_t_start;
        // std::cout << "[FOLLOW] time_from_start: " << point.time_from_start << std::endl;
        // dummy_traj.points.at(0) = point;
        // dummy_traj.header.stamp = ros::Time::now();
        // streaming_pub.publish(dummy_traj);
        // std::cout << "Send init zero-vel point at time: " << ros::Time::now();
        // g_t_last = ros::Time::now();
        g_last_state = g_state;
        g_pose = g_pose_init;
        continue;
      }
      // double dt_cb = (ros::Time::now() - g_t_last_cb).toSec();
      // if (dt_cb > dt_pose_lim) {
      //   ROS_DEBUG_NAMED("stream_dbg", "dt_cb: %1.3f", dt_cb);
      //   g_state = STATE_IDLE;
      //   ROS_DEBUG_NAMED("state", "STATE_POSE_FOLLOW --> STATE_IDLE");
      //   ROS_DEBUG_NAMED("event", "Event: dt_cb > dt_pose_lim");\
      //   continue; // pass all computing and send command below
      // } 
      // Time since last point:
      dt = (ros::Time::now() - g_t_last).toSec();
      g_t_last = ros::Time::now();
      kinematic_state.setVariableValues(g_current_joints);
      std::cout << "[DEBUG][FOLLOW] joint state update:" << g_current_joints.position[0] << ";"
                                                       << g_current_joints.position[1] << ";"
                                                       << g_current_joints.position[2] << ";"
                                                       << g_current_joints.position[3] << ";"
                                                       << g_current_joints.position[4] << ";"
                                                       << g_current_joints.position[5] << ";"
                                                       << std::endl;
      
      Eigen::Affine3d end_effector_state = kinematic_state.getGlobalLinkTransform("tool0");
      Eigen::Matrix4d homo_mat = end_effector_state.matrix();
      
      // Get the Jacobian
      kinematic_state.getJacobian(joint_model_group_p,kinematic_state.getLinkModel(joint_model_group_p->getLinkModelNames().back()), ref_point, J);
      
      //e_p = p_destination - p_end_effector
      e_p(0,0) = g_pose.pose.position.x - homo_mat(0,3);
      e_p(1,0) = g_pose.pose.position.y - homo_mat(1,3);
      e_p(2,0) = g_pose.pose.position.z - homo_mat(2,3);
      std::cout << "Debug error in position: e_x=" <<  e_p(0,0) << " e_y=" << e_p(1,0) << " e_z=" << e_p(2,0) << std::endl;
      Eigen::Matrix3d endEffect_rotateMat = homo_mat.block<3,3>(0,0);

      auto roll_d = 179.0;
      auto pitch_d = 0.1;
      auto yaw_d = 0.1;
      //qxd qyd qzd giờ đây đã không còn là quaternion mà đang dùng dưới danh nghĩa roll-pitch-yaw
      Eigen::Matrix3d des_rotateMat;
      des_rotateMat = Eigen::AngleAxisd((roll_d) * M_PI / 180, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd((pitch_d) * M_PI / 180,  Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(yaw_d * M_PI / 180, Eigen::Vector3d::UnitZ());
      
      Eigen::Matrix3d error_rotateMat = des_rotateMat * endEffect_rotateMat.transpose();
      //tính θ xoay quanh r
      double angle_rotate_error = acos(0.5*(error_rotateMat(0,0) + error_rotateMat(1,1) + error_rotateMat(2,2) -1));
      //tinh 3 vector don vi rx ry rz
      double r_unitx = (error_rotateMat(2,1) - error_rotateMat(1,2))/(2*sin(angle_rotate_error));
      double r_unity = (error_rotateMat(0,2) - error_rotateMat(2,0))/(2*sin(angle_rotate_error));
      double r_unitz = (error_rotateMat(1,0) - error_rotateMat(0,1))/(2*sin(angle_rotate_error));
      
      //Tính error
      e_o(0,0) = angle_rotate_error*r_unitx;
      e_o(1,0) = angle_rotate_error*r_unity;
      e_o(2,0) = angle_rotate_error*r_unitz;

      if(Kp_elastic < Kp_max)
      {
        // MAX / (second*NodeRate)
        Kp_elastic += Kp_max / (10 * NODE_RATE);
      }
      if(Ko_elastic < Ko_max)
      {
        // MAX / (second*NodeRate)
        Ko_elastic += Ko_max / (10 * NODE_RATE);
      }
      //p_and_o_quat_vel << Kp_elastic * K_p * e_p ,  Ko_elastic * K_o * e_o;
      p_and_o_quat_vel << Kp_elastic * K_p * e_p + Kdp * (e_p - e_p_last) ,  Ko_elastic * K_o * e_o + Kdo * (e_o - e_o_last);
      e_p_last = e_p;
      e_o_last = e_o;
      //theta_d = J.inverse() * (p_and_o_quat_vel);

      w = pow((J * J.transpose()).determinant(), 0.5);
      Eigen::MatrixXd sr_inv_J = calc_sr_inverse(J, w, w0, k0);
      theta_d = sr_inv_J  * (p_and_o_quat_vel);
      //theta_d = calc_p_inverse(J)  * (p_and_o_quat_vel);

      // check condition number:
      J_cond = J.norm() * calc_p_inverse(J).norm();
      J_cond_nakamura = J.norm() * sr_inv_J.norm();
      if (w <= w0) {
        //ROS_WARN_NAMED("stream_J_cnd", "Close to singularity (w = %1.6f).", w);
      }

      // Check if joints velocities exceed velocity limit for each joint
      if (uCheck_JointsVelocityLimit(theta_d) != true)
      {
          ROS_WARN("Angular velocity exceed vel limit");
          ROS_WARN("Changing state to STATE_STOP.");
          g_state = STATE_STOP;
          continue; // not calculate and block sending new joint command when fall to this condition
      }
      
      for (unsigned int j = 0; j < NUMBER_OF_JOINT; j++) {
        point.positions.at(j) = point.positions.at(j) + theta_d[j] * dt;
        point.velocities.at(j) = theta_d[j];
      }
      //Check if joint variables (with safe padding ) exceed Joint Limit
      if (uCheck_JointsLimit(point) != true)
      {
        point = last_point;
        for (unsigned int j = 0; j < NUMBER_OF_JOINT; j++)
        {
          point.velocities.at(j) = 0;
        }
      }
      point.time_from_start = ros::Time::now() - g_t_start;
      dummy_traj.points.at(0) = point;
      dummy_traj.header.stamp = ros::Time::now();
      last_point = point;
      streaming_pub.publish(dummy_traj);
      /* --- END STATE POSE FOLLOWING --- */
    } // end switch(g_state)

    loop_rate.sleep();
    // FIXME This is a blocking call that should be avoided in the future
    ros::spinOnce();

  } // end while(ros::ok())
  return 1;
}

bool uCheck_JointsLimit(trajectory_msgs::JointTrajectoryPoint &point)
{
  float safety_joint_padding = 5 * M_PI / 180.0;
  //Check Joint_1_S
  if (point.positions.at(0) <= JOINT_1_S_LOWER_LIMIT_RAD + safety_joint_padding || 
      point.positions.at(0) >= JOINT_1_S_UPPER_LIMIT_RAD - safety_joint_padding) 
      return false;
  //Check Joint_2_L
  if (point.positions.at(1) <= JOINT_2_L_LOWER_LIMIT_RAD + safety_joint_padding || 
      point.positions.at(1) >= JOINT_2_L_UPPER_LIMIT_RAD - safety_joint_padding) 
      return false;
  //Check Joint_3_U
  if (point.positions.at(2) <= JOINT_3_U_LOWER_LIMIT_RAD + safety_joint_padding || 
      point.positions.at(2) >= JOINT_3_U_UPPER_LIMIT_RAD - safety_joint_padding) 
      return false;
  //Check Joint_4_R
  if (point.positions.at(3) <= JOINT_4_R_LOWER_LIMIT_RAD + safety_joint_padding || 
      point.positions.at(3) >= JOINT_4_R_UPPER_LIMIT_RAD - safety_joint_padding) 
      return false;
  //Check Joint_5_B
  if (point.positions.at(4) <= JOINT_5_B_LOWER_LIMIT_RAD + safety_joint_padding || 
      point.positions.at(4) >= JOINT_5_B_UPPER_LIMIT_RAD - safety_joint_padding) 
      return false;
  //Check Joint_6_T
  if (point.positions.at(5) <= JOINT_6_T_LOWER_LIMIT_RAD + safety_joint_padding || 
      point.positions.at(5) >= JOINT_6_T_UPPER_LIMIT_RAD - safety_joint_padding) 
      return false;

  // if all position checklist is all pass, make it eligible by return true
  return true;
}

bool uCheck_JointsVelocityLimit(Eigen::VectorXd &delta_theta)
{
  if(delta_theta[0] >= JOINT_1_S_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA || 
    delta_theta[0] <= -JOINT_1_S_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA)
    return false;
  if(delta_theta[1] >= JOINT_2_L_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA|| 
    delta_theta[1] <= -JOINT_2_L_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA)
    return false;
  if(delta_theta[2] >= JOINT_3_U_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA|| 
    delta_theta[2] <= -JOINT_3_U_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA)
    return false;
  if(delta_theta[3] >= JOINT_4_R_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA|| 
    delta_theta[3] <= -JOINT_4_R_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA)
    return false;
  if(delta_theta[4] >= JOINT_5_B_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA|| 
    delta_theta[4] <= -JOINT_5_B_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA)
    return false;
  if(delta_theta[5] >= JOINT_6_T_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA|| 
    delta_theta[5] <= -JOINT_6_T_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA)
    return false;
  return true;
}

Eigen::MatrixXd calc_p_inverse(Eigen::MatrixXd J) {
  return (J.transpose() * (J * J.transpose()).inverse());
}

Eigen::MatrixXd calc_sr_inverse(Eigen::MatrixXd J, double w, double w0, double k0) {
  double k = 0;
  if (w < w0) {
    k = k0 * pow(1 - w / w0, 2);
  }
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
  return (J.transpose() * (J * J.transpose() + k * I).inverse());
}

void cb_joint_state(sensor_msgs::JointState msg) {
  g_current_joints = msg;
}

bool cb_start_pose_following(
    manipulator_pose_following::ReplyInt::Request &req,
    manipulator_pose_following::ReplyInt::Response &res) {

  std::cout << "/pose_following/start signal received." << std::endl;
  g_state = STATE_IDLE;
  res.reply = 1;
  return true;
}

bool cb_init_point_accept(
  manipulator_pose_following::InitPoint::Request &req, 
  manipulator_pose_following::InitPoint::Response &res) {
    if (g_state == STATE_IDLE)
    {
      g_state = STATE_INIT;
      g_t_start = ros::Time::now();
    }
    g_pose_init.pose.position.x = req.x;
    g_pose_init.pose.position.y = req.y;
    g_pose_init.pose.position.z = req.z;
    
    if (g_is_init_done == true)
    {
      res.transfer_state = 10;
    }
    return true;
}

bool cb_stop_pose_following(
    manipulator_pose_following::ReplyInt::Request &req,
    manipulator_pose_following::ReplyInt::Response &res) {

  std::cout << "/pose_following/stop signal received." << std::endl;
  g_state = STATE_STOP;
  Kp_elastic = 0.0;
  Ko_elastic = 0.0;
  res.reply = 1;
  return true;
}

void cb_pose_quat(const manipulator_pose_following::PlannedPath::ConstPtr &msg) {

  // if (g_state == STATE_IDLE) {
  //   g_state = STATE_POSE_FOLLOW;
  //   ROS_DEBUG_NAMED("state", "STATE_IDLE --> STATE_POSE_FOLLOW");
  //   ROS_DEBUG_NAMED("event", "Event: received msg on topic /pose");
  //   g_t_start = ros::Time::now();
  // }

  g_t_last_cb = ros::Time::now();
  g_pose.pose.position.x = msg->x;
  g_pose.pose.position.y = msg->y;
  g_pose.pose.position.z = msg->z;
  g_pose.pose.orientation.x = 180.0;
  g_pose.pose.orientation.y = 0.1;
  g_pose.pose.orientation.z = 0.1;
}

void uModify_JacobianParameter(float &Kp, float &Ko, double &w0, double &k0, float &Kdp, float &Kdo)
{
  std::string f_user_input;
  std::cout << "Do you want to modify Kp, Ko, w0, k0, Kdp, Kdo parameter (y/n) ?:  ";
  std::getline(std::cin, f_user_input);
  bool isModify = false;
  char user_modify = 'n';
  std::stringstream(f_user_input) >> user_modify;
  if (user_modify == 'Y' || user_modify == 'y')
  {
    bool continue_modify = true;
    do
    {
      std::cout << "Choose a parameter to be modify:\n1.Kp\n2.Ko\n3.w0\n4.k0\n5.Kdo\n6.Kdp ....Enter you choice: ";
      int user_choose_option = 0;
      while(user_choose_option < 1 || user_choose_option > 6)
      {
        std::getline(std::cin, f_user_input);
        std::stringstream(f_user_input) >> user_choose_option;
        if (user_choose_option < 1 || user_choose_option > 4)
          std::cout << "That's not the right number in this scenario, re-enter again (1-4): !";
      }
      switch (user_choose_option)
      {
      case 1:
        std::cout << "You choose to modify Kp, current Kp is " << Kp << "\n";
        std::cout << "Enter new value Kp: ";
        std::getline(std::cin, f_user_input);
        std::stringstream(f_user_input) >> Kp;
        break;
      case 2:
        std::cout << "You choose to modify Ko, current Ko is " << Ko << "\n";
        std::cout << "Enter new value Ko: ";
        std::getline(std::cin, f_user_input);
        std::stringstream(f_user_input) >> Ko;
        break;
      case 3:
        std::cout << "You choose to modify w0, current w0 is " << w0 << "\n";
        std::cout << "Enter new value w0: ";
        std::getline(std::cin, f_user_input);
        std::stringstream(f_user_input) >> w0;
        break;
      case 4:
        std::cout << "You choose to modify k0, current k0 is " << k0 << "\n";
        std::cout << "Enter new value k0: ";
        std::getline(std::cin, f_user_input);
        std::stringstream(f_user_input) >> k0;
        break;
      case 5:
        std::cout << "You choose to modify Kdp, current Kdp is " << Kdp << "\n";
        std::cout << "Enter new value Kdp: ";
        std::getline(std::cin, f_user_input);
        std::stringstream(f_user_input) >> Kdp;
        break;
      case 6:
        std::cout << "You choose to modify Kdo, current Kdo is " << Kdo << "\n";
        std::cout << "Enter new value Kdo: ";
        std::getline(std::cin, f_user_input);
        std::stringstream(f_user_input) >> Kdo;
        break;
      default:
        break;
      }
      std::cout << "Do you want to continue modify ? (y/n): ";
      continue_modify = false;
      std::getline(std::cin, f_user_input);
      char cont_mod;
      std::stringstream(f_user_input) >> cont_mod;
      if (cont_mod == 'Y' || cont_mod == 'y')
      {
        continue_modify = true;
      }
      
    }
    while (continue_modify);
  }
  else
  {
    std::cout << "\nNo parameter to be modified ! The parameter will be set by default.\n";
    std::cout << "Kp will be set at: " << Kp << "\n";
    std::cout << "Ko will be set at: " << Ko << "\n";
    std::cout << "w0 will be set at: " << w0 << "\n";
    std::cout << "k0 will be set at: " << k0 << "\n";
    std::cout << "Kdp will be set at: " << Kdp << "\n";
    std::cout << "Kdo will be set at: " << Kdo << std::endl;
  }
  
  return;
}

void handleIdleState(trajectory_msgs::JointTrajectoryPoint &point)
{
  if (g_last_state != g_state)
  {
    g_last_state = g_state; // update last state = this state if spot the diff
    std::cout << "Falling into idle state" << std::endl;
  } 
  // --- Keep track of the joint state (positions)
  for (unsigned int joint = 0; joint < NUMBER_OF_JOINT; joint++) {
    point.positions.at(joint) = g_current_joints.position.at(joint);
    point.velocities.at(joint) = 0;
  }
  point.time_from_start = ros::Duration(0.0);
}