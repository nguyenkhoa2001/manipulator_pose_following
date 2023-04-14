#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <manipulator_pose_following/DeltaPoseRPY.h>
#include <manipulator_pose_following/ReplyInt.h>
#include <sensor_msgs/JointState.h>

#include <math.h>

// --- Globals
bool g_do_pose_following = false;

const int STATE_IDLE = 0;
const int STATE_POSE_FOLLOW = 1;
const int STATE_STOP = 2;

int g_state = STATE_IDLE;
sensor_msgs::JointState g_current_joints;
ros::Time g_t_start, g_t_last, g_t_last_cb;

// TODO Make below message a stamped one
manipulator_pose_following::DeltaPoseRPY g_delta_pose;
geometry_msgs::PoseStamped g_pose, g_pose_last;

void cb_joint_state(sensor_msgs::JointState msg) {
  //ROS_DEBUG_NAMED("stream_dbg", "New joint state...");
  g_current_joints = msg;
}


int main(int argc, char **argv) {

  // --- Initializations
  ros::init(argc, argv, "check_kinematic_node");

  // --- Setup node handles for
  //     - pose_following callbacks
  //     - start and stop service callbacks
  ros::NodeHandle nh_pose_following;

  ros::Subscriber sub_joint =
      nh_pose_following.subscribe("/joint_states", 1, cb_joint_state);

  // --- Obtain parameters
  int rate_hz = 30;
  ros::Rate loop_rate(rate_hz);

  std::string group_st = "arm";

  double w0 = 0.1;
  double k0 = 0.001; //modifiable
  double theta_d_limit = 3.14;
  double dt_pose_lim = 0.5;

  // --- Setup MoveIt interface
  moveit::planning_interface::MoveGroupInterface arm(group_st);

  Eigen::MatrixXd J(6, 6);
  Eigen::VectorXd theta_d(6), cart_vel(6);
  Eigen::Vector3d ref_point(0.0, 0.0, 0.0);

  cart_vel.setZero();
  theta_d.setZero();
  double dt = 0.1;
  double J_cond = 1;
  double J_cond_nakamura = 1;
  double w = 1; // Determinat-based jacobian condition
  g_delta_pose.data.resize(6, 0);

  // --- Get set up for kinematics:
  robot_model_loader::RobotModelLoader model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = model_loader.getModel();
  ROS_INFO("Model Frame: %s", kinematic_model->getModelFrame().c_str());

  moveit::core::RobotState kinematic_state(kinematic_model);
  kinematic_state.setToDefaultValues();
  const moveit::core::JointModelGroup *joint_model_group_p;
  joint_model_group_p = kinematic_model->getJointModelGroup(group_st);

  const std::vector<std::string> &joint_names =
      joint_model_group_p->getJointModelNames();
  std::vector<double> joint_values;
  kinematic_state.copyJointGroupPositions(joint_model_group_p, joint_values);

  ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");
  ROS_DEBUG("joint_names: [%s %s %s %s %s %s]", joint_names[0].c_str(),
            joint_names[1].c_str(), joint_names[2].c_str(),
            joint_names[3].c_str(), joint_names[4].c_str(),
            joint_names[5].c_str());

  ROS_DEBUG("joint_values: [%1.2f %1.2f %1.2f %1.2f %1.2f %1.2f]",
            joint_values[0], joint_values[1], joint_values[2], joint_values[3],
            joint_values[4], joint_values[5]);
  Eigen::Isometry3d frame_tf = kinematic_state.getFrameTransform("tool0");
  std::ostream stream(nullptr);
  std::stringbuf str;
  stream.rdbuf(&str);
  kinematic_state.printTransform(frame_tf, stream);
  ROS_DEBUG_STREAM("transform: " << str.str());
  std::ostream stream2(nullptr);
  std::stringbuf str2;
  stream2.rdbuf(&str2);
  joint_model_group_p->printGroupInfo(stream2);
  ROS_DEBUG_STREAM("joint_model_group_info: " << str2.str());

  ROS_DEBUG_STREAM("g_current_joints: \n" << g_current_joints);
  // --- Initialize jogging start
  trajectory_msgs::JointTrajectory dummy_traj;
  dummy_traj.joint_names = g_current_joints.name;
  trajectory_msgs::JointTrajectoryPoint point;
  for (unsigned int joint = 0; joint < 6; joint++) {
    point.positions.push_back(g_current_joints.position.at(joint));
    point.velocities.push_back(0);
  }
  point.time_from_start = ros::Duration(0.0);
  dummy_traj.points.push_back(point);

  g_t_last = ros::Time::now();



  while (ros::ok()) {
    Eigen::Isometry3d frame_tf = kinematic_state.getFrameTransform("tool0");
    ROS_INFO_STREAM(frame_tf.rotation());
    kinematic_state.setVariableValues(g_current_joints);
    std::ostream stream(nullptr);
    std::stringbuf str;
    stream.rdbuf(&str);
    kinematic_state.printTransform(frame_tf, stream);
    ROS_INFO_STREAM("framtf: " << str.str());
    //ROS_INFO_STREAM(g_current_joints);
    loop_rate.sleep();
    // FIXME This is a blocking call that should be avoided in the future
    ros::spinOnce();

  } // end while(ros::ok())
  return 1;
}
