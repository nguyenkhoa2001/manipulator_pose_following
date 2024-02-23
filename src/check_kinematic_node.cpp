#define NODE_RATE_HZ 60

#define EULER_ANGLE_X_INDEX 0
#define EULER_ANGLE_Y_INDEX 1
#define EULER_ANGLE_Z_INDEX 2


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

sensor_msgs::JointState g_current_joints;

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

  // Subcribe /joint_states to get joint space value from q1 -> q6
  ros::Subscriber sub_joint =
      nh_pose_following.subscribe("/joint_states", 1, cb_joint_state);
  
  // Publish xyz+rpy to /pose_real/pose topic
  ros::Publisher pub_real_pose = nh_pose_following.advertise<geometry_msgs::PoseStamped>("pose_real/pose", 1);
  geometry_msgs::PoseStamped pose_ref;


  // --- Obtain parameters
  int rate_hz = NODE_RATE_HZ;
  ros::Rate loop_rate(rate_hz);

  // --- Setup MoveIt interface
  std::string group_st = "arm";
  moveit::planning_interface::MoveGroupInterface arm(group_st);

  Eigen::MatrixXd J(6, 6);
 
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



  while (ros::ok()) {
    /* Eigen::Isometry3d frame_tf = kinematic_state.getFrameTransform("tool0");
    kinematic_state.setVariableValues(g_current_joints);
    std::ostream stream(nullptr);
    std::stringbuf str;
    stream.rdbuf(&str);
    kinematic_state.printTransform(frame_tf, stream);
    pose_ref.header.stamp = ros::Time::now();
    pose_ref.header.frame_id = "/base";
    pose_ref.pose.orientation = pose_init.orientation;
    pose_ref.pose.position.x = frame_tf.translation()[0];
    pose_ref.pose.position.y = frame_tf.translation()[1];
    pose_ref.pose.position.z = frame_tf.translation()[2]; */
    
    //Extract homo matrix forward kinematic
    kinematic_state.setVariableValues(g_current_joints);
    Eigen::Affine3d end_effector_state = kinematic_state.getGlobalLinkTransform("tool0");
    Eigen::Matrix4d homo_mat = end_effector_state.matrix();
    
    // Extract position from 4th column of the matrix
    pose_ref.pose.position.x = homo_mat(0,3);
    pose_ref.pose.position.y = homo_mat(1,3);
    pose_ref.pose.position.z = homo_mat(2,3);
    //Extract rotation matrix from homo by getting a [3x3] block, start from position (r=0,c=0)
    Eigen::Matrix3d endEffect_rotateMat = homo_mat.block<3,3>(0,0);
    Eigen::Vector3d rotation_angle = endEffect_rotateMat.eulerAngles(EULER_ANGLE_X_INDEX, EULER_ANGLE_Y_INDEX, EULER_ANGLE_Z_INDEX);
    pose_ref.pose.orientation.x = rotation_angle(0) * 180 / M_PI;
    pose_ref.pose.orientation.y = rotation_angle(1) * 180 / M_PI;
    pose_ref.pose.orientation.z = rotation_angle(2) * 180 / M_PI;
    pose_ref.header.stamp = ros::Time::now();
    pose_ref.header.frame_id = "/base";
    
    pub_real_pose.publish(pose_ref);
    loop_rate.sleep();
    // FIXME This is a blocking call that should be avoided in the future
    ros::spinOnce();

  } // end while(ros::ok())
  return 1;
}
