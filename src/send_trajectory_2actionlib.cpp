#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <cstdlib>

// Typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;

// Function to solve IK using MoveIt
bool solveIK(const std::vector<double>& end_effector_position, std::vector<double>& joint_positions, const moveit::core::RobotStatePtr& kinematic_state, const moveit::core::JointModelGroup* joint_model_group)
{
    // Set the desired pose for the end-effector
    Eigen::Isometry3d desired_pose = Eigen::Isometry3d::Identity();
    desired_pose.translation() << end_effector_position[0], end_effector_position[1], end_effector_position[2];

    // Solve IK
    bool found_ik = kinematic_state->setFromIK(joint_model_group, desired_pose, 0.1);

    if (found_ik)
    {
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_positions);
        std::cout << "New position = " << joint_positions[0] << ","
                  << joint_positions[1] << ","
                  << joint_positions[2] << ","
                  << joint_positions[3] << ","
                  << joint_positions[4] << ","
                  << joint_positions[5] << std::endl;

        return true;
    }
    else
    {
        ROS_WARN("Failed to find IK solution for the given end-effector position.");
        return false;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_trajectory_action_client");
    ros::NodeHandle nh;

    // Create the action client
    TrajectoryClient client("joint_trajectory_action", true);
    ROS_INFO("Waiting for driver's action server to become available ..");
    client.waitForServer();
    ROS_INFO("Connected to trajectory action server");

    // Setup MoveIt for IK
    robot_model_loader::RobotModelLoader model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = model_loader.getModel();
    printf("Model Frame: %s\n", kinematic_model->getModelFrame().c_str()); // printf instead of ROS_INFO

    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    std::string group_st = "arm";
    moveit::planning_interface::MoveGroupInterface arm(group_st);
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(group_st);

    // Setup simple goal
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names = {"joint_1_s", "joint_2_l", "joint_3_u", "joint_4_r", "joint_5_b", "joint_6_t"};

    // Get current robot joint states
    sensor_msgs::JointStateConstPtr robot_joint_states = ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");

    // Check if joint names match
    std::set<std::string> expected_joint_names(goal.trajectory.joint_names.begin(), goal.trajectory.joint_names.end());
    std::set<std::string> current_joint_names(robot_joint_states->name.begin(), robot_joint_states->name.end());
    if (expected_joint_names != current_joint_names)
    {
        ROS_FATAL("Mismatch between joints specified and seen in current JointState. Cannot continue.");
        return 1;
    }

    // Open the data file
    std::ifstream infile("/home/khoa/catkin_ws/src/manipulator_pose_following/new_data.txt");
    if (!infile.is_open())
    {
        ROS_FATAL("Failed to open data file.");
        return 1;
    }

    std::string line;
    std::vector<trajectory_msgs::JointTrajectoryPoint> points;

    // Read each line and parse the end-effector positions and time intervals
    std::getline(infile, line); // Read the first line to move to the first point

    std::istringstream iss(line);
    std::vector<double> end_effector_position(3);
    double dt;

    char comma;
    iss >> end_effector_position[0] >> comma >> end_effector_position[1] >> comma >> end_effector_position[2] >> comma >> dt;
    end_effector_position[2] += 0.15;
    std::cout << "The first position x = " << end_effector_position[0] << " y = " << end_effector_position[1]
              << " z = " << end_effector_position[2] << std::endl;
    // Solve IK for the first end-effector position
    std::vector<double> joint_positions(goal.trajectory.joint_names.size());
    if (!solveIK(end_effector_position, joint_positions, kinematic_state, joint_model_group))
    {
        ROS_FATAL("Failed to solve IK for the first position (%f, %f, %f).", end_effector_position[0], end_effector_position[1], end_effector_position[2]);
        return 1;
    }

    // Add the current robot position as the first point
    trajectory_msgs::JointTrajectoryPoint start_point;
    start_point.positions = robot_joint_states->position;
    start_point.velocities.resize(robot_joint_states->position.size(), 0.0);
    start_point.time_from_start = ros::Duration(0.0);

    // Create a point for moving to the first end-effector position
    trajectory_msgs::JointTrajectoryPoint first_point;
    first_point.positions = joint_positions;
    first_point.velocities.resize(joint_positions.size(), 0.0);  // Zero target velocity
    first_point.time_from_start = ros::Duration(5.0);  // Move to the first position after 5 seconds

    points.push_back(start_point);
    points.push_back(first_point);

    // Read the rest of the file and create trajectory points
    while (std::getline(infile, line))
    {
        std::istringstream iss(line);
        iss >> end_effector_position[0] >> comma >> end_effector_position[1] >> comma >> end_effector_position[2] >> comma >> dt;
        end_effector_position[2] += 0.15;

        // Solve IK for the end-effector position
        if (!solveIK(end_effector_position, joint_positions, kinematic_state, joint_model_group))
        {
            ROS_WARN("Failed to solve IK for position (%f, %f, %f). Skipping this point.", end_effector_position[0], end_effector_position[1], end_effector_position[2]);
            continue;
        }

        // Create a trajectory point
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = joint_positions;
        point.velocities.resize(joint_positions.size(), 0.0);  // Zero target velocity
        point.time_from_start = ros::Duration(dt);

        points.push_back(point);

        // Update the total time with the current point's duration
    }

    infile.close();

    // Loop through each point and send it as a separate goal
    double lastest_time_point = 0.0;
    for (auto& point : points)
    {
        control_msgs::FollowJointTrajectoryGoal single_goal;
        single_goal.trajectory.joint_names = goal.trajectory.joint_names;

        // Add the current position of the robot as the starting point
        ros::spinOnce();
        sensor_msgs::JointStateConstPtr current_joint_states = ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");
        trajectory_msgs::JointTrajectoryPoint current_point;
        current_point.positions = current_joint_states->position;
        current_point.velocities.resize(current_joint_states->position.size(), 0.0);
        current_point.time_from_start = ros::Duration(0.0);

        single_goal.trajectory.points.push_back(current_point);
        single_goal.trajectory.points.push_back(point);

        // Submit the goal
        ROS_INFO("Submitting goal ..");
        client.sendGoal(single_goal);
        ROS_INFO("Waiting for completion ..");
        client.waitForResult();

        if (client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_WARN("The goal did not succeed.");
        }
        else
        {
            ROS_INFO("Goal succeeded.");
        }

        // Wait for the robot to stop moving
        ros::Duration(0.05).sleep();  // Adjust the sleep duration as needed
    }

    ROS_INFO("All points processed.");
    return 0;
}
