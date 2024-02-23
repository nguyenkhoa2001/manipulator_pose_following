#include <iostream>

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "manipulator_pose_following/InitPoint.h"

bool g_is_init_done = false;

const int STATE_IDLE = 0;
const int STATE_POSE_FOLLOW = 1;
const int STATE_STOP = 2;
const int STATE_INIT = 3;

int g_state = STATE_IDLE;

ros::Time g_t_start;

geometry_msgs::PoseStamped g_pose_init;

void print_out_g_pose();
bool cb_init_point_accept(manipulator_pose_following::InitPoint::Request &req, manipulator_pose_following::InitPoint::Response &res);

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_service");

    ros::NodeHandle nh;
    ros::NodeHandle nh_startstop;

    ros::CallbackQueue queue_startstop;
    nh_startstop.setCallbackQueue(&queue_startstop);
    ros::AsyncSpinner spin_startstop(1, &queue_startstop);
    spin_startstop.start();

    ros::ServiceServer srv_init = nh_startstop.advertiseService(
      "/pose_following/init_point", cb_init_point_accept);

    int count = 0;
    while (ros::ok())
    {
        std::cout << "In main loop: " << count << std::endl;
        ros::Duration(1.0).sleep();
        count++;
        if(count == 10) g_is_init_done = true;
        if (g_is_init_done == true)
        {
            print_out_g_pose();
        }
        
    }
    return 0;
}

bool cb_init_point_accept(
  manipulator_pose_following::InitPoint::Request &req, 
  manipulator_pose_following::InitPoint::Response &res) {
    std::cout << "Receiving a request" << std::endl;

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

void print_out_g_pose()
{
    std::cout << "G_pose coor: x=" << g_pose_init.pose.position.x 
              << " y=" << g_pose_init.pose.position.y 
              << " z=" << g_pose_init.pose.position.z << std::endl;
}