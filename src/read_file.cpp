//ROS include file
#define NODE_LOOP_RATE      40
#define MAX_BUFFER_TO_BE_SENT 100

#define SIZE_DATA_POINT 5
// Data point to be sent -- Field index
#define POSITION_X_INDEX 0
#define POSITION_Y_INDEX 1
#define POSITION_Z_INDEX 2
#define DELTA_T_INDEX 3
#define VEL_CAL_INDEX 4



#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include "manipulator_pose_following/ReplyInt.h"
#include "manipulator_pose_following/PlannedPath.h"
#include "manipulator_pose_following/InitPoint.h"

#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <cstdint>
#include <boost/circular_buffer.hpp>


// ==== IO Functions Declaration Section  ====
void extractNextDataLine(std::ifstream& data_file, std::vector<double> & result_data_point);
bool publishSubPoint(const std::vector<std::vector<double>> *sub_point_list, ros::Publisher &publish_handler, ros::Rate &loop_rate);

// ==== Algorithm Function Declararion Section ====
void calcSubDataPoint(const std::vector<double>& previous_data_point, const std::vector<double>& current_data_point, std::vector<std::vector<double>>* &sub_point_list);

double g_loop_interval = 1.0/NODE_LOOP_RATE;
uint8_t g_track_init_state = 0;
std::vector<double> g_pre_data_point{0.0, 0.0, 0.0, 0.0, 0.0}; // This vector size must match with data point size

int main(int argc, char** argv)
{
    std::cout << "Setting up" << std::endl;
    ros::init(argc, argv, "read_planned_path_file");
    ros::NodeHandle nh;


    //Service Client
    ros::ServiceClient init_handle_client = nh.serviceClient<manipulator_pose_following::InitPoint>("/pose_following/init_point");
    manipulator_pose_following::InitPoint init_reply_message;

    //Publisher
    ros::Publisher desired_position_pub = nh.advertise<manipulator_pose_following::PlannedPath>("/user_defined/desired_path_point", MAX_BUFFER_TO_BE_SENT);

    // Data file section
    std::ifstream data_file("/home/khoa/catkin_ws/src/manipulator_pose_following/new_data.txt");
    std::cout << "Set up done for node's communication !" << std::endl;

    // Counting variable , after a successful reading line, increased this by 1
    int data_line_counter = 0;
    
    ros::Rate loop_rate(NODE_LOOP_RATE);

    // Program loop
    while(ros::ok())
    {
        // Check the EOF
        if (data_file.eof()) {
            // if EOF then loop inside, keeping publish the last point
            manipulator_pose_following::PlannedPath last_point;
            last_point.x = g_pre_data_point[POSITION_X_INDEX];
            last_point.y = g_pre_data_point[POSITION_Y_INDEX];
            last_point.z = g_pre_data_point[POSITION_Z_INDEX] + 0.05;
            last_point.delta_time = g_pre_data_point[DELTA_T_INDEX];
            last_point.calc_vel = g_pre_data_point[VEL_CAL_INDEX];
            desired_position_pub.publish(last_point);
            loop_rate.sleep();
            continue;
        }
        
        // Indicate to read the first line from file
        if (data_line_counter == 0) {
            std::vector<double> data_point;
            extractNextDataLine(data_file, data_point);
            g_pre_data_point = data_point;
            ros::Duration(g_loop_interval).sleep();
            ++data_line_counter;
        }

        // Call init service, continue run inside this if transfer state is ready (do )
        if (g_track_init_state == 0)
        {
            std::cout << "Doing call to init point service" << std::endl;
            init_reply_message.request.x = g_pre_data_point[POSITION_X_INDEX];
            init_reply_message.request.y = g_pre_data_point[POSITION_Y_INDEX];
            init_reply_message.request.z = g_pre_data_point[POSITION_Z_INDEX] + 0.05;
            init_handle_client.call(init_reply_message);
            ros::Duration(1.0).sleep();
            if(init_reply_message.response.transfer_state != 10) continue;
            else
            {
                g_track_init_state = 1;
            }
        }
        
        std::vector<double> data_point;
        std::cout << "Right now data line: << " << data_line_counter << std::endl;
        extractNextDataLine(data_file, data_point);
        if(data_point.empty()) continue;
        std::cout << "Right now point: x= " << data_point[POSITION_X_INDEX] 
                                << "\ty= " << data_point[POSITION_Y_INDEX] 
                                << "\tz= " << data_point[POSITION_Z_INDEX] 
                                << "\tdt= " << data_point[DELTA_T_INDEX] << std::endl;
        std::vector<std::vector<double>> *subpoint_list = nullptr;
        calcSubDataPoint(g_pre_data_point, data_point, subpoint_list);
        publishSubPoint(subpoint_list, desired_position_pub, loop_rate);
        std::cout << "Size sublist = " <<subpoint_list->size() << std::endl;
        delete subpoint_list;
        data_line_counter++;
        g_pre_data_point = data_point;
    }
    return 0;
}


/* ==== IO Functions Implementation Section  ====
   For the purpose of:
   - Read data-file contains points from planned path.*/
void extractNextDataLine(std::ifstream& data_file, std::vector<double> & result_data_point)
{
    // Put a line into `single_data_line_string` variable
    std::string single_data_line_string;
    std::getline(data_file, single_data_line_string);
    // Confirm that it still a valid string
    // std::size_t found = single_data_line_string.find(',');
    // if (found == std::string::npos) return;
    // Start to extract data_point from string - command to seperated points
    std::stringstream data_line_stream(single_data_line_string);
    std::string number_string; // a place-holder for getline func to work
    while (std::getline(data_line_stream, number_string, ','))
    {
        double number = std::stod(number_string);
        result_data_point.push_back(number);
    }
    return;
}

bool publishSubPoint(const std::vector<std::vector<double>> *sub_point_list, ros::Publisher &publish_handler, ros::Rate &loop_rate)
{
    size_t number_of_points = sub_point_list->size();
    for (size_t i = 0; i < number_of_points; i++)
    {
        const std::vector<double>& sub_point = (*sub_point_list)[i];
        manipulator_pose_following::PlannedPath sub_point_msg;
        sub_point_msg.x = sub_point[POSITION_X_INDEX];
        sub_point_msg.y = sub_point[POSITION_Y_INDEX];
        sub_point_msg.z = sub_point[POSITION_Z_INDEX] + 0.05;
        sub_point_msg.delta_time = sub_point[DELTA_T_INDEX];
        sub_point_msg.calc_vel = sub_point[VEL_CAL_INDEX];
        publish_handler.publish(sub_point_msg);
        std::cout << "\t sp[" << i << "] x=" << sub_point[POSITION_X_INDEX] 
                    << "\t y=" << sub_point[POSITION_Y_INDEX] 
                    << "\t z=" << sub_point[POSITION_Z_INDEX] 
                    << "\t dt =" << sub_point[DELTA_T_INDEX] << std::endl;
        loop_rate.sleep();
    }
    return true;
}

void calcSubDataPoint(const std::vector<double>& previous_data_point, const std::vector<double>& current_data_point, std::vector<std::vector<double>>* &sub_point_list)
{
    // Extract the time (dt) variable between points, to generate a consistent rate of point
    // For example: dt = 0.3s and NODE_LOOP_RATE = 60HZ => sperate 2 point into 0.3/(1/60) = 18 points
    double dt = current_data_point[DELTA_T_INDEX];
    int number_of_points = floor(dt / g_loop_interval);
    double linear_velocity = std::sqrt(
        std::pow(current_data_point[POSITION_X_INDEX] - previous_data_point[POSITION_X_INDEX], 2) +
        std::pow(current_data_point[POSITION_Y_INDEX] - previous_data_point[POSITION_Y_INDEX], 2) +
        std::pow(current_data_point[POSITION_Z_INDEX] - previous_data_point[POSITION_Z_INDEX], 2)
    ) / dt;
    // Sliding 2 points into `number_of_points` subpoints
    double position_x_step = (current_data_point[POSITION_X_INDEX] - previous_data_point[POSITION_X_INDEX]) / (number_of_points - 1);
    double position_y_step = (current_data_point[POSITION_Y_INDEX] - previous_data_point[POSITION_Y_INDEX]) / (number_of_points - 1);
    double position_z_step = (current_data_point[POSITION_Z_INDEX] - previous_data_point[POSITION_Z_INDEX]) / (number_of_points - 1);
    sub_point_list = new std::vector<std::vector<double>>(number_of_points);
    for (size_t i = 0; i < number_of_points; i++)
    {
        std::vector<double> sub_point(SIZE_DATA_POINT); //size of 4
        sub_point[POSITION_X_INDEX] = previous_data_point[POSITION_X_INDEX] + position_x_step * i;
        sub_point[POSITION_Y_INDEX] = previous_data_point[POSITION_Y_INDEX] + position_y_step * i;
        sub_point[POSITION_Z_INDEX] = previous_data_point[POSITION_Z_INDEX] + position_z_step * i;
        sub_point[DELTA_T_INDEX] = dt;
        sub_point[VEL_CAL_INDEX] = linear_velocity;
        sub_point_list->at(i) = sub_point;
    }
    // for (size_t i = 0; i < number_of_points; i++)
    // {
    //     std::vector<double> sub_point = sub_point_list->at(i);
    //     std::cout << "\t sp[" << i << "] x=" << sub_point[POSITION_X_INDEX] 
    //                 << "\t y=" << sub_point[POSITION_Y_INDEX] 
    //                 << "\t z=" << sub_point[POSITION_Z_INDEX] 
    //                 << "\t dt =" << sub_point[DELTA_T_INDEX] << std::endl;
    // }
}


