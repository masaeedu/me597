//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 2
// It outlines the basic setup of a ros node and the various
// inputs and outputs needed for this lab
//
// Author: James Servos
// Edited: Nima Mohajerin
//
// //////////////////////////////////////////////////////////

#include <Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include "utils.h"

using namespace Eigen;

// Mapping parameters
const int cell_width = 200;
const int cell_height = 200;
const double width = 10;
const double height = 10;
const double alpha = 0.1;
const double p_0 = 0.5;
const double p_occ = 0.9;
const double p_free = 0.3;
nav_msgs::OccupancyGrid og;

// State variables
Vector3d pose;

Vector2i get_indices(Vector2d coordinates) {
    return Vector2i((int)(coordinates[0] * cell_width / width), (int)(coordinates[1] * cell_height / height));
}

Vector2d get_coordinates(Vector2i indices) {
    return Vector2d((0.5 + indices[0]) * width / cell_width, (0.5 + indices[1]) * height / cell_height);
}

//Callback function for the Position topic (SIMULATION)
void pose_callback(const gazebo_msgs::ModelStates& msg)
{
    int i;
    for(i = 0; i < msg.name.size(); i++)
        if(msg.name[i] == "mobile_base") 
            break;
    
    pose << msg.pose[i].position.x, msg.pose[i].position.y, tf::getYaw(msg.pose[i].orientation);
}

double inverse_range_sensor_model(int i_x, int i_y, double range, double max_range) {
    auto coordinates = get_coordinates(Vector2i(i_x, i_y));
    
    // Find distance to cell and angle of cell relative to robot pose
    double r = sqrt(pow(coordinates[1] - pose[1], 2) + pow(coordinates[0] - pose[0], 2));
    double phi = atan2(coordinates[1] - pose[1], coordinates[0] - pose[0]) - pose[2];
    
    // If the range is NaN, mark all cells closer than max range free, otherwise mark as unknown
    if (isnan(range)) {
        if (r < max_range) {
            return p_free;
        } else {
            return p_0;
        }
    }
    
    // If the range is not NaN, mark the cell as unknown if it exceeds the measured and/or max range
    if ((range < max_range) && (std::abs(r - range) < (alpha / 2))) {
        return p_occ;
    } else if (r > std::min(max_range, range + alpha / 2)) {
        return p_0;
    } else if (r <= range) {
        return p_free;
    }
}

double logodds(double sddogol) {
    return log(sddogol / (1 - sddogol));
}

double sddogol(double logodds) {
    return 1 - 1 / (1 + exp(logodds));
}

int get_og_index(int x, int y) {
    return (x + cell_width / 2) + (y + cell_height / 2) * cell_width;
}

void scan_callback(const sensor_msgs::LaserScan& msg)
{
    const double max_range = 2.0;
    std::vector<int> x { };
    std::vector<int> y { };
    std::vector<double> z { }; // measurements
    
    auto curr_cell = get_indices(pose.head<2>());
    
    // For each ray, find the cells that need to be updated
    for (int i = 0; i < msg.ranges.size(); i++) {
        if (i % 30 == 0) {
            double theta = msg.angle_min + i * msg.angle_increment + pose[2];
            
            // Find location of cutoff point along ray
            Vector2d horizon;
            horizon[0] = pose[0] + (max_range) * cos(theta);
            horizon[1] = pose[1] + (max_range) * sin(theta);
            
            // Get indices of farther point along ray
            auto horizon_cell = get_indices(horizon);
            
            // Apply Bresenham's algorithm to get affected cells
            int num_cells = x.size();
            bresenham(curr_cell[0], curr_cell[1], horizon_cell[0], horizon_cell[1], x, y);
            
            // Fill in corresponding measurements
            for (int j = num_cells; j < x.size(); j++) {
                z.push_back(msg.ranges[i]);
            }
        }
    }
    
    // Update cells identified through Bresenham's algorithm
    for (int i = 0; i < x.size(); i++) {        
        // Perform log odds update
        int idx = get_og_index(x[i], y[i]);
        
        double present_val = (double)og.data[idx] / 100;
        double inv_rng_sense = inverse_range_sensor_model(x[i], y[i], z[i], max_range);
        double prior = p_0;
        double val = sddogol(logodds(present_val) + logodds(inv_rng_sense) - logodds(prior)) * 100;
        og.data[idx] = val;
    }
}


int main(int argc, char **argv)
{
    // Initialize the ROS framework
    ros::init(argc, argv, "mapping");
    ros::NodeHandle n;
    
    // The message
    og.header.frame_id = "world";
    og.info.resolution = width / cell_width;
    og.info.width = cell_width;
    og.info.height = cell_height;
    og.info.origin.position.x = -width / 2;
    og.info.origin.position.y = -height / 2;
    og.data.resize(cell_width * cell_height, 50);
    
    // Kinect subscription
    auto scan_sub = n.subscribe("/scan", 1, scan_callback);
    auto ips_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
    
    // Map publisher
    auto map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    
    // Set the loop rate
    ros::Rate loop_rate(10);    //20Hz update rate
    while (ros::ok())
    {
        loop_rate.sleep(); //Maintain the loop rate
        ros::spinOnce();   //Check for new messages
        
        og.header.stamp = ros::Time::now();
        map_pub.publish(og); // Publish Occupancy Grid
    }
    
    return 0;
}