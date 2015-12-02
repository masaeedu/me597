//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 2
// It outlines the basic setup of a ros node and the various 
// inputs and outputs needed for this lab
// 
// Author: James Servos 
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <Eigen/Dense>
#include <ctime>
#include "prm.cpp"

using namespace std;

typedef tuple<double, double> coord;

#define TAGID 0

ros::Publisher marker_pub;
Eigen::Vector3d IPS;
int m = 100;
Eigen::MatrixXd PRM_x_y(2,m);
nav_msgs::OccupancyGrid og;

double ipsYaw, ipsX, ipsY;

//controller global variables
vector<double> previousErrorYaw;
double kp = 0.5;
double ki = 0;
double kd = 0;

//Callback function for the Position topic (LIVE)

void pose_callback_live(const geometry_msgs::PoseWithCovarianceStamped & msg)
{
    //This function is called when a new position message is received
    double X = msg.pose.pose.position.x; // Robot X psotition
    double Y = msg.pose.pose.position.y; // Robot Y psotition
     double Yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw

    std::cout << "X: " << X << ", Y: " << Y << ", Yaw: " << Yaw << std::endl ;
}

//Callback function for the Position topic (SIMULATION)
void pose_callback_sim(const gazebo_msgs::ModelStates& msg) 
{

    int i;
    for(i = 0; i < msg.name.size(); i++) 
    {
            if(msg.name[i] == "mobile_base") break;
    }
    //IPS << msg.pose[i].position.x +gauss(0.1),msg.pose[i].position.y +gauss(0.1),tf::getYaw(msg.pose[i].orientation)+gauss(0.01);
    IPS << msg.pose[i].position.x ,msg.pose[i].position.y ,tf::getYaw(msg.pose[i].orientation);
    ROS_INFO("pose_callback X: %f Y: %f Yaw: %f", IPS[0], IPS[1], IPS[2]);
}

double constrainAngle(double x){
    x = fmod(x,2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x;
}

//yaw PID controller
double yawController(double currentYaw, double targetX, double targetY, double ipsX, double ipsY){

  //opposite over adjacent distance is angle to destination
  double targetYaw = atan2((targetY - ipsY),(targetX - ipsX));

  targetYaw = targetYaw;

  cout << "targetYaw: " << targetYaw << endl;

  //error feedback for yaw
  double errorInYaw = targetYaw - currentYaw;
  
  cout << "errorYaw: " << errorInYaw << endl;
 
  //change in error for derivative
  //double changeInErrorYaw = errorInYaw - previousErrorYaw.back();
  double changeInErrorYaw = 0;
  //record new error
  previousErrorYaw.push_back(errorInYaw);

  //total error over time for integral
  double totalErrorYaw = 0;

  for(int i = 0; i < previousErrorYaw.size(); i++){
      totalErrorYaw += previousErrorYaw[i];
  }

  //new output yaw
  double outputYaw = kp*errorInYaw + ki*totalErrorYaw + kd*changeInErrorYaw;
  return outputYaw;

}

//Example of drawing a curve

void drawCurve(int k) 
{
   // Curves are drawn as a series of stright lines
   // Simply sample your curves into a series of points

   double x = 0;
   double y = 0;
   double steps = 50;

   visualization_msgs::Marker lines;
   lines.header.frame_id = "/map";
   lines.id = k; //each curve must have a unique id or you will overwrite an old ones
   lines.type = visualization_msgs::Marker::LINE_STRIP;
   lines.action = visualization_msgs::Marker::ADD;
   lines.ns = "curves";
   lines.scale.x = 0.1;
   lines.color.r = 1.0;
   lines.color.b = 0.2*k;
   lines.color.a = 1.0;

   //generate curve points
   for(int i = 0; i < steps; i++) {
       geometry_msgs::Point p;
       p.x = x;
       p.y = y;
       p.z = 0; //not used
       lines.points.push_back(p); 

       //curve model
       x = x+0.1;
       y = sin(0.1*i*k);   
   }

   //publish new curve
   marker_pub.publish(lines);
}
    
void drawPoint(int k, geometry_msgs::Point p) 
{
    visualization_msgs::Marker points; //Define points
    points.header.frame_id = "/map";
    points.header.stamp = ros::Time::now();
    points.ns = "lab3";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = k;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.2;
    points.scale.y = 0.2;
    points.color.b = 1.0f;
    points.color.a = 1.0;
}

void drawLineSegment(int k, geometry_msgs::Point start_point, geometry_msgs::Point end_point)
{
   visualization_msgs::Marker lines;
   lines.header.frame_id = "/map";
   lines.id = k; //each curve must have a unique id or you will overwrite an old ones
   lines.type = visualization_msgs::Marker::LINE_STRIP;
   lines.action = visualization_msgs::Marker::ADD;
   lines.ns = "lab3";
   lines.scale.x = 0.1;
   lines.color.r = 1.0;
   lines.color.b = 0.2*k;
   lines.color.a = 1.0;

   lines.points.push_back(start_point);
   lines.points.push_back(end_point);

   //publish new segment line
   marker_pub.publish(lines);
}

//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid msg)
{
    //This function is called when a new map is received
    og = msg;
}

void drawPrm(tuple<vector<coord>, map<int, map<int, double>>> prmResult) {
	// Plot the points
	int pointId = 0;
    vector<coord> coords = std::get<0>(prmResult);
    int num_nodes=coords.size();
    std::cout<<num_nodes<<std::endl;
    for (int i=0; i<num_nodes; ++i)
    {
        coord item = coords[i]; 
        double x = std::get<0>(item); 
        double y = std::get<1>(item);
        std::cout<<"X-val"<<x<<" Y-val"<<y<<std::endl;
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        drawPoint(pointId++, p);
        //std::cin.get();
    }
	
	// Plot the edges
    map<int, map<int, double>> edges = std::get<1>(prmResult);
    int num_edges = edges.size();
    std::cout<<num_edges<<std::endl;
    for (auto kv1: edges)
    {
        int idx1 = kv1.first;
        coord c1 = coords[idx1];
        map<int, double> connections = kv1.second;

	std::cout << "num connections: " << connections.size() << std::endl;
        for(auto kv2: connections) {
            geometry_msgs::Point start_, end_;

            int idx2 = kv2.first;
            coord c2 = coords[idx2];
            
            // Get start and end coordinates
            start_.x = std::get<0>(c1);
            start_.y = std::get<1>(c1);    
            end_.x = std::get<0>(c2);
            end_.y = std::get<1>(c2);
            
            // Draw line between c1 and c2
            drawLineSegment(pointId++, start_, end_);
        }
    } 
}


int main(int argc, char **argv)
{
    ROS_INFO("Probability Roadmap Plots Running");
    srand((unsigned)time(0));
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;
    
    auto map_sub = n.subscribe("/map", 1, map_callback);
    auto pose_sub_live = n.subscribe("/indoor_pos", 1, pose_callback); //switch for live tests
    auto pose_sub_sim = n.subscribe("/gazebo/model_states", 1, pose_callback_sim);
    auto velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 20);
    
    //Velocity control variable
    geometry_msgs::Twist vel;

    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate

	// Get the occupancy grid
    auto grid = *ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map", n, ros::Duration(1.0));
	
	// Waypoints we need to hit
	auto waypoints = vector<coord>{coord{4.0,0.0},coord{8.0,-4.0},coord{8.0,0.0}};
	
	// Get the probabilistic roadmap
    ROS_INFO("Attempting to get PRM");
	bool success = true;
	int attempt = 0;
	tuple<vector<coord>, map<int, map<int, double>>> result;

	do {
		attempt++;
        success = true;
		std::cout << "Attempt " << attempt << ": " << std::endl;
		result = prm(m, 6, coord{IPS[0],IPS[1]}, waypoints, grid);
		drawPrm(result);
        std::cin.get();
        
		for (auto wp: vector<int>{1, 2, 3}) {
			success = success && a_star(0, wp, std::get<0>(result), std::get<1>(result)).size() > 0;
		}
	} while (!success);
    ROS_INFO("Obtained the PRM");

    while (ros::ok())
    {
        loop_rate.sleep(); //Maintain the loop rate
        ros::spinOnce();   //Check for new messages
    }
}
