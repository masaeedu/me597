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
#include <math.h>
#include <Eigen/Dense>

ros::Publisher marker_pub;

#define TAGID 0

using namespace std;

double ipsYaw, ipsX, ipsY;

//Callback function for the Position topic (LIVE)

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped & msg)
{
	//This function is called when a new position message is received
	double X = msg.pose.pose.position.x; // Robot X psotition
	double Y = msg.pose.pose.position.y; // Robot Y psotition
 	double Yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw

  ips_x = X;
  ips_y = Y;
  ips_yaw = Yaw;

	std::cout << "X: " << X << ", Y: " << Y << ", Yaw: " << Yaw << std::endl ;
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

void drawLineSegment(int k, geometry_msgs::Point start_point, geometry_msgs::Point end_point)
{
   visualization_msgs::Marker lines;
   lines.header.frame_id = "/map";
   lines.id = k; //each curve must have a unique id or you will overwrite an old ones
   lines.type = visualization_msgs::Marker::LINE_STRIP;
   lines.action = visualization_msgs::Marker::ADD;
   lines.ns = "line_segments";
   lines.scale.x = 0.1;
   lines.color.r = 1.0;
   lines.color.b = 0.2*k;
   lines.color.a = 1.0;

   lines.points.push_back(start_point);
   lines.points.push_back(end_point);

   //publish new line segment
   marker_pub.publish(lines);
}

//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid& msg)
{
    //This function is called when a new map is received
    
    //you probably want to save the map into a form which is easy to work with
}


void stopRobot(){

  geometry_msgs::Twist vel;
  vel.linear.x = 0;
  vel.linear.y = 0;
  vel.angular.z = 0;
  velocity_publisher.publish(vel);

}

vector<double> previousErrorYaw;
int kp = 2;
int ki = 0.5;
int kd = 0.3;

// PID Yaw (takes current yaw and positions, and uses goal position to determine error)
double yawController(double currentYaw, double ipsX, double ipsY, double targetX, double targetY){

  //opposite over adjacent distance is angle to destination
  double targetYaw = (targetY - ipsY)/(targetX - ipsX);

  //error feedback for yaw
  double errorInYaw = targetYaw - currentYaw;
  
  //record new error
  previousErrorYaw.push_back(errorInYaw);

  //total error over time for integral
  double totalErrorYaw = 0;

  for(int i = 0; i < previousErrorYaw.size; i++){
      totalErrorYaw += previousErrorYaw[i];
  }

  //change in error for derivative
  double changeInErrorYaw = currentYaw - previousErrorYaw.back();

  //new output yaw
  double outputYaw = kp*errorInYaw + ki*totalErrorYaw + kd*changeInErrorYaw;
  return outputYaw;

}

bool correctYaw(double currentYaw, double desiredYaw)

    geometry_msgs::Twist vel;

    if(abs(currentYaw - desiredYaw) < 0.05){
      return true;
    }
     else{
      vel.angular.z = 0.01;
    }

    velocity_publisher.publish(vel);

}

bool goForward(double speed){
  geometry_msgs::Twist vel;
  vel.angular.z = 0;
}


int main(int argc, char **argv)
{
	//Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
    ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
    
    //Velocity control variable
    geometry_msgs::Twist vel;

    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate

    geometry_msgs::Point start_, end_;

    start_.x = 12.0;
    start_.y = 12.0;
    start_.z = 0.0;
    end_.x = -12.0;
    end_.y = -12.0;
    end_.z = 0.0;
	
    geometry_msgs::Point waypoint1, waypoint2, waypoint3;

    waypoint1.x = 4;
    waypoint1.y = 0;
    waypoint1.z = 0;

    waypoint2.x = 8;
    waypoint2.y = -4;
    waypoint2.z = 3.14;

    waypoint3.x = 8;
    waypoint3.y = 0; 
    waypoint3.z = -1.57;

    bool outsideXThreshold, outsideYThreshold, outsideYawThreshold = true;

    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages

      //draw curves
      drawCurve(1);
      drawCurve(2);
      drawCurve(4);
      drawLineSegment(5,start_,end_);

      //here's my current pose
      //run prm to get path to next waypoint

      //am I outside the error bounds of the waypoint coordinate (+/- 0.25m)
      if(outsideXThreshold || outsideYThreshold){
        //if not, move towards destination
        vel.linear.x = 0.1;
        //update angular velocity using PID controller
        vel.angular.z = ipsYaw + yawController(ipsYaw,waypoint1.x,waypoint1.y,ipsX,ipsY);
      }
      else{
        //if within error bounds, stop moving
        vel.linear.x = 0;
        vel.angular.z = 0;
      }

      //check if within error bound of x waypoint
      if(abs(waypoint1.x - ipsX) < 0.25){
        outsideXThreshold = false;
      }
      //check if within error bounds of y waypoint
      if(abs(waypoint1.y - ipsY) < 0.25){
        outsideYThreshold = false;
      }

      velocity_publisher.publish(vel);
    }

    return 0;
}
