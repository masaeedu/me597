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

//Callback function for the Position topic (LIVE)

//void pose_callback(const geometry_msgs::PoseWithCovarianceStamped & msg)
//{
//	//This function is called when a new position message is received
//	double X = msg.pose.pose.position.x; // Robot X psotition
//	double Y = msg.pose.pose.position.y; // Robot Y psotition
 //	double Yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
//
//	std::cout << "X: " << X << ", Y: " << Y << ", Yaw: " << Yaw << std::endl ;
//}
//Callback function for the Position topic (SIMULATION)
void pose_callback(const gazebo_msgs::ModelStates& msg) 
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
   //marker_pub.publish(lines);

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

void PRMpoints() //this function just sets the random points at beginning of program
{
	for (int i=0; i<m; ++i)
	{
		PRM_x_y.col(i)[0] = 10*(((double)rand()/RAND_MAX)-0.5);
		PRM_x_y.col(i)[1] = 10*(((double)rand()/RAND_MAX)-0.5);
		//ROS_INFO("%f",PRM_x_y.col(i)[0]);
		//ROS_INFO("%f",PRM_x_y.col(i)[1]);
	}
}

//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid msg)
{
    //This function is called when a new map is received
    og = msg;
    //you probably want to save the map into a form which is easy to work with
}


int main(int argc, char **argv)
{
	ROS_INFO("Probability Roadmap Plots Running");
	srand((unsigned)time(0));
	ros::init(argc,argv,"main_control");
	ros::NodeHandle n;
	ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);

	//ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback); //switch for live tests
	ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
	ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 20);
    
	//Velocity control variable
	geometry_msgs::Twist vel;

	//Set the loop rate
	ros::Rate loop_rate(20);    //20Hz update rate

	geometry_msgs::Point start_, end_;
	PRMpoints();
	visualization_msgs::Marker points; //Define points
	points.header.frame_id = "/map";
	points.header.stamp = ros::Time::now();
	points.ns = "lab3";
	points.action=visualization_msgs::Marker::ADD;
	points.pose.orientation.w=1.0;
	points.id=0;
	points.type=visualization_msgs::Marker::POINTS;
	points.scale.x = 0.2;
	points.scale.y = 0.2;
	points.color.b = 1.0f;
	points.color.a = 1.0;
	auto grid = *ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map", n, ros::Duration(1.0));
	ROS_INFO("Before the PRM");
	std::cout << grid.info.resolution << endl;
	auto result = prm(m, 6, coord{IPS[0],IPS[1]}, vector<coord>{coord{4.0,0.0},coord{8.0,-4.0},coord{8.0,0.0}}, grid);

	vector<coord> coords = std::get<0>(result);
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
		points.points.push_back(p);
		marker_pub.publish(points);
		//std::cin.get();
	}
	map<int, map<int, double>> edges = std::get<1>(result);
	int num_edges = edges.size();
	std::cout<<num_edges<<std::endl;
	int j=1;
	for (auto kv1: edges)
	{
		int idx1 = kv1.first;
		coord c1 = coords[idx1];
		map<int, double> connections = kv1.second;

                std::cout << "num connections: " << connections.size() << std::endl;
		for(auto kv2: connections) {
			visualization_msgs::Marker lines;
			lines.header.frame_id = "/map";
			lines.scale.x = 0.1;
			lines.color.r = 1.0;
			lines.color.b = 0.2;
			lines.color.a = 1.0;
			lines.ns = "lab3";
			lines.action = visualization_msgs::Marker::ADD;
			lines.id =j;
			lines.type = visualization_msgs::Marker::LINE_STRIP;
			geometry_msgs::Point start_, end_;
			std::cin.get();
			int idx2 = kv2.first;
			coord c2 = coords[idx2];
			start_.x = std::get<0>(c1);
			start_.y = std::get<1>(c1);	
			end_.x = std::get<0>(c2);
			end_.y = std::get<1>(c2);
			// Draw line between c1 and c2
			lines.points.push_back(start_);
			lines.points.push_back(end_);
			marker_pub.publish(lines);
			//std::cin.get();
			j++;
		}
	}

	//coord item = coords[0]; 
	//double x = std::get<0>(item); 
	//double y = std::get<1>(item);

	ROS_INFO("Passed the PRM");
	//for (int i=0; i<m; ++i)
	//{
		//visualization_msgs::Marker PRMpoint;
		//visualization_msgs::Point PRMpoint;
		//geometry_msgs::Point p, start_, end_;
		//float x = PRM_x_y.col(i)[0];
		//float y = PRM_x_y.col(i)[1];
		//p.x = x;
		//p.y = y;
		//start_.x=x;
		//start_.y=y;
		//if (i!=0)
		//{
		//	end_.x=PRM_x_y.col(i-1)[0];
		//	end_.y=PRM_x_y.col(i-1)[1];
		//}
		//else
		//{
		//	end_.x=PRM_x_y.col(0)[0];
		//	end_.y=PRM_x_y.col(0)[1];
		//}
		//lines.points.push_back(start_);
		//lines.points.push_back(end_);
		//drawLineSegment(i, start_, end_);
		//ROS_INFO("%f",PRM_x_y.col(i)[0]);
		//ROS_INFO("%f",PRM_x_y.col(i)[1]);
		
		//points.points.push_back(p);

	//}
	marker_pub.publish(points);
	

	while (ros::ok())
	{
		loop_rate.sleep(); //Maintain the loop rate
		ros::spinOnce();   //Check for new messages
		//marker_pub.publish(points);
		//marker_pub.publish(lines);


		 //Draw Curves
		 //drawCurve(1);
		 //drawCurve(2);
		 //drawCurve(4);

		 //drawLineSegment(5,start_,end_);

		//velocity_publisher.publish(vel); // Publish the command velocity

		//points.id=0;
		//points.type = visualization_msgs::Marker::POINTS;
		//points.scale.x=0.1;

	}
}
