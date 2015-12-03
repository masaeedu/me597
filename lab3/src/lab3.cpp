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
ros::Publisher marker_RobotPoints;
Eigen::Vector3d IPS;
const int m = 100;
const int k = 10;
nav_msgs::OccupancyGrid og;

// controller global variables
vector<double> previousErrorYaw;

// Callback function for the Position topic (LIVE)
void pose_callback_live(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    // This function is called when a new position message is received
    double X = msg.pose.pose.position.x; // Robot X psotition
    double Y = msg.pose.pose.position.y; // Robot Y psotition
    double Yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
    
    IPS << X * 2.2, Y * 2.2, Yaw;
}

// Callback function for the Position topic (SIMULATION)
void pose_callback_sim(const gazebo_msgs::ModelStates& msg)
{
    int i;
    for (i = 0; i < msg.name.size(); i++) {
        if (msg.name[i] == "mobile_base")
            break;
    }
    IPS << msg.pose[i].position.x, msg.pose[i].position.y,
        tf::getYaw(msg.pose[i].orientation);
}

double angleError(double globalYaw, double targetYaw)
{
    double result = targetYaw - globalYaw;
    return fmod(result + M_PI, 2 * M_PI) - M_PI;
}

// yaw PID controller
double yawController(double currentYaw, double targetX, double targetY,
    double ipsX, double ipsY)
{
    double kp = 0.5;
    double ki = 0;
    double kd = 0;

    // opposite over adjacent distance is angle to destination
    double targetYaw = atan2((targetY - ipsY), (targetX - ipsX));

    // error feedback for yaw
    double errorInYaw = angleError(currentYaw, targetYaw);

    cout << "errorYaw: " << errorInYaw << endl;

    // change in error for derivative
    // double changeInErrorYaw = errorInYaw - previousErrorYaw.back();
    double changeInErrorYaw = 0;
    // record new error
    previousErrorYaw.push_back(errorInYaw);

    // total error over time for integral
    double totalErrorYaw = 0;

    for (int i = 0; i < previousErrorYaw.size(); i++) {
        totalErrorYaw += previousErrorYaw[i];
    }

    // new output yaw
    double outputYaw = kp * errorInYaw + ki * totalErrorYaw + kd * changeInErrorYaw;
    return outputYaw;
}


// Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid msg)
{
    // This function is called when a new map is received
    og = msg;
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
	points.scale.x = 0.05;
	points.scale.y = 0.05;
	points.color.b = 1.0f;
	points.color.a = 1.0;
	points.points.push_back(p); 
	usleep(2000);
	//std::cin.get();
	marker_pub.publish(points);
}

void drawWaypoint(int k, geometry_msgs::Point p) 
{
	visualization_msgs::Marker points; //Define points
	points.header.frame_id = "/map";
	points.header.stamp = ros::Time::now();
	points.ns = "lab3";
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = 1.0;
	points.id = k;
	points.type = visualization_msgs::Marker::POINTS;
	points.scale.x = .2;
	points.scale.y = .2;
	points.color.b = 0.0f;
	points.color.g = 1.0f;
	points.color.a = 1.0;
	points.points.push_back(p); 
	usleep(2000);
	//std::cin.get();
	marker_pub.publish(points);
}

void drawRobotPoint(int k, geometry_msgs::Point p) 
{
	visualization_msgs::Marker points; //Define points
	points.header.frame_id = "/map";
	points.header.stamp = ros::Time::now();
	points.ns = "lab3";
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = 1.0;
	points.id = k;
	points.type = visualization_msgs::Marker::POINTS;
	points.scale.x = .05;
	points.scale.y = .05;
	points.color.b = 0.39f;
	points.color.g = 0.90f;
	points.color.r = 0.87f;
	points.color.a = 1.0;
	points.points.push_back(p); 
	usleep(2000);
	//std::cin.get();
	marker_RobotPoints.publish(points);
}

void drawLineSegment(int k, geometry_msgs::Point start_point, geometry_msgs::Point end_point)
{
	visualization_msgs::Marker lines;
	lines.header.frame_id = "/map";
	lines.type = visualization_msgs::Marker::LINE_STRIP;
	lines.ns = "lab3";
	
	
	lines.id = k; //each curve must have a unique id or you will overwrite an old ones
	lines.type = visualization_msgs::Marker::LINE_STRIP;
	lines.action = visualization_msgs::Marker::ADD;
	
	lines.scale.x = 0.02;
	lines.scale.y = 0.02;
	lines.color.r = 1.0;
	lines.color.b = 0.2;
	lines.color.a = 0.5;

	lines.points.push_back(start_point);
	lines.points.push_back(end_point);

	//publish new segment line
	usleep(2000);
	marker_pub.publish(lines);
	//sleep(0.05);
}

void drawWHEYPROTEIN(int k, geometry_msgs::Point start_point, geometry_msgs::Point end_point)
{
	visualization_msgs::Marker lines;
	lines.header.frame_id = "/map";
	lines.type = visualization_msgs::Marker::LINE_STRIP;
	lines.ns = "lab3";
	
	
	lines.id = k; //each curve must have a unique id or you will overwrite an old ones
	lines.type = visualization_msgs::Marker::LINE_STRIP;
	lines.action = visualization_msgs::Marker::ADD;
	
	lines.scale.x = 0.05;
	lines.scale.y = 0.05;
	lines.color.r = 1.0;
	lines.color.b = 1.0;
	lines.color.a = 1.0;

	lines.points.push_back(start_point);
	lines.points.push_back(end_point);

	//publish new segment line
	usleep(2000);
	marker_pub.publish(lines);
	//sleep(0.05);
}

void delLine(int k)
{
	visualization_msgs::Marker lines;
	lines.header.frame_id = "/map";
	lines.id = k;
	lines.ns = "lab3";
	lines.action = visualization_msgs::Marker::DELETE;

	//publish new segment line
	usleep(2000);
	marker_pub.publish(lines);
	//marker_RobotPoints.publish(lines);
}

void drawPrm(tuple<vector<coord>, map<int, map<int, double>>> prmResult) {
	// Plot the points
	int pointId = 0;
	vector<coord> coords = std::get<0>(prmResult);
	int num_nodes=coords.size();
	std::cout<<num_nodes<<std::endl;

	for (int i=0; i<m*(k+1); i++)
	{
		usleep(2000);
		delLine(i);
	}

	for (int i=0; i<num_nodes; ++i)
	{
	        coord item = coords[i]; 
	        double x = std::get<0>(item); 
	        double y = std::get<1>(item);
	        std::cout<<"X-val"<<x<<" Y-val"<<y<<std::endl;
	        geometry_msgs::Point p;
	        p.x = x;
	        p.y = y;
		
		usleep(2000);
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
		for(auto kv2: connections) 
		{
			geometry_msgs::Point start_, end_;
	       		int idx2 = kv2.first;
        		coord c2 = coords[idx2];
            
        		// Get start and end coordinates
        		start_.x = std::get<0>(c1);
        		start_.y = std::get<1>(c1);    
        		end_.x = std::get<0>(c2);
        		end_.y = std::get<1>(c2);
            
        		// Draw line between c1 and c2
			usleep(800);
        		drawLineSegment(pointId++, start_, end_);
		
		}
	}
}


void drawChosenPath(vector<int> path_nodes,tuple<vector<coord>, map<int, map<int, double>>> prmResult) {
	// Plot the points
	int pointId = 0;
	int num_nodes = path_nodes.size();
	double x = IPS[0];
	double y = IPS[1];

	geometry_msgs::Point p;
	p.x = 1.45;
	p.y = -2.23+1.5;
	p.z = 0.1;
		
	usleep(2000);
	drawWaypoint(pointId++, p);
	p.x = 0.08;
	p.y = 0.6+1.5;
	p.z =0.1;
		
	usleep(2000);
	drawWaypoint(pointId++, p);
	p.x = -1.72;
	p.y = 0.43+1.5;
	p.z = 0.1;
		
	usleep(2000);
	drawWaypoint(pointId++, p);
	//std::cin.get();

	vector<coord> coords = std::get<0>(prmResult);
	for (int i=0; i<num_nodes; i++)
	{
		geometry_msgs::Point start_, end_;
		coord item = coords[path_nodes[i]];
		cout << "This is pathnodes " << path_nodes[i] << endl;
		
		cout << "pathnode x: " << std::get<0>(item);
	 	cout << " pathnode y: " << std::get<1>(item) << endl;
		

		start_.x = x;
       		start_.y = y;
		start_.z = 0.2;    
       		end_.x = std::get<0>(item);
       		end_.y = std::get<1>(item);
		end_.z = 0.2;
		usleep(2000);
		drawWHEYPROTEIN(pointId++, start_, end_);
		
		x=std::get<0>(item);
		y=std::get<1>(item);
		cout << x << endl;
		cout << y << endl;
	}


}


int main(int argc, char** argv)
{
    srand((unsigned)time(0));
    ros::init(argc, argv, "main_control");
    ros::NodeHandle n;

    auto map_sub = n.subscribe("/map", 1, map_callback);
    auto pose_sub_live = n.subscribe("/indoor_pos", 1, pose_callback_live); // switch for live tests
    auto pose_sub_sim = n.subscribe("/gazebo/model_states", 1, pose_callback_sim);
    auto velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 20);
	marker_RobotPoints = n.advertise<visualization_msgs::Marker>("Robot_marker", 20);
    // Velocity control variable
    geometry_msgs::Twist vel;

    // Set the loop rate
    ros::Rate loop_rate(20); // 20Hz update rate

    // Get the occupancy grid
    auto grid = *ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(
        "/map", n, ros::Duration(1.0));
        
    // Initialize our IPS
    auto val = *ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>(
        "/indoor_pos", n, ros::Duration(1.0));
    pose_callback_live(val);

    // Waypoints we need to hit
    //auto waypoints = vector<coord>{ coord{ 4.0, 0.0 }, coord{ 8.0, -4.0 }, coord{ 8.0, 0.0 } };
    double gridx = grid.info.origin.position.x;
    double gridy = grid.info.origin.position.y;
    auto waypoints = vector<coord>{coord{1.45, -2.23 + 1.5}, coord{0.08, 0.6 + 1.5}, coord{-1.72, 0.43 + 1.5}};
    // Get the probabilistic roadmap
    bool success = true;
    int attempt = 0;
	int RobotPosCount =0;
    tuple<vector<coord>, map<int, map<int, double> > > result;
    vector<int> path_nodes;

    // Preparing PRM
    ROS_INFO("Preparing PRM");
    do {
        attempt++;
        success = true;
        path_nodes = {};
        result = prm(m, k, coord{ IPS[0], IPS[1] }, waypoints, grid);
        drawPrm(result);

        for (auto wp : vector<int>{ 1, 2, 3 }) {
            auto path_seg = a_star(wp - 1, wp, std::get<0>(result), std::get<1>(result));
            success = success && (path_seg.size() > 0);
            for (auto i = 1; i < path_seg.size(); i++) {
                path_nodes.push_back(path_seg[i]);
            }
        }
    } while (!success);
    
    drawChosenPath(path_nodes,result);
	
    
    // Preparing for tracking
    ROS_INFO("Preparing tracking");
    vector<coord> coords = std::get<0>(result);
    bool outsideXThreshold, outsideYThreshold;
    int i = 0;
    outsideXThreshold = outsideYThreshold = true;

    for (auto i: path_nodes) {
        std::cout << i << "; ";
        std::cout << std::endl;
    }
    
    std::cin.get();

    // bool outsideYawThreshold = true;
    ROS_INFO("Starting trajectory");
    while (i < path_nodes.size() && ros::ok()) {

        loop_rate.sleep(); // Maintain the loop rate
        ros::spinOnce(); // Check for new messages

        coord target = coords[path_nodes[i]];

        cout << "Target is: " << std::get<0>(target) << ", " << std::get<1>(target);

        // check if within error bound of x waypoint
        outsideXThreshold = abs(std::get<0>(target) - IPS[0]) > 0.15;

        // check if within error bounds of y waypoint
        outsideYThreshold = abs(std::get<1>(target) - IPS[1]) > 0.15;

        if (outsideXThreshold || outsideYThreshold) {
            // update angular velocity using PID controller
            vel.angular.z = -yawController(IPS[2], std::get<0>(target),
                std::get<1>(target), IPS[0], IPS[1]);
            
            // if not, move towards destination
            vel.linear.x = min(0.1, abs(0.1 / vel.angular.z));
        }
        else {
            // if within error bounds, stop moving
            vel.linear.x = 0;
            vel.angular.z = 0;
            i++;
        }

        cout << "ipsX: " << IPS[0] << " ipsY: " << IPS[1] << " ipsYaw: " << IPS[2]
                << endl;
        cout << "Velocity is; " << vel.linear.x << " m/s, " << vel.angular.z << " rad/s" << endl;
        cout << "Outside X: " << outsideXThreshold
                << " Outside Y: " << outsideYThreshold << endl;
        velocity_publisher.publish(vel);

	geometry_msgs::Point p;
	p.x = IPS[0];
	p.y = IPS[1];
	p.z = 0.4;
	drawRobotPoint(	RobotPosCount++, p);
    }
}
