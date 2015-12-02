#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Dense>
#include <string>
#include <sstream>
#include <math.h>
#include <time.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <limits>
#include <random>
#include <fenv.h>
#include "utils.h"

using namespace Eigen;
using namespace visualization_msgs;

// Initialize variables for u and z
const auto m = 700;
const auto k = 3;
double dt = 1;
Matrix<double, k, 2> odom;
Matrix<double, k, 1> z;
Matrix<double, k, m> belief;
Matrix<double, k, 1> belief_mean;
Matrix<double, 3, 3> Q;

Vector3d introduce_particle(Vector3d x_p) {
    // Choice of particle to introduce is fairly easy for IPS sensors; just choose the IPS data as the value
    Vector3d result;    
    result << z[0] + sample_normal(0.01), z[1] + sample_normal(0.01), z[2];
    
    return result;
}

// Given previous belief particle set, control signal, measurement and state transition function, compute new particle set
template<int k, int m>
Matrix<double, k, m> aug_mcl(Matrix<double, k, m> belief, Matrix<double, k, 2> u, double (*meas_model)(Matrix<double, k, 1>), Matrix<double, k, 1> (*g)(Matrix<double, k, 1>, Matrix<double, k, 2>)) {
    // Moving averages
    static double w_slow = 0.0000001;
    static double w_fast = 0;
    
    // Decay parameters
    const double a_slow = 0.01;
    const double a_fast = 0.5;
    
    // Initialize particle set matrices for prediction and final current belief
    auto x_p = Matrix<double, k, m>();
    auto x = Matrix<double, k, m>();
    auto w = std::vector<double>(m);
    double w_avg = 0;

    // Iterate over previous belief particles
    for(int i = 0; i < m; i++) {
        // Use motion model to calculate new prediction        
        Matrix<double, k, 1> x_p_i = g(belief.col(i), u);
        x_p.col(i) = x_p_i;
        
        // Work out weights using measurement model
        w[i] = meas_model(x_p_i);
        
        // Is overall likelihood of our measurements given our current beliefs really bad?
        w_avg += w[i] / m;
    }
    
    // Accumulate weights so it's easy to pick a weighted sample
    std::partial_sum(w.begin(), w.end(), w.begin());
    double total_w = w.back();
    
    // Slow moving average conforms to overall measurement likelihood slowly, fast moving etc.
    w_slow += a_slow * (w_avg - w_slow);
    w_fast += a_fast * (w_avg - w_fast);
    
    // Don't resample if no changes greater than a predefined value
    // bool shouldResample = ((u.col(1) - u.col(0)).array().abs() > 1e-4).any();
    // if (!shouldResample) {
    //     ROS_INFO("Not resampling!");
    //     return x_p;
    // }
    
    // If overall likelihood of measurement being correct has been low for some time, start relocalizing
    double likelihood = std::max(0.0, 1.0 - w_fast / w_slow);
    // ROS_INFO("%f, %f, %f, %f", w_avg, w_fast, w_slow, likelihood);
    
    // Update particle set
    for(int i = 0; i < m; i++) {
        // Re-localize
        if (drand() < likelihood) {
            x.col(i) << introduce_particle(x_p.rowwise().mean());
        // Otherwise, resample from existing particle set
        }  else {
            double rnd = drand() * total_w;
            int j = std::lower_bound(w.begin(), w.end(), rnd) - w.begin(); // Find first element with weight greater than rnd
    
            x.col(i) = x_p.col(j);   
        }
    }
    return x;
}

double ips_measurement_model(Vector3d x_p) {
    return normpdf(z, x_p, Q);
}

// Rot-trans-rot odometry based motion model
Vector3d motion_model(Vector3d x, Matrix<double, 3, 2> u) {
    Matrix<double, 4, 1> noise;
    noise << 0.05, 0.26, 0.01, 0.0057; // rad/rad, rad/m, m/m, m/rad
    // noise << 0.1, 0.1, 0.1, 0.1;
    
    auto u_old = u.col(0);
    auto u_new = u.col(1);
    
    // Start by defining del
    Vector3d del;
    auto drot1 = atan2(u_new[1] - u_old[1], u_new[0] - u_old[0]) - u_old[2];
    del << 
        sqrt(pow(u_new[0] - u_old[0], 2) + pow(u_new[1] - u_old[1], 2)),
        drot1,
        u_new[2] - u_old[2] - drot1;
        
    // Introduce noise
    Vector3d del_hat;
    
    auto d0 = sample_normal(noise[2] * del[0] + noise[3] * (abs(del[1]) + abs(del[2])));
    auto d1 = sample_normal(noise[0] * abs(del[1]) + noise[1] * del[0]);
    auto d2 = sample_normal(noise[0] * abs(del[2]) + noise[1] * del[0]);
    
    del_hat << 
        del[0] - d0,
        del[1] - d1,
        del[2] - d2;
        
    // Evaluate output
    Vector3d x_new;
    x_new << 
        x[0] + del_hat[0] * cos(x[2] + del_hat[1]),
        x[1] + del_hat[0] * sin(x[2] + del_hat[1]),
        x[2] + del_hat[1] + del_hat[2];
    
    return x_new;
}



// For actual IPS messages
void pose_callback_live(const geometry_msgs::PoseWithCovarianceStamped& msg) {
    auto lin = msg.pose.pose.position;
    auto yaw = tf::getYaw(msg.pose.pose.orientation);
    
    z << lin.x, lin.y, yaw;
}

// For gazebo "fake IPS" messages
void pose_callback_sim(const gazebo_msgs::ModelStates msg) {
    int i;    
    for(i = 0; i < msg.name.size(); i++) 
        if (msg.name[i] == "mobile_base") 
            break;
    
    // Variance in x, y measurement updates is 0.01 m^2. Variance in theta is 0.01 rad^2
    double ips_x = msg.pose[i].position.x + sample_normal(sqrt(0.01));
    double ips_y = msg.pose[i].position.y + sample_normal(sqrt(0.01));
    double ips_yaw = tf::getYaw(msg.pose[i].orientation) + sample_normal(sqrt(0.01));
    
    z << ips_x, ips_y, ips_yaw;
}

void odom_callback(const nav_msgs::Odometry msg) {
    odom.col(0) = odom.col(1);
    odom.col(1) << msg.pose.pose.position.x, msg.pose.pose.position.y, tf::getYaw(msg.pose.pose.orientation);
};

template<int k, int m>
Marker build_particleset_viz(Matrix<double, k, m> pset) {
    Marker msg;
    msg.points.clear();
    auto mean = pset.rowwise().mean();
    
    msg.id = 0;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time();
    msg.ns = "localization";
    msg.action = Marker::ADD;
    msg.type = Marker::POINTS;
    msg.scale.x = 0.05;
    msg.scale.y = 0.05;
    msg.scale.z = 0.05;
    msg.color.a = 1.0;
    msg.color.r = 0.0;
    msg.color.g = 1.0;
    msg.color.b = 0.0;
    
    for (int i = 0; i < m; i++) {
        geometry_msgs::Point p;
        auto col = pset.col(i);

        p.x = col[0];
        p.y = col[1];
        p.z = 0;
        
        msg.points.push_back(p);
    }
    
    return msg;
}

// Update TF transform
void publish_world_robot_transform(double x, double y, double omega){
    static tf::TransformBroadcaster br;
    
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, 0.0));
    
    tf::Quaternion q;
    q.setEuler(omega, 0, 0);
    transform.setRotation(q);
    
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
}

int main(int argc, char **argv)
{
    // Initialize the ROS framework
    ros::init(argc, argv, "localization");
    ros::NodeHandle n;
    
    // Initalize parameters
    Q << 0.01, 0,   0,
         0,    0.01, 0,
         0,    0,    0.01;
    
    // Initialize our belief (no clue where we are, just guess wildly)
    for (int i = 0; i < m; i++) {
        // Going for a 10m x 10m world
        belief.col(i) << (drand() - 0.5) * 2 * 10, (drand() - 0.5) * 2 * 10, (drand() - 0.5) * 2 * M_PI;
    }
    
    // Publisher for belief particle set
    auto particles_pub = n.advertise<Marker>("belief_set", 1, true);    
    
    // Subscribers for odometry
    auto odom_sub = n.subscribe("/odom", 1, odom_callback);
	
	// Subscribe handlers for whichever of the two IPS methods is active
    auto pose_sub_live = n.subscribe("/indoor_pos", 1, pose_callback_live);
    auto pose_sub_sim = n.subscribe("/gazebo/model_states", 1, pose_callback_sim);

    // Set the loop rate
    ros::Rate loop_rate(1 / dt);
    
    while (ros::ok())
    {        
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages

    	//Main loop code goes here
        belief = aug_mcl(belief, odom, ips_measurement_model, motion_model);
        belief_mean = belief.rowwise().mean();
        
        // Publish information
        particles_pub.publish(build_particleset_viz(belief));
        publish_world_robot_transform(belief_mean[0], belief_mean[1], belief_mean[2]);        
        
        // Output information
        Matrix<double, 3, 4> output;
        output << belief_mean, z, odom;
        std::cout << output << std::endl << std::endl;
    }

    return 0;
}
