//
// wheelのv とimuのyawからodomをpublish
//
//
#ifndef ODOM_PUBLISHER_H
#define ODOM_PUBLISHER_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
// #include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
// #include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
// #include <time_util/stopwatch.h>
// #include <Eigen/Core>
#include "ceres_msgs/AMU_data.h"


using namespace std;

class odomPublisher
{
	ros::NodeHandle n;
	ros::Subscriber sub_wheel;
	ros::Subscriber sub_gyro;
	ros::Publisher pub_odom;
	ros::Publisher pub_flag_run;
	tf::TransformBroadcaster odom_broadcaster;
	string topic_wheel;
	string topic_gyro;
	string topic_pub;
	double x;
	double y;
	double pitch;
	double yaw;
	geometry_msgs::Quaternion odom_quat;
	double vel;
	double dyaw;
	double pitch_init;
	double drift_dyaw;
	double rate;
	std_msgs::Bool flag_run;
	ros::Time current_time, last_time;

	public:
	odomPublisher();
	void wheelCallback(const nav_msgs::Odometry::ConstPtr&);
	void gyroCallback(const ceres_msgs::AMU_data::ConstPtr&);
	void complement();
	void publisher();
	void pubIsRun();
	void pubTF(const string&);
	void setRate(const double&);
};

#endif

