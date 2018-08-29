//
// wheelのv とimuのyawからodomをpublish
//
//
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
// #include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
// #include <time_util/stopwatch.h>
// #include <Eigen/Core>
#include "complement/odom_publisher.h"


using namespace std;

odomPublisher::odomPublisher()
	: n("~"), x(0.0), y(0.0), pitch(0.0), yaw(0.0), vel(0.0), dyaw(0.0), rate(1.0)
{
	n.param<string>("topic_name/wheel", topic_wheel, "/tinypower/odom");
	n.param<string>("topic_name/gyro", topic_gyro, "/AMU_data");
	n.param<string>("topic_name/odom_complement", topic_pub, "/odom/complement");

	sub_wheel = n.subscribe<nav_msgs::Odometry>(topic_wheel, 1, &odomPublisher::wheelCallback, this);
	if(topic_gyro == "/AMU_data"){
		sub_gyro = n.subscribe<ceres_msgs::AMU_data>
			                  (topic_gyro, 1, &odomPublisher::gyroCallback, this);
	}else{
		sub_gyro = n.subscribe<sensor_msgs::Imu>
			                  (topic_gyro, 1, &odomPublisher::gyroCallback, this);
	}

	pub_odom = n.advertise<nav_msgs::Odometry>(topic_pub, 1);
	pub_flag_run = n.advertise<std_msgs::Bool>("/flag/is_run", 1);

	n.getParam("/pitch/init", pitch_init);
	cout << "init_pitch: " << pitch_init << endl;

	if(!n.getParam("dyaw/drift", drift_dyaw)){ drift_dyaw = 0.287207006; }
	cout << "drift_dyaw: " << drift_dyaw << endl;

	flag_run.data = false;

	current_time = ros::Time::now();
	last_time= ros::Time::now();
}

void odomPublisher::wheelCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	vel = msg->twist.twist.linear.x;
	vel = vel > 1.5 ? 1.5 : vel;
	flag_run.data = fabs(vel) < 0.1 ? false : true;

	vel *= rate;
}

void odomPublisher::gyroCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	dyaw = msg->angular_velocity.z - drift_dyaw;
	
	dyaw *= rate;
}

void odomPublisher::gyroCallback(const ceres_msgs::AMU_data::ConstPtr& msg)
{
	// dyaw = -(msg->dyaw - 0.295030001) * M_PI / 180; //left
	// dyaw = -(msg->dyaw - 0.275030001) * M_PI / 180; //org
	// dyaw = -(msg->dyaw - 0.259380006) * M_PI / 180; //right
	dyaw = -(msg->dyaw - drift_dyaw) * M_PI / 180;
	// dyaw = msg->dyaw * M_PI / 180;
	// yaw = msg->yaw * M_PI / 180;
	
	// dpitch = msg->pitch - pitch_init;
	// pitch = msg->pitch;
	
	dyaw *= rate;
}

void odomPublisher::complement()
{
	current_time = ros::Time::now();
	double dt = (current_time - last_time).toSec();
	last_time = ros::Time::now();

	double dist = vel * dt;

	yaw += dyaw * dt;

	while(yaw > M_PI) yaw -= 2*M_PI;
	while(yaw < -M_PI) yaw += 2*M_PI;
	x += dist * cos(yaw);// * cos(pitch);
	y += dist * sin(yaw);// * cos(pitch);
	odom_quat = tf::createQuaternionMsgFromYaw(yaw);
}

void odomPublisher::publisher()
{
	nav_msgs::Odometry odom;
	odom.header.frame_id = "/map";
	odom.header.stamp = ros::Time::now();

	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	pub_odom.publish(odom);
}

void odomPublisher::pubIsRun()
{
	pub_flag_run.publish(flag_run);
}

void odomPublisher::pubTF(const string& child_frame)
{
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "/map";
    odom_trans.child_frame_id = child_frame;

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);
}

void odomPublisher::setRate(const double& bag_rate)
{
	rate = bag_rate;
}

