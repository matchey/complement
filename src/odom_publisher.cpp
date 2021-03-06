//
// wheelのv とimuのyawからodomをpublish
//
//
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include "complement/odom_publisher.h"

using std::cout;
using std::endl;
using std::string;

namespace complement
{
	template<typename WheelT, typename GyroT>
	odomPublisher<WheelT, GyroT>::odomPublisher()
		: sync(SyncPolicy(150), sub_wheel, sub_gyro), n("~"),
		  flag_tf(true), x(0.0), y(0.0), pitch(0.0), yaw(0.0),// odom_quat(0, 0, 0, 1),
		  vel(0.0), dyaw(0.0), rate(1.0)
	{
		n.param<string>("topic_name/wheel", topic_wheel, "/tinypower/odom");
		n.param<string>("topic_name/gyro", topic_gyro, "/AMU_data");
		n.param<string>("topic_name/odom_complement", topic_pub, "/odom/complement");

		n.param<string>("frame_name/this", frame_this, "/map");
		n.param<string>("frame_name/child", frame_child, "/matching_base_link");

		sub_wheel.subscribe(n, topic_wheel, 10);
		sub_gyro.subscribe(n, topic_gyro, 150);
		// sync.connectInput(sub_wheel, sub_gyro);

		sync.registerCallback(boost::bind(&odomPublisher::callback, this, _1, _2));
		// sync.setMaxIntervalDuration(ros::Duration(0.1));

		pub_odom = n.advertise<nav_msgs::Odometry>(topic_pub, 1);
		// pub_flag_run = n.advertise<std_msgs::Bool>("/flag/is_run", 1);

		n.getParam("/pitch/init", pitch_init);
		cout << "init_pitch: " << pitch_init << endl;

		if(!n.getParam("dyaw/drift", drift_dyaw)){ drift_dyaw = 0.287207006; }
		cout << "drift_dyaw: " << drift_dyaw << endl;

		odom_quat = tf::createQuaternionMsgFromYaw(yaw);
		// flag_run.data = false;

		current_time = ros::Time(0.0);
		last_time= current_time;
	}

	template<typename WheelT, typename GyroT>
	void odomPublisher<WheelT, GyroT>::setPubTF(const bool flag)
	{
		flag_tf = flag;
	}

	template<typename WheelT, typename GyroT>
	void odomPublisher<WheelT, GyroT>::setRate(const double& bag_rate)
	{
		rate = bag_rate;
	}

	//////// private ////////
	template<typename WheelT, typename GyroT>
	void odomPublisher<WheelT, GyroT>::callback
	(const typename WheelT::ConstPtr& wheel, const typename GyroT::ConstPtr& gyro)
	{
		current_time = wheel->header.stamp + (gyro->header.stamp - wheel->header.stamp) * 0.5;
		wheelCallback(wheel);
		gyroCallback(gyro);
		complement();
		if(flag_tf) pubTF();
		publisher();
	}

	template<typename WheelT, typename GyroT>
	void odomPublisher<WheelT, GyroT>::wheelCallback(const typename WheelT::ConstPtr& msg)
	{
		vel = msg->twist.twist.linear.x;
		vel = vel > 1.5 ? 1.5 : vel;
		// flag_run.data = fabs(vel) < 0.1 ? false : true;

		vel *= rate;
	}

	template<typename WheelT, typename GyroT>
	void odomPublisher<WheelT, GyroT>::gyroCallback(const sensor_msgs::Imu::ConstPtr& msg)
	{
		dyaw = msg->angular_velocity.z - drift_dyaw;

		dyaw *= rate;
	}

	template<typename WheelT, typename GyroT>
	void odomPublisher<WheelT, GyroT>::gyroCallback(const ceres_msgs::AMU_data::ConstPtr& msg)
	{
		dyaw = -(msg->dyaw - drift_dyaw) * M_PI / 180;

		dyaw *= rate;
	}

	template<typename WheelT, typename GyroT>
	void odomPublisher<WheelT, GyroT>::complement()
	{
		// current_time = ros::Time::now();
		double dt = (current_time - last_time).toSec();
		last_time = current_time;

		if(0.1 < dt){ // for the first callback
			return;
		}

		double dist = vel * dt;

		yaw += dyaw * dt;

		// while(yaw > M_PI) yaw -= 2*M_PI;
		// while(yaw < -M_PI) yaw += 2*M_PI;
		x += dist * cos(yaw);// * cos(pitch);
		y += dist * sin(yaw);// * cos(pitch);
		odom_quat = tf::createQuaternionMsgFromYaw(yaw);
	}

	template<typename WheelT, typename GyroT>
	void odomPublisher<WheelT, GyroT>::publisher() const
	{
		nav_msgs::Odometry odom;
		odom.header.frame_id = frame_this;
		odom.header.stamp = current_time;

		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		pub_odom.publish(odom);
	}

	// template<typename WheelT, typename GyroT>
	// void odomPublisher<WheelT, GyroT>::pubIsRun()
	// {
	// 	pub_flag_run.publish(flag_run);
	// }

	template<typename WheelT, typename GyroT>
	void odomPublisher<WheelT, GyroT>::pubTF()
	{
		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = frame_this;
		odom_trans.child_frame_id = frame_child;

		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		//send the transform
		odom_broadcaster.sendTransform(odom_trans); // not const
	}
} // namespace complement

