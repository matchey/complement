//
// wheelのv とimuのyawからodomをpublish
//
//
#ifndef ODOM_PUBLISHER_H
#define ODOM_PUBLISHER_H

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
// #include <std_msgs/Bool.h>
#include "ceres_msgs/AMU_data.h"

namespace complement
{
	template<typename WheelT, typename GyroT>
	class odomPublisher
	{
		public:
		using SyncPolicy = message_filters::sync_policies::ApproximateTime<WheelT, GyroT>;

		odomPublisher();
		void setFrame(const std::string&);
		void setFrameChild(const std::string&);
		void setRate(const double&);

		private:
		void callback(const typename WheelT::ConstPtr&, const typename GyroT::ConstPtr&);
		void wheelCallback(const typename WheelT::ConstPtr&);
		void gyroCallback(const sensor_msgs::Imu::ConstPtr&);
		void gyroCallback(const ceres_msgs::AMU_data::ConstPtr&);
		void complement();
		void publisher();
		void pubTF();
		// void pubIsRun();

		ros::NodeHandle n;
		message_filters::Subscriber<WheelT> sub_wheel;
		message_filters::Subscriber<GyroT> sub_gyro;
		message_filters::Synchronizer<SyncPolicy> sync;
		ros::Publisher pub_odom;
		ros::Publisher pub_flag_run;
		tf::TransformBroadcaster odom_broadcaster;

		std::string topic_wheel;
		std::string topic_gyro;
		std::string topic_pub;
		std::string frame_this;
		std::string frame_child;

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
		// std_msgs::Bool flag_run;
		ros::Time current_time, last_time;
	};
	template class odomPublisher<nav_msgs::Odometry, sensor_msgs::Imu>;
	template class odomPublisher<nav_msgs::Odometry, ceres_msgs::AMU_data>;
} // namespace complement
#endif

