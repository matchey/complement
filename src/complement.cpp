//
// wheelのv とimuのyawからodomをpublish
//
//
#include <ros/ros.h>
#include "complement/odom_publisher.h"

using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "complement_pub_tf");

	string topic_gyro;
	bool flag_tf;

	ros::param::param<string>("~topic_name/gyro", topic_gyro, "/imu/data");
	ros::param::param<bool>("~flag/pubtf", flag_tf, true);

	if(topic_gyro =="/AMU_data"){

		complement::odomPublisher<nav_msgs::Odometry, ceres_msgs::AMU_data> op;
		op.setPubTF(flag_tf);

		if(argc == 2){
			op.setRate(atof(argv[1])); // bag rate (e.g. rosbag play *.bag -r 0.5 )
		}

		ros::spin();

	}else{

		complement::odomPublisher<nav_msgs::Odometry, sensor_msgs::Imu> op;
		op.setPubTF(flag_tf);

		if(argc == 2){
			op.setRate(atof(argv[1])); // bag rate (e.g. rosbag play *.bag -r 0.5 )
		}

		ros::spin();
	}


	return 0;
}

