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

	odomPublisher op;

	if(argc == 2){
		op.setRate(atof(argv[1])); // bag rate (e.g. rosbag play *.bag -r 0.5 )
	}

	ros::Rate loop_rate(100);

	while(ros::ok()){
		op.complement();
		op.publisher();
		op.pubIsRun();
		op.pubTF("/matching_base_link");
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

