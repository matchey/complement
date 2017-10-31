//
// wheelのv とimuのyawからodomをpublish
//
//
#include <ros/ros.h>
#include "complement/odom_publisher.h"


using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "complement");

	odomPublisher op;

	ros::Rate loop_rate(60);

	while(ros::ok()){
		op.complement();
		op.publisher();
		op.pubIsRun();
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

