//
// wheelのv とimuのyawからodomをpublish
//
//
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <Eigen/Core>
#include "lf/node_edge_manager.h"


using namespace std;

class odomSaver
{
	ros::NodeHandle n;
	ros::Subscriber sub_complement;
	ros::Subscriber sub_corrected;
	ros::Publisher pub_complement;
	ros::Publisher pub_corrected;
	double x_init;
	double y_init;
	double yaw_init;
	nav_msgs::Odometry odom_complement;
	geometry_msgs::PoseArray odom_corrected;

	void set_init_pose(XmlRpc::XmlRpcValue&);

	public:
	odomSaver();
	void complementCallback(const nav_msgs::Odometry::ConstPtr&);
	void correctedCallback(const geometry_msgs::PoseArray::ConstPtr&);
	void saveCorrectedOdoms();
	void odomCorrectedPublisher();
};

odomSaver::odomSaver()
	: x_init(0.0), y_init(0.0), yaw_init(0.0)
{
	sub_complement = n.subscribe<nav_msgs::Odometry>("/odom/complement", 1, &odomSaver::complementCallback, this);
	sub_corrected = n.subscribe<geometry_msgs::PoseArray>("/corrected/odom", 1, &odomSaver::correctedCallback, this);
	pub_complement = n.advertise<nav_msgs::Odometry>("/odom/complement/affine", 1);
	pub_corrected = n.advertise<geometry_msgs::PoseArray>("/odom/corrected/all", 1);

	odom_complement.header.frame_id = "map";

	XmlRpc::XmlRpcValue init_node;
	init_node["begin"] = 0;
	init_node["end"] = 1;
	init_node["div"] = 0.0;
	n.getParam("/init/node", init_node);
	set_init_pose(init_node);
}

void odomSaver::complementCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	odom_complement = *msg;

	odom_complement.pose.pose.position.x = 
		msg->pose.pose.position.x*cos(yaw_init) - msg->pose.pose.position.y*sin(yaw_init) + x_init;

	odom_complement.pose.pose.position.y = 
		msg->pose.pose.position.x*sin(yaw_init) + msg->pose.pose.position.y*cos(yaw_init) + y_init;

	double roll, pitch, yaw;
	tf::Quaternion q(msg->pose.pose.orientation.x,
					 msg->pose.pose.orientation.y,
					 msg->pose.pose.orientation.z,
					 msg->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw + yaw_init);

    // tf::Quaternion init_quat;
	// init_quat.setEuler(yaw_init, 0.0, 0.0);
    // tf::Quaternion odom_quat = msg->pose.pose.orientation;

	odom_complement.pose.pose.orientation = odom_quat;

	pub_complement.publish(odom_complement);
}

void odomSaver::correctedCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
	// odom_corrected.poses.insert(odom_corrected.poses.end(), msg->poses.begin(), msg->poses.end());
}

void odomSaver::set_init_pose(XmlRpc::XmlRpcValue &in)
{
	int begin = in["begin"];
	int end = in["end"];
	double div = in["div"];
	string filename = "/home/amsl/ros_catkin_ws/src/mapping/latlng2xy/xy/tsukuba.csv";
	n.getParam("/node_edge", filename);
	NodeEdgeManager nem(filename);

	int edge_num = nem.edgeGetter(begin, end);
	double dist = sqrt(nem.distGetter(edge_num)) * div;

	yaw_init = nem.edgeGetter(edge_num).orientation;
	x_init = nem.nodeGetter(begin).x + dist * cos(yaw_init);
	y_init = nem.nodeGetter(begin).y + dist * sin(yaw_init);
}

void odomSaver::saveCorrectedOdoms()
{
}

void odomSaver::odomCorrectedPublisher()
{
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odom_saver");

	ros::NodeHandle n;
	ros::Publisher pub_finish = n.advertise<std_msgs::Bool>("/flag/finish", 1);

	std_msgs::Bool flag;
	flag.data = true;

	odomSaver os;

	// ros::Rate loop_rate(60);

	while(ros::ok()){
		ros::spin();
		// loop_rate.sleep();
	}
	cout << "aaa" << endl;

	pub_finish.publish(flag);

	return 0;
}

