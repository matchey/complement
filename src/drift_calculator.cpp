//
// amui dyawのdrift誤差を計算
//
//
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include "complement/drift_calculator.h"
#include "mstring/mstring.h"

using std::string;

driftCalculator::driftCalculator()
	: n("~"), vel(0.0), pitch(0.0), dyaw(0.0), pitch_ave(0.0), dyaw_ave(0.0), cnt_dyaw(0)
{
	string topic_wheel;
	string topic_gyro;
	string topic_pub;

	n.param<string>("topic_name/wheel", topic_wheel, "/tinypower/odom");
	n.param<string>("topic_name/gyro", topic_gyro, "/AMU_data");
	n.param<string>("topic_name/drift", topic_pub, "/AMU/drift_error/dyaw");

	if(topic_gyro == "/AMU_data"){
		sub_gyro = n.subscribe<ceres_msgs::AMU_data>
			                  (topic_gyro, 1, &driftCalculator::gyroCallback, this);
	}else{
		sub_gyro = n.subscribe<sensor_msgs::Imu>
			                  (topic_gyro, 1, &driftCalculator::gyroCallback, this);
	}
	sub_wheel = n.subscribe<nav_msgs::Odometry>
		                   (topic_wheel, 1, &driftCalculator::wheelCallback, this);
	pub_drift = n.advertise<std_msgs::Float64>(topic_pub, 1);
}

void driftCalculator::wheelCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	vel = msg->twist.twist.linear.x;
}

void driftCalculator::gyroCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	pitch = msg->angular_velocity.x;
	dyaw = msg->angular_velocity.z;
	calc_dyaw_ave();
}

void driftCalculator::gyroCallback(const ceres_msgs::AMU_data::ConstPtr& msg)
{
	// dyaw = msg->dyaw * M_PI / 180;
	pitch = msg->pitch;
	dyaw = msg->dyaw;
	calc_dyaw_ave();
}

void driftCalculator::calc_dyaw_ave()
{
	if(fabs(vel) < 1e-8 && fabs(dyaw) < 0.5){
		pitch_ave = (pitch_ave * cnt_dyaw + pitch) / (cnt_dyaw + 1.0);
		dyaw_ave  = ( dyaw_ave * cnt_dyaw + dyaw ) / (cnt_dyaw + 1.0);
		cnt_dyaw++;
	}else{
		pitch_ave = 0.0;
		dyaw_ave = 0.0;
		cnt_dyaw = 0;
	}
}

double driftCalculator::pitchInitGetter(){ return pitch_ave; }

double driftCalculator::driftErrorGetter(){ return dyaw_ave; }

double driftCalculator::countGetter(){ return cnt_dyaw; }

void driftCalculator::yamlWriter(const std::string topic_name, const std::string filename)
{
	std::map<std::string, double> param;
	std::vector<std::string> v;
	std::ofstream ofs(filename);

	param["dyaw"] = dyaw_ave;
	param["pitch"] = pitch_ave;

	v=split(topic_name,'/');

	for(auto it = v.begin(); it != v.end(); ++it){
		if(!it->empty()){
			if(*it != v.back()){
				ofs << *it << ":\n    ";
			}else{
				ofs << *it << ": ";
			}
		}
	}
	ofs << param[v[1]] << std::endl;

	ofs.close();
}

void driftCalculator::publisher()
{
	std_msgs::Float64 drift_error;

	drift_error.data = dyaw_ave;
	pub_drift.publish(drift_error);
}

