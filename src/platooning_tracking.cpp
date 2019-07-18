#include "ros/ros.h"
//#include "sensor_msgs/LaserScan.h"
#include <iostream>
#include <cstring>
#include <cmath>

#include <obstacle_detector/Obstacles.h>
#include <geometry_msgs/Point.h>

using namespace std;

class Tracking{
private:
	ros::NodeHandle nh_;

	ros::Subscriber obstacle_sub_;

	int steer;
	int size_of_segments;
	float dist_ob;

	geometry_msgs::Point first_point;
	geometry_msgs::Point last_point;
	geometry_msgs::Point middle_point;
public:
	Tracking();
	~Tracking();
	void obstacle_cb(const obstacle_detector::Obstacles& data);
	void run();
	void initSetup();
};


Tracking::Tracking():nh_("~")
{
	initSetup();
}

Tracking::~Tracking()
{
	initSetup();
}

void Tracking::initSetup()
{
	ROS_INFO("init start");
	obstacle_sub_ = nh_.subscribe("/raw_obstacles", 100, &Tracking::obstacle_cb, this);
}

void Tracking::obstacle_cb(const obstacle_detector::Obstacles& data)
{
	first_point.x = 0.0;
	first_point.y = 0.0;
	last_point.x = 0.0;
	last_point.y = 0.0;
	size_of_segments = data.segments.size();

	for(int i = 0; i < size_of_segments ; i++){
		first_point.x += data.segments[i].first_point.x;
		first_point.y += data.segments[i].first_point.y;
		last_point.x += data.segments[i].last_point.x;
		last_point.y += data.segments[i].last_point.y;

	}
	
	if(size_of_segments != 0){
		first_point.x = first_point.x/size_of_segments;
		first_point.y = first_point.y/size_of_segments;
		last_point.x = last_point.x/size_of_segments;
		last_point.y = last_point.y/size_of_segments;
	}

	middle_point.x = -(first_point.x + last_point.x)/2;
	middle_point.y = -(first_point.y + last_point.y)/2;

	dist_ob = sqrt(pow(middle_point.x, 2) + pow(middle_point.y, 2));
	cout << size_of_segments << " " << dist_ob << endl;
}


void Tracking::run()
{
	ros::Rate r(100);
	ROS_INFO("run!!");
	while(ros::ok()){
		ros::spinOnce();
		r.sleep();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "platooning_tracking");
	cout << "node start" << endl;
	Tracking Tracking;
	Tracking.run();

	return 0;
}
