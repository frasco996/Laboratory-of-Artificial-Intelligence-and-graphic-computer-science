#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>

ros::Publisher v;
bool res = false;
geometry_msgs::Twist receivedV;

void laser_scan_sub_callback(const sensor_msgs::LaserScan::ConstPtr& msg){

  if (!res) return;
  res = false;

  tf::TransformListener list;
  laser_geometry::LaserProjection proj;
  sensor_msgs::PointCloud cloud;
  proj.transformLaserScanToPointCloud("Projector laser",*msg,cloud,list);

}

void vel_cmd_sub_callback(const geometry_msgs::Twist::ConstPtr& msg){ 
	res = true;
	receivedV = *msg;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "robot_avoidance");
  ros::NodeHandle p;
  ros::Subscriber vel_cmd_sub = p.subscribe("vel_cmd", 5, vel_cmd_sub_callback);
  ros::Subscriber laser_scan_sub = p.subscribe("laser_scan", 500, laser_scan_sub_callback); 
  ros::spin();
  return 0;

}
