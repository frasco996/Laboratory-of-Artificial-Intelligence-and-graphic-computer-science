#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_spec.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>

ros::Publisher speed;
bool res = false;
geometry_msgs::Twist receivedV;
const int N=200;

void laser_scan_sub_callback(const sensor_msgs::LaserScan::ConstPtr& msg){

	if (!res) return;
	res = false;

	tf::TransformListener list;
	laser_geometry::LaserProjection proj;
	sensor_msgs::PointCloud cloud;
	proj.transformLaserScanToPointCloud("base_laser_link",*msg,cloud,list);

	tf::StampedTransform ob;

	try{
		list.waitForTransform("base_footprint", "base_laser_link", ros::Time(0), ros::Duration(1.0));
		list.lookupTransform("base_footprint", "base_laser_link", ros::Time(0), ob); 

	}
	catch(tf::TransformException &ex){
		ROS_ERROR("%s", ex.what());
		return;
	    }

	Eigen::Isometry2f laserTrans = convertPose2D(ob);
	Eigen::Vector2f p_min; 

	p_min(0) = cloud.points[540].x; 
	p_min(1) = cloud.points[540].y;
	auto ob_pos = laserTrans * p_min;
	float ob_dist = sqrt(std::pow(ob_pos(0),2)+std::pow(ob_pos(1),2));

	const float min = cloud.points.size()/2-N;
	const float max = cloud.points.size()/2+N;

	for(int i=min;i<=max;i++){
	  Eigen::Vector2f v;

	  v(0) = cloud.points[i].x; 
	  v(1) = cloud.points[i].y;  

	  auto ob_posA = laserTrans * v;
	  float ob_distA = sqrt(std::pow(ob_posA(0),2)+std::pow(ob_posA(1),2));

	  if(ob_distA < ob_dist){
	    ob_dist = ob_distA;
	    ob_pos = ob_posA;
	  }
	}
	if(ob_dist < 0.5 && receivedV.linear.x > 0 ){
	  float force_int = (1.0 / ob_dist) * 0.3 ;
	  float forceX = -(ob_pos(0) / ob_dist) * force_int;
	  float forceY = -(ob_pos(1) / ob_dist) * force_int;

	  geometry_msgs::Twist msg_send;

	  msg_send.linear.x = receivedV.linear.x + forceX;
	  msg_send.linear.y = receivedV.linear.y + forceY;
	  msg_send.linear.z = receivedV.linear.z;

	  if(ob_pos(1) > 0)
	    msg_send.angular.z = - force_int; 
	  else if (ob_pos(1) < 0)
	    msg_send.angular.z = force_int; 
	  ROS_INFO("msg_send.linear.x %f",msg_send.linear.x);

	  speed.publish(msg_send);
	}
	else{
  		speed.publish(receivedV);
	}
}

void vel_cmd_sub_callback(const geometry_msgs::Twist::ConstPtr& msg){ 
	res = true;
	receivedV = *msg;
}

int main(int argc, char **argv){

	ros::init(argc, argv, "robot_avoidance");
	ros::NodeHandle p;
	ros::Subscriber vel_cmd_sub = p.subscribe("cmd_vel_sub", 1, vel_cmd_sub_callback);
	ros::Subscriber laser_scan_sub = p.subscribe("base_scan", 1000, laser_scan_sub_callback); 
	speed = p.advertise<geometry_msgs::Twist>("cmd_vel",1000); 
	ros::spin();
	return 0;

}
