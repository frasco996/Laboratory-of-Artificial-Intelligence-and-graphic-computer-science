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

ros::Publisher v;
bool res = false;
geometry_msgs::Twist receivedV;

void laser_scan_sub_callback(const sensor_msgs::LaserScan::ConstPtr& msg){

	if (!res) return;
	res = false;

	tf::TransformListener list;
	laser_geometry::LaserProjection proj;
	sensor_msgs::PointCloud cloud;
	proj.transformLaserScanToPointCloud("Projector_laser",*msg,cloud,list);

	tf::StampedTransform ob;

	try{
		listener.waitForTransform("pos_base", "Projector_laser", ros::Time(0), ros::Duration(1.0));
		listener.lookupTransform("pos_base", "Projector_laser", ros::Time(0), ob); 

	}
	catch(tf::TransformException &ex){
		ROS_ERROR("%s", ex.what());
		return;
	    }

	Eigen::Isometry2f laserTrans = convertPose2D(ob);
	Eigen::Vector2f p_min; 

	p_min(0) = cloud.points[700].x; 
	p_min(1) = cloud.points[700].y;
	auto ob_pos = laserTrans * p_min;
	float ob_dist = sqrt(std::pow(ob_pos(0),2)+std::pow(ob_pos(1),2));

	const float min = cloud.points.size()/2-200;
	const float max = cloud.points.size()/2+200;

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
	v = p.advertise<geometry_msgs::Twist>("vel",1000); 
	ros::spin();
	return 0;

}
