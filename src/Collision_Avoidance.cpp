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
ros::Publisher vel_cmd_sub;
geometry_msgs::Twist msg_send;
const int N=40;
float goal_x=0;
float goal_y=0;
float force_int=0;
float distanza_robot_goal=0;
bool goal_arrived=false;

void laser_scan_sub_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
	if(goal_arrived==false) return;
	//definisco variabili
	sensor_msgs::PointCloud cloud;
	laser_geometry::LaserProjection proj;
	tf::TransformListener list;
	tf::TransformListener list_goal;
	tf::StampedTransform ob;
 	tf::StampedTransform transform_goal;
	//i punti degli ostacoli rispetto al laser
	proj.transformLaserScanToPointCloud("base_laser_link",*msg,cloud,list);

	try{
		//calcolo la trasformata per ottenere la posizione degli ostacoli
		list.waitForTransform("base_footprint", "base_laser_link", ros::Time(0), ros::Duration(1.0));
		list.lookupTransform("base_footprint", "base_laser_link", ros::Time(0), ob); 
	}
	catch(tf::TransformException &ex){
		ROS_ERROR("%s", ex.what());
		return;
	    }

	//richiamo la funzione convertPose2D
	Eigen::Isometry2f laserTrans = convertPose2D(ob);
	Eigen::Isometry2f goal_transform = convertPose2D(transform_goal);
	// posizione dagli oggetti del laser
	Eigen::Vector2f p_min; 
	//cloud points 540 
	p_min(0) = cloud.points[540].x; 
	p_min(1) = cloud.points[540].y;
	//posizione dell'oggetto
	auto ob_pos = laserTrans * p_min;
	float ob_dist = sqrt(std::pow(ob_pos(0),2)+std::pow(ob_pos(1),2));
	//range fra 340 e 740
	const float min = cloud.points.size()/2-N;
	const float max = cloud.points.size()/2+N;


	for(int i=min;i<=max;i++){
	  Eigen::Vector2f v;
	  
	  v(0) = cloud.points[i].x; 
	  v(1) = cloud.points[i].y;  

	  auto ob_posA = laserTrans * v;
	  float ob_distA = sqrt(std::pow(ob_posA(0),2)+std::pow(ob_posA(1),2));
	  //ricerca del minimo
	  if(ob_distA < ob_dist){
	    ob_dist = ob_distA;
	    ob_pos = ob_posA;
	  }
	}

	p_min(0) = goal_x;
    	p_min(1) = goal_y;
    	p_min = goal_transform * p_min;
    	float robot_goal_x=p_min(0); //coordinata x del punto goal rispetto al robot
    	float robot_goal_y=p_min(1);
	//distanza_robot_goal=sqrt(std::pow(robot_goal_y,2) +std::pow(robot_goal_x,2));

	if(ob_dist < 0.5 ){
	// intensità della forza
	  force_int = (1.0 / ob_dist) * 35 ;
	  float forceX = -(ob_pos(0) / ob_dist) * force_int;
	  float forceY = -(ob_pos(1) / ob_dist) * force_int;

	  
	//definisco messaggi della velocità negativa
	  msg_send.linear.x = robot_goal_x + forceX;
	  msg_send.linear.y = robot_goal_y + forceY;
	
	//definisco la velocità angolare
	  if(ob_pos(1) > 0)
	    msg_send.angular.z = - 0.8*force_int; 
	  else if (ob_pos(1) < 0)
	    msg_send.angular.z = 0.8*force_int;
	//pubblico e il robot esegue
	  vel_cmd_sub.publish(msg_send);
 	ROS_INFO("comando velocita': [vx:%f, vy%f, wz:%f]\n", msg_send.linear.x, msg_send.linear.y, msg_send.angular.z);

	}
}

void Goal_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	goal_arrived=true;
    	goal_x=msg->pose.position.x;
    	goal_y=msg->pose.position.y;
    	ROS_INFO("DESTINAZIONE: (%f %f)\n", goal_x, goal_y);
}

void Laser_Callback(const sensor_msgs::LaserScan::ConstPtr& laser_msg){
	if(goal_arrived)laser_scan_sub_callback(laser_msg);
}

int main(int argc, char **argv){

	ros::init(argc, argv, "robot_avoidance");
	ros::NodeHandle p;
	
 	vel_cmd_sub=p.advertise<geometry_msgs::Twist>(argv[1], 1);
   	ROS_INFO("publisher su %s avviato\n", argv[1]);
	ros::Subscriber laser_scan_sub = p.subscribe(argv[2], 1, Laser_Callback); 

	ros::Subscriber goal_sub = p.subscribe("/move_base_simple/goal", 1, Goal_Callback);
        ROS_INFO("subscriber su /move_base_simple/goal avviato\n");


	ROS_INFO("ricezione messaggi avviata\n");
	ros::spin();
	return 0;

}
