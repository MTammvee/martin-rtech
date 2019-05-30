#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Twist.h"


#include <sstream>

geometry_msgs::Twist twist;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "map");


  ros::NodeHandle n;

  ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  ros::Publisher dist_publisher= n.advertise<geometry_msgs::Twist>("/cmd_vel",1000); 


  ros::Rate loop_rate(10);

  while (ros::ok())
  {

	visualization_msgs::Marker marker;
	marker.header.frame_id = "visualization_marker";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	//marker.pose.position.x = 1;
	//marker.pose.position.y = 1;
	//marker.pose.position.z = 1;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 1;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;

	twist.linear.x=2;
	twist.linear.y=marker.pose.position.y;
	twist.linear.z=marker.pose.position.z;
	dist_publisher.publish(twist);
	
	//only if using a MESH_RESOURCE marker type:
	marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
	vis_pub.publish( marker );
  }


  return 0;
}
