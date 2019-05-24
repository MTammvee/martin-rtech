#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <math.h>   

geometry_msgs::Point distance;
geometry_msgs::Twist twist;

std::vector<float> x;
std::vector<float> y;
std::vector<float> z;
std::vector<float> time_sec;

int t_nsec;
int t_sec;
float time_calc;
int i=0;

void Callback(const geometry_msgs::PointStamped &point)
{

	x.push_back(point.point.x);
	y.push_back(point.point.y);
	z.push_back(point.point.z);
	t_nsec=point.header.stamp.nsec;
	t_sec=point.header.stamp.sec;
	t_sec=t_sec-1557814000;
	time_calc=float(t_sec)+float(t_nsec*pow(10,-9));
	time_sec.push_back(time_calc);
	
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "task1");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("positioning/point", 1000, Callback);  
	ros::Publisher dist_publisher= n.advertise<geometry_msgs::Twist>("cmd_vel",1000); 
	ros::Rate loop_rate(30);
	while(ros::ok())
	{
		if(x.size()>1){
			float x_vel=x[1]-x[0]; 
			float y_vel=y[1]-y[0];
			float z_vel=z[1]-z[0];
        		float dt = (time_sec[1]-time_sec[0]);
	
			twist.linear.x=x_vel/dt;
			twist.linear.y=y_vel/dt;
			twist.linear.z=z_vel/dt;
	
			float dx = x[1]-x[0]; 
			float dy = y[1]-y[0];
	
			twist.angular.z = (atan2(dy,dx))/dt;

			x.erase(x.begin());
			time_sec.erase(time_sec.begin());

		}

		dist_publisher.publish(twist);
 
		ros::spinOnce();
	}	
  return 0;
}
