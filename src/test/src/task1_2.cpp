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

	ros::init(argc, argv, "task1_2");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("positioning/point", 1, Callback);  
	//ros::Publisher dist_publisher= n.advertise<geometry_msgs::Twist>("cmd_vel_30",1000);
	ros::Publisher dist_publisher= n.advertise<geometry_msgs::Point>("cmd_vel_30",1000); 
	ros::Rate r(30);
	float x_vel30 = 0;
	float y_vel30 = 0;
	float z_vel30 = 0;
	float x_est=0;
	float y_est=0;
	float z_est=0;
	while(ros::ok())
	{
		if(x.size()>1){
			x_est=x[0];
			y_est=y[0];
			z_est=z[0];

			float dx = x[1]-x[0]; 
			float dy = y[1]-y[0];
			float dz = z[1]-z[0];
	
			x.erase(x.begin());
			y.erase(y.begin());
			z.erase(z.begin());
			time_sec.erase(time_sec.begin());
			x_vel30 = dx / 30;
			y_vel30 = dy / 30;
			z_vel30 = dz / 30;
		
		}
		
		distance.x = x_est + x_vel30;
		distance.y = y_est + y_vel30;
		distance.z = z_est + z_vel30;

		dist_publisher.publish(distance);

		r.sleep();

		ros::spinOnce();
	}	
  return 0;
}
