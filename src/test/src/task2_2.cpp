#include "ros/ros.h"
#include "std_msgs/String.h"
#include <vector>
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Point.h"
#include <sstream>
#include "geometry_msgs/Twist.h"

sensor_msgs::Imu imu;
geometry_msgs::Twist twist;
geometry_msgs::Point angular_velocity;
geometry_msgs::Point linear_acceleration;
std::vector<float> ang_x;
std::vector<float> lin_x;
std::vector<float> lin_y;
std::vector<float> time_sec;
std::vector<float> linear_velocity_x;
std::vector<float> linear_velocity_y;
int t_sec;
int t_nsec;
float time_calc;
float avg_velocity0 = 0;

int average_count=10;

 
void Callback(const sensor_msgs::Imu &data)
{
	ang_x.push_back(data.angular_velocity.x);
	lin_x.push_back(data.linear_acceleration.x);
	lin_y.push_back(data.linear_acceleration.y);
	

	t_nsec=data.header.stamp.nsec;
	t_sec=data.header.stamp.sec;
	t_sec=t_sec-1557814000;
	time_calc=float(t_sec)+float(t_nsec*pow(10,-9));
	time_sec.push_back(time_calc);	
	
	time_sec.push_back(time_calc);

}

int main(int argc, char **argv)
{
	linear_velocity_x.push_back(avg_velocity0);
	linear_velocity_y.push_back(avg_velocity0);

	ros::init(argc, argv, "filter_imu");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/imu/data_MPU",1000,Callback);
	ros::Publisher range_publisher= n.advertise<geometry_msgs::Point>("filtered_data",1000);
	ros::Publisher range_publisher2= n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
	ros::Rate loop_rate(30);
	int count =0;

	while(ros::ok())
	{
	//Running average filter of size 10
		if(ang_x.size()>(average_count-1)){

			float running_avg_value1_1=0;
			float running_avg_value2_1=0;
			float running_avg_value2_2=0;
		
			for (int i=0;i<average_count;i++){
	
				running_avg_value1_1+=ang_x[i];
				running_avg_value2_1+=lin_x[i];
				running_avg_value2_2+=lin_y[i];
				
			}	

			running_avg_value1_1/=average_count;
			running_avg_value2_1/=average_count;
			running_avg_value2_2/=average_count;

			angular_velocity.x = running_avg_value1_1;
			linear_acceleration.x = running_avg_value2_1;
			linear_acceleration.y = running_avg_value2_2;
		
			ang_x.erase(ang_x.begin());
			lin_x.erase(lin_x.begin());
			lin_y.erase(lin_y.begin());
		
			range_publisher.publish(angular_velocity);
		
		
		}
	

		if(time_sec.size()>1 && lin_x.size()>1){

			//v = v0 + at		
		
			float dt = time_sec[1] - time_sec[0];
			float vel_app_x = linear_velocity_x[0] + dt * linear_acceleration.x;
			float vel_app_y = linear_velocity_y[0] + dt * linear_acceleration.y;

			twist.linear.x = vel_app_x;
			twist.linear.y = vel_app_y;
	
			time_sec.erase(time_sec.begin());

			linear_velocity_x[0] = vel_app_x;
			linear_velocity_y[0] = vel_app_y;	
		
			range_publisher2.publish(twist);
	
			}
		
	ros::spinOnce();
	

	}

	return 0;
}	

