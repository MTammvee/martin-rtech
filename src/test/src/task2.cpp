#include "ros/ros.h"
#include "std_msgs/String.h"
#include <vector>
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"
#include <sstream>

sensor_msgs::Imu imu;
geometry_msgs::Vector3 angular_velocity;
geometry_msgs::Vector3 linear_acceleration;

std::vector<float> ang_x;
std::vector<float> ang_y;
std::vector<float> ang_z;

std::vector<float> lin_x;
std::vector<float> lin_y;
std::vector<float> lin_z;

int average_count=10;
 
void Callback(const sensor_msgs::Imu &data)
{
	ang_x.push_back(data.angular_velocity.x);
	ang_y.push_back(data.angular_velocity.y);
	ang_z.push_back(data.angular_velocity.z);

	lin_x.push_back(data.linear_acceleration.x);
	lin_y.push_back(data.linear_acceleration.y);
	lin_z.push_back(data.linear_acceleration.z);


}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "filter_imu");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/imu/data_MPU",1000,Callback);
	ros::Publisher range_publisher= n.advertise<sensor_msgs::Imu>("filtered_data",1000);
	ros::Rate loop_rate(30);
	int count =0;


	while(ros::ok())
	{
	//Running average filter of size 10
		if(ang_x.size()>(average_count-1)){

			float running_avg_value1_1=0;
			float running_avg_value1_2=0;
			float running_avg_value1_3=0;

			float running_avg_value2_1=0;
			float running_avg_value2_2=0;
			float running_avg_value2_3=0;

			for (int i=0;i<average_count;i++){

				running_avg_value1_1+=ang_x[i];
				running_avg_value1_2+=ang_y[i];
				running_avg_value1_3+=ang_z[i];

				running_avg_value2_1+=lin_x[i];
				running_avg_value2_2+=lin_y[i];
				running_avg_value2_3+=lin_z[i];

			}
	
			running_avg_value1_1/=average_count;
			running_avg_value1_2/=average_count;
			running_avg_value1_2/=average_count;

			running_avg_value2_1/=average_count;
			running_avg_value2_2/=average_count;
			running_avg_value2_2/=average_count;

			angular_velocity.x = running_avg_value1_1;
			angular_velocity.y = running_avg_value1_2;
			angular_velocity.z = running_avg_value1_3;

			linear_acceleration.x = running_avg_value2_1;
			linear_acceleration.y = running_avg_value2_2;
			linear_acceleration.z = running_avg_value2_3;

			ang_x.erase(ang_x.begin());
			ang_y.erase(ang_y.begin());
			ang_z.erase(ang_z.begin());

			lin_x.erase(lin_x.begin());
			lin_y.erase(lin_y.begin());
			lin_z.erase(lin_z.begin());

			imu.angular_velocity = angular_velocity;
			imu.linear_acceleration = linear_acceleration;

			range_publisher.publish(imu);
		}

	ros::spinOnce();
	
	}

	return 0;
}
