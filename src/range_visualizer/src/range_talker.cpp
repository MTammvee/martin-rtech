#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <sensor_msgs/Range.h>
 

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");


  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<sensor_msgs::Range>("chatter", 1000);
 
  ros::Rate loop_rate(10);

  int count = 0;

  while (ros::ok())
  {

    sensor_msgs::Range msg;

    msg.header.frame_id = "range";
    msg.range = ((float)rand()/RAND_MAX)*4;
    msg.field_of_view = 0.785; 
    msg.min_range = 0.02;
    msg.max_range = 4;
    
    ROS_INFO("%f", msg.range);

    



    chatter_pub.publish(msg);

    ros::spinOnce();
 
    loop_rate.sleep();
    ++count;
  }
 
 
  return 0;
}
