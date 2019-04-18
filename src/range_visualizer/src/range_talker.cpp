
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <sensor_msgs/Range.h>
 
/*
*Defining global variable
*/
sensor_msgs::Range msg;

/**
*This is the callback function that will get called when a new message has arrived on the range topic. The message is passed in a boost shared_ptr, which means you can store it off if you want, without worrying about it getting deleted underneath you, and without copying the underlying data. 
*/
void sensorCallback(sensor_msgs:: Range sensor){
	/*
	double a = 100.0;
	
	if(msg.range < a ){
		msg.range = sensor.range;
	}else{
		msg.range = 0;
	}
	*/ 
	
	msg.range = sensor.range/2;


}

/**
* The ros::init() function needs to see argc and argv so that it can perform
* any ROS arguments and name remapping that were provided at the command line.
* For programmatic remappings you can use a different version of init() which takes
* remappings directly, but for most command-line programs, passing argc and argv is
* the easiest way to do it.  The third argument to init() is the name of the node.
*
* You must call one of the versions of ros::init() before using any other
* part of the ROS system.
*/
int main(int argc, char **argv)
{

  ros::init(argc, argv, "range_talker");

  
  msg.header.frame_id = "ultrasonic";
  msg.range = 0;
  msg.field_of_view = 0.785; 
  msg.min_range = 0.02;
  msg.max_range = 4;
/*    
  ROS_INFO("%f", msg.range);
*/
/**
* NodeHandle is the main access point to communications with the ROS system.
* The first NodeHandle constructed will fully initialize this node, and the last
* NodeHandle destructed will close down the node.
*/

  ros::NodeHandle n;

/**
* The subscribe() call is how you tell ROS that you want to receive messages
* on a given topic.  This invokes a call to the ROS
* master node, which keeps a registry of who is publishing and who
* is subscribing.  Messages are passed to a callback function, here
* called chatterCallback.  subscribe() returns a Subscriber object that you
* must hold on to until you want to unsubscribe.  When all copies of the Subscriber
* object go out of scope, this callback will automatically be unsubscribed from
* this topic.
*
* The second parameter to the subscribe() function is the size of the message
* queue.  If messages are arriving faster than they are being processed, this
* is the number of messages that will be buffered up before beginning to throw
* away the oldest ones.
*/

  ros::Subscriber range_subscriber = n.subscribe("ultrasonic/raw", 1000, sensorCallback);
/**
* The advertise() function is how you tell ROS that you want to
* publish on a given topic name. This invokes a call to the ROS
* master node, which keeps a registry of who is publishing and who
* is subscribing. After this advertise() call is made, the master
* node will notify anyone who is trying to subscribe to this topic name,
* and they will in turn negotiate a peer-to-peer connection with this
* node.  advertise() returns a Publisher object which allows you to
* publish messages on that topic through a call to publish().  Once
* all copies of the returned Publisher object are destroyed, the topic
* will be automatically unadvertised.
*
* The second parameter to advertise() is the size of the message queue
* used for publishing messages.  If messages are published more quickly
* than we can send them, the number here specifies how many messages to
* buffer up before throwing some away.
*/

  ros::Publisher range_talker = n.advertise<sensor_msgs::Range>("ultrasonic/filtered", 1000);	
/*
*frequency, in hz
*/
  ros::Rate loop_rate(10);

/**
* A count of how many messages we have sent. This is used to create
* a unique string for each message.
*/

  int count = 0;

  while (ros::ok())
  {
  /**
  * This is a message object. You stuff it with data, and then publish it.
  */

    range_talker.publish(msg);

    ros::spinOnce();
 
    loop_rate.sleep();
    ++count;
  }
 
 
  return 0;
}
