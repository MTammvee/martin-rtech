/*
 beginner_tutorials/AddTwoInts.h is the header file generated from the srv file that we created earlier. 
*/
#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"

/*
 This function provides the service for adding two ints, it takes in the request and response type defined in the srv file and returns a boolean. 
*/
bool add(beginner_tutorials::AddTwoInts::Request  &req,
      beginner_tutorials::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}
 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;
/*
 Here the two ints are added and stored in the response. Then some information about the request and response are logged. Finally the service returns true when it is complete. 
*/
  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();
 
  return 0;
}
