#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{

/**
* The ros::init() function needs to see argc and argv so that it can perform
* any ROS arguments and name remapping that were provided at the command line.
* For programmatic remappings you can use a different version of init() which takes
* remappings directly, but for most command-line programs, passing argc and argv is
* the easiest way to do it.  The third argument to init() is the name of the node.
*
* You must call one of the versions of ros::init() before using any other
* part of the ROS system.
*
*Initate the node called
*/
  ros::init(argc, argv, "move_group_interface_tutorial");

/**
* NodeHandle is the main access point to communications with the ROS system.
* The first NodeHandle constructed will fully initialize this node, and the last
* NodeHandle destructed will close down the node
**/
  ros::NodeHandle node_handle;

/**
*The end result is that without a little bit of work from the user your subscription, service and other callbacks will never be called. The most common solution is *ros::spin(), but you must use one of the options below. 
*
*A more useful threaded spinner is the AsyncSpinner. Instead of a blocking spin() call, it has start() and stop() calls,
*and will automatically stop when it is destroyed.
*
*Go and reads the data in the que that has been published
**/
  ros::AsyncSpinner spinner(1);
  spinner.start();


/**
*SETUP
*MoveIt! operates on sets of joints called “planning groups” and stores them in an object called the JointModelGroup. Throughout MoveIt! the terms
*“planning group” and “joint model group” are used interchangably.
*
*Defining a var called planning_group
**/
  static const std::string PLANNING_GROUP = "sia5";


/**
*The MoveGroup class can be easily setup using just the name of the planning group you would like to control and plan for.
*
*moveit namespace, planngin_interface - package, method(function) of this package
*
*Passing the plaanign_group
**/

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);


/**
*Raw pointers are frequently used to refer to the planning group for improved performance.
**/
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);



/**
*GETTING BASIC INFORMATION
*
*print the reference plane for this robot
*
*print the end effector link for this group
**/
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());




/**
*Represtation of a motion plan(as a ROS msg)
*Saves the plan to my_plan
**/
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

/**
*Defining starting pose postition.
**/
   geometry_msgs::Pose target_pose3 = move_group.getCurrentPose().pose;


/**
*Movements are constructed of different paths (waypoints)
*
*Cereating a vec of poses called waypoitns
*
*pushing the poses into vec
**/
   std::vector<geometry_msgs::Pose> waypoints;
   waypoints.push_back(target_pose3);

   target_pose3.position.z -= 0.2;
   target_pose3.position.y += 0.2;
   waypoints.push_back(target_pose3);  // up and left
   

   target_pose3.position.y -= 0.4;
   waypoints.push_back(target_pose3);  // right
   

   target_pose3.position.z += 0.2;
   target_pose3.position.y += 0.2;
   waypoints.push_back(target_pose3);  // up and left

/**
*Decrease the oscillations and jumps lvl
**/

  move_group.setMaxVelocityScalingFactor(0.1);

/**
*Maintain a sequence of waypoints and the time durations between these waypoints.
*
*We want the Cartesian path to be interpolated at a resolution of 1 cm which is why we will specify 0.01 as the max step in Cartesian translation.
* We will specify the *jump threshold as 0.0, effectively disabling it
**/
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

/**
*passing a vec of waypoints and it generates the trajectry
*
*
**/
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);



/**
*Paasing the trajectory to plan and execute it
**/
   my_plan.trajectory_ = trajectory; 
   move_group.execute(my_plan);


  ros::shutdown();
  return 0;
}
