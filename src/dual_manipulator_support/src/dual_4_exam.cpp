#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP1 = "left_manipulator";
  static const std::string PLANNING_GROUP2 = "right_manipulator";

  moveit::planning_interface ::MoveGroupInterface move_group1(PLANNING_GROUP1);
  moveit::planning_interface ::MoveGroupInterface move_group2(PLANNING_GROUP2);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const robot_state::JointModelGroup* joint_model_group1 =
      move_group1.getCurrentState()->getJointModelGroup(PLANNING_GROUP1);
  const robot_state::JointModelGroup* joint_model_group2 =
      move_group2.getCurrentState()->getJointModelGroup(PLANNING_GROUP2);


  moveit::planning_interface::MoveGroupInterface::Plan my_plan1;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan2;

  move_group1.clearPathConstraints();
  move_group2.clearPathConstraints();

  move_group1.setStartStateToCurrentState();
  move_group2.setStartStateToCurrentState();

   geometry_msgs::Pose target_pose1 = move_group1.getCurrentPose().pose;
   geometry_msgs::Pose target_pose2 = move_group2.getCurrentPose().pose;

   std::vector<geometry_msgs::Pose> waypoints1;
   std::vector<geometry_msgs::Pose> waypoints2;
   waypoints.push_back(target_pose1);
   waypoints.push_back(target_pose2);

   target_pose1.position.z -= 0.2;
   target_pose1.position.y += 0.2;
   waypoints.push_back(target_pose1);  // up and left

   target_pose2.position.z -= 0.2;
   target_pose2.position.y += 0.2;
   waypoints.push_back(target_pose2);  // up and left
   

   target_pose1.position.y -= 0.4;
   waypoints.push_back(target_pose1);  // right
 
   target_pose2.position.y -= 0.4;
   waypoints.push_back(target_pose2);  

   target_pose1.position.z += 0.2;
   target_pose1.position.y += 0.2;
   waypoints.push_back(target_pose1);  // up and left

   target_pose2.position.z += 0.2;
   target_pose2.position.y += 0.2;
   waypoints.push_back(target_pose2);

  move_group1.setMaxVelocityScalingFactor(0.1);
  move_group2.setMaxVelocityScalingFactor(0.1);

  moveit_msgs::RobotTrajectory trajectory1;
  moveit_msgs::RobotTrajectory trajectory2;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction1 = move_group1.computeCartesianPath(waypoints1, eef_step, jump_threshold, trajectory);
  double fraction2 = move_group2.computeCartesianPath(waypoints2, eef_step, jump_threshold, trajectory);

  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i),   rvt::SMALL);
  visual_tools.trigger();

   my_plan1.trajectory_ = trajectory1;
   my_plan2.trajectory_ = trajectory2; 
   move_group1.execute(my_plan1);
   move_group2.execute(my_plan2);


  ros::shutdown();
  return 0;
}
