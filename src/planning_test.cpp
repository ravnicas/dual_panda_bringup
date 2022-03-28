#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit/macros/console_colors.h>
// https://ros-planning.github.io/moveit_tutorials/doc/move_group_interface/move_group_interface_tutorial.html

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

void prompt(const std::string& message)
{
  printf(MOVEIT_CONSOLE_COLOR_GREEN "\n%s" MOVEIT_CONSOLE_COLOR_RESET, message.c_str());
  fflush(stdout);
  while (std::cin.get() != '\n' && rclcpp::ok())
    ;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);
  // For current state monitor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "panda_dual";

  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // We can print the name of the reference frame for this robot.
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  // Planning to a Pose goal
  geometry_msgs::msg::PoseStamped target_pose1;
  target_pose1.pose.orientation.w = 0.478573;
  target_pose1.pose.orientation.x = 0.560641;
  target_pose1.pose.orientation.y = 0.520299;
  target_pose1.pose.orientation.z = 0.431206;
  target_pose1.pose.position.x = -0.6;
  target_pose1.pose.position.y = 1.35;
  target_pose1.pose.position.z = 1.61;
  target_pose1.header.frame_id = "world";
  target_pose1.header.stamp = move_group_node->now();

  geometry_msgs::msg::PoseStamped target_pose2;
  target_pose2.pose.orientation.w = -0.454849;
  target_pose2.pose.orientation.x = -0.54129;
  target_pose2.pose.orientation.y = -0.541398;
  target_pose2.pose.orientation.z = 0.454979;
  target_pose2.pose.position.x = 0.6;
  target_pose2.pose.position.y = 0.45;
  target_pose2.pose.position.z = 1.61;
  target_pose2.header.frame_id = "world";
  target_pose2.header.stamp = move_group_node->now();

  move_group.setPoseTarget(target_pose1, "panda_left_link8");
  move_group.setPoseTarget(target_pose2, "panda_right_link8");

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success)
    move_group.execute(my_plan);

  rclcpp::shutdown();
  return 0;
}