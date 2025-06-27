#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

bool execute_position_with_retry(moveit::planning_interface::MoveGroupInterface &group,
                                 double x, double y, double z,
                                 int max_attempts = 15)
{
  group.setPlanningTime(8.0);
  group.setPositionTarget(x, y, z);
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  for (int i = 1; i <= max_attempts; ++i)
  {
    if (group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
      if (group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS)
      {
        return true;
      }
    }
    RCLCPP_WARN(rclcpp::get_logger("retry"), "Attempt %d/%d failed, retrying...", i, max_attempts);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  RCLCPP_ERROR(rclcpp::get_logger("retry"), "All planning attempts failed.");
  return false;
}

void control_gripper(moveit::planning_interface::MoveGroupInterface &gripper,
                     const std::string &joint_name, double position)
{
  gripper.setJointValueTarget(joint_name, position);
  gripper.move();
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("pick_place_stack_node",
                  rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  auto logger = rclcpp::get_logger("pick_place");

  // Publicador del progreso
  auto progress_pub = node->create_publisher<std_msgs::msg::Int32>("pick_place/progress", 10);

  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface arm(node, "arm");
  // PLANNERS: "PilzPipeline" "OMPL::LinearPath"
  arm.setPlannerId("PilzPipeline");
  MoveGroupInterface gripper(node, "gripper");
  const std::string gripper_joint = "gripper_left_joint";

  // Coordenates
  double x = 0.25;
  double y = 0;
  double pre_grasp_z = 0.20;
  double grasp_z = 0.15;
  double lift_z = 0.20;
  double base_place_x = 0.2;
  double base_place_y = 0.2;
  double base_place_z = 0.15;
  double stack_step = 0.03;
  int stack_count = 3;

  // Read last ROS cycle completed parameter 
  int start_cycle = 0;
  node->declare_parameter("last_completed_cycle", 0);
  node->get_parameter("last_completed_cycle", start_cycle);

  for (int i = start_cycle; i < stack_count; ++i)
  {
    RCLCPP_INFO(logger, "------ Cycle %d ------", i + 1);

    std_msgs::msg::Int32 msg;
    msg.data = i + 1;
    progress_pub->publish(msg);

    control_gripper(gripper, gripper_joint, 0.01);  // Open
    RCLCPP_INFO(logger, "Opening Gripper...");

    if (!execute_position_with_retry(arm, x, y, pre_grasp_z)) return 1;
    if (!execute_position_with_retry(arm, x, y, grasp_z)) return 1;

    RCLCPP_INFO(logger, "Grabbing...");
    control_gripper(gripper, gripper_joint, 0.003);  // Closed

    RCLCPP_INFO(logger, "Lifting...");
    if (!execute_position_with_retry(arm, x, y, lift_z)) return 1;

    double place_z = base_place_z + i * stack_step;
    if (!execute_position_with_retry(arm, base_place_x, base_place_y, place_z)) return 1;

    RCLCPP_INFO(logger, "Placing...");
    control_gripper(gripper, gripper_joint, 0.01);  // Open

    RCLCPP_INFO(logger, "Retreating...");
    if (!execute_position_with_retry(arm, base_place_x, base_place_y, place_z + 0.10)) return 1;

    // save progress in parameters
    node->set_parameter(rclcpp::Parameter("last_completed_cycle", i + 1));
  }

  RCLCPP_INFO(logger, "Stacking task completed.");
  RCLCPP_INFO(logger, "Go Home...");
  if (!execute_position_with_retry(arm, 0.2, 0.0, 0.07)) return 1;
  if (!execute_position_with_retry(arm, 0.1, 0.0, 0.07)) return 1;
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
