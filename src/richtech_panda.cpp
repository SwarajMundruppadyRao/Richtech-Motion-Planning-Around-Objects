#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "richtech_panda", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("richtech_panda");

  // We spin up a SingleThreadedExecutor for the current state monitor to get
  // information about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools =
      moveit_visual_tools::MoveItVisualTools{ node, "panda_link0", rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                              move_group_interface.getRobotModel() };
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Create a closure for updating the text in rviz
  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  };
  auto const prompt = [&moveit_visual_tools](auto text) { moveit_visual_tools.prompt(text); };
  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools, jmg = move_group_interface.getRobotModel()->getJointModelGroup("panda_arm")](
          auto const trajectory) { moveit_visual_tools.publishTrajectoryLine(trajectory, jmg); };

  // Set a target Pose with updated values !!!
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.y = 0.8;
    msg.orientation.w = 0.6;
    msg.position.x = 0.1;
    msg.position.y = 0.4;
    msg.position.z = 0.4;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create collision object for the robot to avoid
  auto const collision_object = [frame_id = move_group_interface.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box1";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.5;
    primitive.dimensions[primitive.BOX_Y] = 0.1;
    primitive.dimensions[primitive.BOX_Z] = 0.5;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.2;
    box_pose.position.y = 0.2;
    box_pose.position.z = 0.25;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_object);

  // Save the starting pose
  auto start_pose = move_group_interface.getCurrentPose().pose;

  // Add more collision objects
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

  // Box 1 (already added)
  collision_objects.push_back(collision_object);

  // Box 2
  moveit_msgs::msg::CollisionObject box2;
  box2.header.frame_id = move_group_interface.getPlanningFrame();
  box2.id = "box2";
  shape_msgs::msg::SolidPrimitive primitive2;
  primitive2.type = primitive2.BOX;
  primitive2.dimensions = {0.2, 0.2, 0.2};
  geometry_msgs::msg::Pose box2_pose;
  box2_pose.orientation.w = 1.0;
  box2_pose.position.x = 0.3;
  box2_pose.position.y = -0.2;
  box2_pose.position.z = 0.1;
  box2.primitives.push_back(primitive2);
  box2.primitive_poses.push_back(box2_pose);
  box2.operation = box2.ADD;
  collision_objects.push_back(box2);

  // Box 3
  moveit_msgs::msg::CollisionObject box3;
  box3.header.frame_id = move_group_interface.getPlanningFrame();
  box3.id = "box3";
  shape_msgs::msg::SolidPrimitive primitive3;
  primitive3.type = primitive3.BOX;
  primitive3.dimensions = {0.1, 0.4, 0.1};
  geometry_msgs::msg::Pose box3_pose;
  box3_pose.orientation.w = 1.0;
  box3_pose.position.x = -0.2;
  box3_pose.position.y = 0.0;
  box3_pose.position.z = 0.2;
  box3.primitives.push_back(primitive3);
  box3.primitive_poses.push_back(box3_pose);
  box3.operation = box3.ADD;
  collision_objects.push_back(box3);

  // Apply all collision objects
  planning_scene_interface.applyCollisionObjects(collision_objects);

  // Sequence of target poses
  std::vector<geometry_msgs::msg::Pose> waypoints;

  // First target pose (already defined)
  waypoints.push_back(target_pose);

  // Second pose (move around box2)
  geometry_msgs::msg::Pose pose2 = target_pose;
  pose2.position.x = 0.35;
  pose2.position.y = -0.3;
  pose2.position.z = 0.3;
  waypoints.push_back(pose2);

  // Remove all other poses (only plan for first two)

  // Set planner to CHOMP
  move_group_interface.setPlannerId("chomp");

  // Plan and execute for each pose
  for (size_t i = 0; i < waypoints.size(); ++i)
  {
    move_group_interface.setPoseTarget(waypoints[i]);
    draw_title("Planning to pose " + std::to_string(i+1));
    moveit_visual_tools.trigger();
    prompt("Press 'next' in the RvizVisualToolsGui window to plan");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_interface.plan(plan));
    if (success)
    {
      draw_trajectory_tool_path(plan.trajectory_);
      moveit_visual_tools.trigger();
      prompt("Press 'next' in the RvizVisualToolsGui window to execute");
      draw_title("Executing");
      moveit_visual_tools.trigger();
      move_group_interface.execute(plan);
    }
    else
    {
      draw_title("Planning Failed!");
      moveit_visual_tools.trigger();
      RCLCPP_ERROR(logger, "Planning failed for pose %zu!", i+1);
      break;
    }
  }

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}