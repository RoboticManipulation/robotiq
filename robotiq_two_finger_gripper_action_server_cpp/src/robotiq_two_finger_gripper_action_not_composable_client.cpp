#include <chrono>
#include <cinttypes>
#include <memory>
#include <string>

#include<tuple>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <robotiq_two_finger_gripper_actions/action/robotiq_two_finger_gripper.hpp>
#include <robotiq_2f_gripper_control/msg/robotiq2_f_gripper_robot_input.hpp>
#include <robotiq_2f_gripper_control/msg/robotiq2_f_gripper_robot_output.hpp>

using RobotiqTwoFingerGripper = robotiq_two_finger_gripper_actions::action::RobotiqTwoFingerGripper;
rclcpp::Node::SharedPtr g_node = nullptr;


void feedback_callback(
  rclcpp_action::ClientGoalHandle<RobotiqTwoFingerGripper>::SharedPtr,
  const std::shared_ptr<const RobotiqTwoFingerGripper::Feedback> feedback)
{
    float position = feedback->current_position;
    RCLCPP_INFO(g_node->get_logger(),"Received feedback. Currently the gripper is %f m wide", position);
}

std::tuple<bool, float> execute_gripper_action(rclcpp::Node::SharedPtr g_node, float goal_position, int force){
  bool success = false;
  float position = -100.0;

  auto gripper_action_client = rclcpp_action::create_client<RobotiqTwoFingerGripper>(g_node, "robotiq_two_finger_gripper");

  if (!gripper_action_client->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(g_node->get_logger(), "Action server not available after waiting");
    success = false;
    return std::make_tuple(success, position);
  }

  // Populate a goal
  auto goal_msg = RobotiqTwoFingerGripper::Goal();
  goal_msg.goal_position = goal_position;
  goal_msg.force = force;

  RCLCPP_INFO(g_node->get_logger(), "Sending goal");
  // Ask server to achieve some goal and wait until it's accepted
  auto send_goal_options = rclcpp_action::Client<RobotiqTwoFingerGripper>::SendGoalOptions();
  send_goal_options.feedback_callback = feedback_callback;
  auto goal_handle_future = gripper_action_client->async_send_goal(goal_msg, send_goal_options);

  if (rclcpp::spin_until_future_complete(g_node, goal_handle_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(g_node->get_logger(), "send goal call failed :(");
    success = false;
    return std::make_tuple(success, position);
  }

  rclcpp_action::ClientGoalHandle<RobotiqTwoFingerGripper>::SharedPtr goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(g_node->get_logger(), "Goal was rejected by server");
    success = false;
    return std::make_tuple(success, position);
  }

  // Wait for the server to be done with the goal
  auto result_future = gripper_action_client->async_get_result(goal_handle);

  RCLCPP_INFO(g_node->get_logger(), "Waiting for result");
  if (rclcpp::spin_until_future_complete(g_node, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(g_node->get_logger(), "get result call failed :(");
    success = false;
    return std::make_tuple(success, position);
  }

  rclcpp_action::ClientGoalHandle<RobotiqTwoFingerGripper>::WrappedResult wrapped_result = result_future.get();

  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(g_node->get_logger(), "Goal was aborted");
      success = false;
      return std::make_tuple(success, position);
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(g_node->get_logger(), "Goal was canceled");
      success = false;
      return std::make_tuple(success, position);
    default:
      RCLCPP_ERROR(g_node->get_logger(), "Unknown result code");
      success = false;
      return std::make_tuple(success, position);
  }

  RCLCPP_INFO(g_node->get_logger(), "result received");

  position = wrapped_result.result->final_position;
  RCLCPP_INFO(g_node->get_logger(),"Final position, the gripper is %f cm wide", position);
 

  gripper_action_client.reset();

  return std::make_tuple(success, position);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("robotiq_two_finger_gripper_action_client");

  float goal_position = 0.05;
  int force = 50;

  bool success = false;
  float position = -100.0;
  std::tie(success, position) = execute_gripper_action(g_node, goal_position, force);

  RCLCPP_INFO(g_node->get_logger(),"Received success = %s, final_position = %f cm", success ? "True" : "False", position);
 

  g_node.reset();
  rclcpp::shutdown();
  return 0;
}