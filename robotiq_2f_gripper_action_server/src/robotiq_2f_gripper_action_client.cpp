// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>
//#include <sstream>

#include <robotiq_2f_gripper_actions/action/robotiq2f_gripper.hpp>
#include "rclcpp/rclcpp.hpp"
// TODO(jacobperron): Remove this once it is included as part of 'rclcpp.hpp'
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

//#include "action_tutorials_cpp/visibility_control.h"

#include <robotiq_2f_gripper_control/msg/robotiq2_f_gripper_robot_input.hpp>
#include <robotiq_2f_gripper_control/msg/robotiq2_f_gripper_robot_output.hpp>


typedef robotiq_2f_gripper_control::msg::Robotiq2FGripperRobotInput GripperInput;
typedef robotiq_2f_gripper_control::msg::Robotiq2FGripperRobotOutput GripperOutput;


namespace robotiq_2f_gripper_action_server
{
class Robotiq2fGripperActionClient : public rclcpp::Node
{
public:
  using Robotiq2fGripper = robotiq_2f_gripper_actions::action::Robotiq2fGripper;
    using GoalHandleRobotiq2fGripper = rclcpp_action::ClientGoalHandle<Robotiq2fGripper>;

  //ACTION_TUTORIALS_CPP_PUBLIC
  explicit Robotiq2fGripperActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("robotiq_2f_gripper_action_client", node_options)
  {
    this->client_ptr_ = rclcpp_action::create_client<Robotiq2fGripper>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "gripper");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&Robotiq2fGripperActionClient::send_goal, this));
  }

  //ACTION_TUTORIALS_CPP_PUBLIC
  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
      return;
    }

    auto goal_msg = Robotiq2fGripper::Goal();
    goal_msg.goal_position = 0.007;
    goal_msg.force = 100;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Robotiq2fGripper>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&Robotiq2fGripperActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&Robotiq2fGripperActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&Robotiq2fGripperActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Robotiq2fGripper>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  //ACTION_TUTORIALS_CPP_LOCAL
  void goal_response_callback(GoalHandleRobotiq2fGripper::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      rclcpp::shutdown();
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  //ACTION_TUTORIALS_CPP_LOCAL
  void feedback_callback(
    GoalHandleRobotiq2fGripper::SharedPtr,
    const std::shared_ptr<const Robotiq2fGripper::Feedback> feedback)
  {
    float position = feedback->current_position;
    RCLCPP_INFO(this->get_logger(),"Recieved Feedback. Currently the gripper is %f cm wide", position);

  }

  //ACTION_TUTORIALS_CPP_LOCAL
  void result_callback(const GoalHandleRobotiq2fGripper::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    float position = result.result->final_position;
    RCLCPP_INFO(this->get_logger(),"Final position, the gripper is %f cm wide", position);
 
    rclcpp::shutdown();
  }
};  // class Robotiq2fGripperActionClient

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(robotiq_2f_gripper_action_server::Robotiq2fGripperActionClient)
