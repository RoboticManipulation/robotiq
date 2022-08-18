#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <boost/thread/thread.hpp>

#include "action_tutorials_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "action_tutorials_cpp/visibility_control.h"

#include <control_msgs/GripperCommandAction.h>

#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_input.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>

namespace robotiq_2f_gripper_action_server
{
typedef robotiq_2f_gripper_control::Robotiq2FGripper_robot_input GripperInput;
typedef robotiq_2f_gripper_control::Robotiq2FGripper_robot_output GripperOutput;

typedef control_msgs::GripperCommandGoal GripperCommandGoal;
typedef control_msgs::GripperCommandFeedback GripperCommandFeedback;
typedef control_msgs::GripperCommandResult GripperCommandResult;

struct Robotiq2FGripperParams
{
    double min_gap_;  // meters
    double max_gap_;
    double min_effort_;  // N / (Nm)
    double max_effort_;
};

// Defines a default for the c2 model 85 gripper
Robotiq2FGripperParams c2_85_defaults()
{
    robotiq_2f_gripper_action_server::Robotiq2FGripperParams params;
    params.min_gap_ = -.017;
    params.max_gap_ = 0.085;
    params.min_effort_ = 40.0;  // This is a guess. Could not find data with quick search.
    params.max_effort_ = 100.0;

    return params;
}

namespace 
{
class Robotiq2FGripperActionServer : public rclcpp::Node
{
public:
  using GoalHandleRobotiq2fGripper = rclcpp_action::ServerGoalHandle<Fibonacci>;

  ACTION_TUTORIALS_CPP_PUBLIC
  explicit Robotiq2FGripperActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("robotiq_2f_gripper_action_server", options)
  {
    using namespace std::placeholders;

    std::string action_name_ = "gripper";
    GripperInput current_state_;
    GripperOutput goal_state_;

    this->action_server_ = rclcpp_action::create_server<Fibonacci>(
      this,
      action_name,
      std::bind(&Robotiq2FGripperActionServer::handle_goal, this, _1, _2),
      std::bind(&Robotiq2FGripperActionServer::handle_cancel, this, _1),
      std::bind(&Robotiq2FGripperActionServer::handle_accepted, this, _1));

    state_subscriber_ = this->create_subscription<GripperInput>(
      "Robotiq2FGripperRobotInput", 10, std::bind(&this::state_callback, this, _1));

    goal_publisher_ = this->create_publisher<GripperOutput>("Robotiq2FGripperRobotOutput", 10);
  }

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

  void state_callback(GripperInput & msg){

    current_state = *msg;

    if (current_state_.gSTA != 0x3)
    {
        // Check to see if the gripper is active or if it has been asked to be active
        if (current_reg_state_.gSTA == 0x0 && goal_state_.rACT != 0x1)
        {
            // If it hasn't been asked, active it
            issueActivation();
        }

        // Otherwise wait for the gripper to activate
        // TODO: If message delivery isn't guaranteed, then we may want to resend activate
        return;
    }

  }



  rclcpp_action::GoalResponse handle_goal( const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Fibonacci::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;

    if (current_reg_state_.gSTA != 0x3)
    {
        RCLCPP_WARN(this->get_logger(),"%s could not accept goal because the gripper is not yet active", action_name_.c_str());
        rclcpp_action::GoalResponse::REJECT;
    }
    else
    {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
  }



  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleRobotiq2fGripper> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&Robotiq2fGripperActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    //goal: position + speed + force
    //feedback position
    //result position


    /*
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto & sequence = feedback->partial_sequence;
    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<Fibonacci::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Update sequence
      sequence.push_back(sequence[i] + sequence[i - 1]);
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
    */
    rclcpp::Rate loop_rate(10);
    const auto goal = goal_handle->get_goal();
    params = c2_85_defaults();
    double dist_per_tick = (params.max_gap_ - params.min_gap_) / 255;
    double eff_per_tick = (params.max_effort_ - params.min_effort_) / 255;

    if (goal.command.position > params.max_gap_ || goal.command.position < params.min_gap_)
    {
        RCLCPP_WARN(this->get_logger(),"Goal gripper gap size is out of range(%f to %f): %f m", params.min_gap_, params.max_gap_,
                 goal.command.position);
        throw BadArgumentsError();
    }

    if (goal.command.max_effort < params.min_effort_ || goal.command.max_effort > params.max_effort_)
    {
        RCLCPP_WARN(this->get_logger(),"Goal gripper effort out of range (%f to %f N): %f N", params.min_effort_, params.max_effort_,
                 goal.command.max_effort);
        throw BadArgumentsError();
    }

    GripperOutput output;
    output.r_act = 0x1;  // active gripper
    output.r_gto = 0x1;  // go to position
    output.r_atr = 0x0;  // No emergency release
    output.r_sp = 128;   // Middle ground speed

    output.rPR = static_cast<uint8_t>((params.max_gap_ - goal.command.position) / dist_per_tick);
    output.rFR = static_cast<uint8_t>((goal.command.max_effort - params.min_effort_) / eff_per_tick);

    RCLCPP_INFO(this->get_logger(),"Setting goal position register to %hhu", result.rPR);

    goal_state_ = output;
    goal_publisher_->publish(goal_state_);

    boost::this_thread::sleep( boost::posix_time::seconds(3) );

    // Check if goal is done
    if (rclcpp::ok()) {
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }

  }

  void Robotiq2FGripperActionServer::issueActivation()
  {
    RCLCPP_INFO(this->get_logger(),"Activating gripper for gripper action server: %s", action_name_.c_str());
    GripperOutput out;
    out.r_acT = 0x1;
    // other params should be zero
    goal_state_ = out;
    goal_publisher_->publish(out);
  }
};  // class Robotiq_2f_Gripper_ActionServer

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionServer)