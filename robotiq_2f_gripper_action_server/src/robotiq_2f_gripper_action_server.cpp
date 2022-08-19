#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <boost/thread/thread.hpp>

// #include "action_tutorials_interfaces/action/fibonacci.hpp"
#include <robotiq_2f_gripper_actions/action/robotiq2f_gripper.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

//#include <robotiq_2f_gripper_action_server/visibility_control.h>

#include <robotiq_2f_gripper_control/msg/robotiq2_f_gripper_robot_input.hpp>
#include <robotiq_2f_gripper_control/msg/robotiq2_f_gripper_robot_output.hpp>


typedef robotiq_2f_gripper_control::msg::Robotiq2FGripperRobotInput GripperInput;
typedef robotiq_2f_gripper_control::msg::Robotiq2FGripperRobotOutput GripperOutput;



namespace robotiq_2f_gripper_action_server
{

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

  class Robotiq2FGripperActionServer : public rclcpp::Node
  {
  public:
    using Robotiq2fGripper = robotiq_2f_gripper_actions::action::Robotiq2fGripper;
    using GoalHandleRobotiq2fGripper = rclcpp_action::ServerGoalHandle<Robotiq2fGripper>;

    //ROBOTIQ_2F_GRIPPER_ACTION_SERVER_PUBLIC
    explicit Robotiq2FGripperActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("robotiq_2f_gripper_action_server", options)
    {
      using namespace std::placeholders;

      std::string action_name_ = "gripper";
      GripperInput current_state_;
      GripperOutput goal_state_;

      this->action_server_ = rclcpp_action::create_server<Robotiq2fGripper>(
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        action_name_,
        std::bind(&Robotiq2FGripperActionServer::handle_goal, this, _1, _2),
        std::bind(&Robotiq2FGripperActionServer::handle_cancel, this, _1),
        std::bind(&Robotiq2FGripperActionServer::handle_accepted, this, _1));

      /*state_subscriber_ = this->create_subscription<GripperInput>(
        "Robotiq2FGripperRobotInput", 10, std::bind(&this::state_callback, this, _1));

      goal_publisher_ = this->create_publisher<GripperOutput>("Robotiq2FGripperRobotOutput", 10);
      */
    }

  private:
    rclcpp_action::Server<Robotiq2fGripper>::SharedPtr action_server_;
    //ROBOTIQ_2F_GRIPPER_ACTION_SERVER_LOCAL
    void state_callback(GripperInput & msg)
    {

      this->current_state_ = *msg;

      if (this->current_state_.gSTA != 0x3)
      {
          // Check to see if the gripper is active or if it has been asked to be active
          if (current_reg_state_.gSTA == 0x0 && goal_state_.rACT != 0x1)
          {
              // If it hasn't been asked, active it
              RCLCPP_INFO(this->get_logger(),"Activating gripper for gripper action server: %s", action_name_.c_str());
              GripperOutput out;
              out.r_acT = 0x1;
              // other params should be zero
              goal_state_ = out;
              goal_publisher_->publish(out);
          }

          // Otherwise wait for the gripper to activate
          // TODO: If message delivery isn't guaranteed, then we may want to resend activate
          return;
      }

    }
    //ROBOTIQ_2F_GRIPPER_ACTION_SERVER_LOCAL
    rclcpp_action::GoalResponse handle_goal( const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Robotiq2fGripper::Goal> goal)
    {
      RCLCPP_INFO(this->get_logger(), "Received goal request");
      (void)uuid;

      if (current_reg_state_.gSTA != 0x3)
      {
          RCLCPP_WARN(this->get_logger(),"%s could not accept goal because the gripper is not yet active", action_name_.c_str());
          return rclcpp_action::GoalResponse::REJECT;
      }
      else if ((goal->goal_position > 0.085) || (goal->goal_position < 0.0))
      {
        RCLCPP_WARN(this->get_logger(),"%s could not accept goal because those values are too big/small", action_name_.c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }
      else
      {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      }
    }

    //ROBOTIQ_2F_GRIPPER_ACTION_SERVER_LOCAL
    rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleRobotiq2fGripper> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    }
    //ROBOTIQ_2F_GRIPPER_ACTION_SERVER_LOCAL
    void handle_accepted(const std::shared_ptr<GoalHandleRobotiq2fGripper> goal_handle)
    {
      using namespace std::placeholders;
      // this needs to return quickly to avoid blocking the executor, so spin up a new thread
      std::thread{std::bind(&Robotiq2fGripperActionServer::execute, this, _1), goal_handle}.detach();
    }
    //ROBOTIQ_2F_GRIPPER_ACTION_SERVER_LOCAL
    void execute(const std::shared_ptr<GoalHandleRobotiq2fGripper> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Executing goal");

      rclcpp::Rate loop_rate(10);
      const auto goal = goal_handle->get_goal();
      params = c2_85_defaults();
      double dist_per_tick = (params.max_gap_ - params.min_gap_) / 255;
      double eff_per_tick = (params.max_effort_ - params.min_effort_) / 255;

      auto feedback = std::make_shared<Robotiq2fGripper::Feedback>();
      auto result = std::make_shared<Robotiq2fGripper::Result>();

      GripperOutput output;
      output.r_act = 0x1;  // active gripper
      output.r_gto = 0x1;  // go to position
      output.r_atr = 0x0;  // No emergency release
      output.r_sp = 128;   // Middle ground speed

      output.r_pr = static_cast<uint8_t>((params.max_gap_ - goal->goal_position) / dist_per_tick);

      if (goal_handle->is_canceling()) 
      {
          result->final_position = current_state_.g_po;
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
          return;
      }

      RCLCPP_INFO(this->get_logger(),"Setting goal position register to %hhu", result.r_pr);

      if (goal->force < 25)
      {
        output.r_fr = static_cast<uint8_t>(25);
        RCLCPP_INFO(this->get_logger(),"Setting force to the minimum working value of 25");
      }
      else
      {
        output.r_fr = static_cast<uint8_t>(goal->force);
      }

      goal_state_ = output;
      goal_publisher_->publish(goal_state_);

      feedback->current_position = current_state_.g_po;
      goal_handle->publish_feedback(feedback);
      CLCPP_INFO(this->get_logger(), "Publish feedback");

      //loop_rate.sleep();

      boost::this_thread::sleep( boost::posix_time::seconds(3) );

      // Check if goal is done
      if (rclcpp::ok()) {
        result->final_position = current_state_.g_po;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      }

    }
    
  };  // class Robotiq_2f_Gripper_ActionServer

}  // namespace robotiq_action_server

RCLCPP_COMPONENTS_REGISTER_NODE(robotiq_2f_gripper_action_server::Robotiq2fGripperActionServer)