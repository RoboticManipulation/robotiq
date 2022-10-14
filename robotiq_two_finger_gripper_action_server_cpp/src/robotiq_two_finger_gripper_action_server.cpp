
#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <string>

// #include "action_tutorials_interfaces/action/fibonacci.hpp"
#include <robotiq_two_finger_gripper_actions/action/robotiq_two_finger_gripper.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
//using std::placeholders::_1;

#include <robotiq_two_finger_gripper_action_server_cpp/visibility_control.h>

#include <robotiq_2f_gripper_control/msg/robotiq2_f_gripper_robot_input.hpp>
#include <robotiq_2f_gripper_control/msg/robotiq2_f_gripper_robot_output.hpp>


typedef robotiq_2f_gripper_control::msg::Robotiq2FGripperRobotInput GripperInput;
typedef robotiq_2f_gripper_control::msg::Robotiq2FGripperRobotOutput GripperOutput;


namespace robotiq_two_finger_gripper_action_server_cpp
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
      robotiq_two_finger_gripper_action_server_cpp::Robotiq2FGripperParams params;
      params.min_gap_ = 0.0;
      params.max_gap_ = 0.085;
      params.min_effort_ = 40.0;  // This is a guess. Could not find data with quick search.
      params.max_effort_ = 100.0;

      return params;
  }

  class RobotiqTwoFingerGripperActionServer : public rclcpp::Node
  {
  public:
    using RobotiqTwoFingerGripper = robotiq_two_finger_gripper_actions::action::RobotiqTwoFingerGripper;
    using GoalHandleRobotiqTwoFingerGripper = rclcpp_action::ServerGoalHandle<RobotiqTwoFingerGripper>;

    GripperInput current_state_;
    std::string action_name_ ;
    bool first_call_;


    ROBOTIQ_TWO_FINGER_GRIPPER_ACTION_SERVER_PUBLIC
    explicit RobotiqTwoFingerGripperActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("robotiq_two_finger_gripper_action_server", options)
      
    {
      using namespace std::placeholders;

         /* These define the callback groups
       * They don't really do much on their own, but they have to exist in order to
       * assign callbacks to them. They're also what the executor looks for when trying to run multiple threads
       */
      callback_group_publisher_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      callback_group_subscriber_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

      // Each of these callback groups is basically a thread
      // Everything assigned to one of them gets bundled into the same thread
      auto sub2_opt = rclcpp::SubscriptionOptions();
      sub2_opt.callback_group = callback_group_subscriber_;
      auto sub3_opt = rclcpp::PublisherOptions();
      sub3_opt.callback_group = callback_group_publisher_;

      action_name_ = "robotiq_two_finger_gripper";

      first_call_ = true;

      //current_state_.g_sta = 0x3;
      //current_state_.g_po = 42;
      

      this->action_server_ = rclcpp_action::create_server<RobotiqTwoFingerGripper>(
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "robotiq_two_finger_gripper",
        std::bind(&RobotiqTwoFingerGripperActionServer::handle_goal, this, _1, _2),
        std::bind(&RobotiqTwoFingerGripperActionServer::handle_cancel, this, _1),
        std::bind(&RobotiqTwoFingerGripperActionServer::handle_accepted, this, _1));

      state_subscriber_ = this->create_subscription<GripperInput>(
        "Robotiq2FGripperRobotInput", 10, std::bind(&RobotiqTwoFingerGripperActionServer::state_callback, this, _1),sub2_opt);

      goal_publisher_ = this->create_publisher<GripperOutput>("Robotiq2FGripperRobotOutput", 10, sub3_opt);
      
    }

  private:
    rclcpp_action::Server<RobotiqTwoFingerGripper>::SharedPtr action_server_;
    rclcpp::Subscription<GripperInput>::SharedPtr state_subscriber_;
    rclcpp::Publisher<GripperOutput>::SharedPtr goal_publisher_;

    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_;
    rclcpp::CallbackGroup::SharedPtr callback_group_publisher_;

    
    ROBOTIQ_TWO_FINGER_GRIPPER_ACTION_SERVER_LOCAL
    void state_callback(const GripperInput::SharedPtr msg)
    {
      this->current_state_ = *msg;
      //RCLCPP_INFO(this->get_logger(), "updating status");

      if (first_call_){

        RCLCPP_INFO(this->get_logger(),"Reseting Gripper Status for: %s", action_name_.c_str());
              GripperOutput out;
              out.r_act = 0x0;
              goal_publisher_->publish(out);
              first_call_ = false;
              std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }

      if (this->current_state_.g_sta != 0x3)
      {
          // Check to see if the gripper is activating
          if (current_state_.g_sta == 0x0)
          {
              // If it hasn't been asked, active it
              RCLCPP_INFO(this->get_logger(),"Activating gripper for gripper action server: %s", action_name_.c_str());
              GripperOutput out;
              out.r_act = 0x1;
              // other params should be zero
              goal_publisher_->publish(out);
              std::this_thread::sleep_for(std::chrono::milliseconds(100));

          }
          // Otherwise wait for the gripper to activate
      }
    }
    
    
    ROBOTIQ_TWO_FINGER_GRIPPER_ACTION_SERVER_LOCAL
    rclcpp_action::GoalResponse handle_goal( const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const RobotiqTwoFingerGripper::Goal> goal)
    {
      RCLCPP_INFO(this->get_logger(), "Received goal request");
      (void)uuid;

      if (current_state_.g_sta != 0x3)
      {
          RCLCPP_WARN(this->get_logger(),"%s could not accept goal because the gripper is not yet active", action_name_.c_str());
          return rclcpp_action::GoalResponse::REJECT;
      }
      else if ((goal->goal_position > 0.086) || (goal->goal_position < 0.0))
      {
        RCLCPP_WARN(this->get_logger(),"%s could not accept goal because those values are too big/small", action_name_.c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }
      else
      {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      }
    }


    ROBOTIQ_TWO_FINGER_GRIPPER_ACTION_SERVER_LOCAL
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleRobotiqTwoFingerGripper> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    }


    ROBOTIQ_TWO_FINGER_GRIPPER_ACTION_SERVER_LOCAL
    void handle_accepted(const std::shared_ptr<GoalHandleRobotiqTwoFingerGripper> goal_handle)
    {
      using namespace std::placeholders;
      // this needs to return quickly to avoid blocking the executor, so spin up a new thread
      std::thread{std::bind(&RobotiqTwoFingerGripperActionServer::execute, this, _1), goal_handle}.detach();
    }


    ROBOTIQ_TWO_FINGER_GRIPPER_ACTION_SERVER_LOCAL
    void execute(const std::shared_ptr<GoalHandleRobotiqTwoFingerGripper> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Executing goal");


      //WARNING: increasing the looprate may result in the server spinning to quick for the subscriber,
      //         so some status variables will not be up to date and break the loop

      rclcpp::Rate loop_rate(5);
      const auto goal = goal_handle->get_goal();
      Robotiq2FGripperParams params = c2_85_defaults();
      //we only consider 230 steps instead of the full 255 range, because the gripper does not close further beyond 230
      double dist_per_tick = (params.max_gap_ - params.min_gap_) / 230;

      auto feedback = std::make_shared<RobotiqTwoFingerGripper::Feedback>();
      auto result = std::make_shared<RobotiqTwoFingerGripper::Result>();

      GripperOutput output;
      output.r_act = 0x1;  // active gripper
      output.r_gto = 0x1;  // go to position
      output.r_atr = 0x0;  // No emergency release
      output.r_sp = 100;   // Middle ground speed

      RCLCPP_INFO(this->get_logger(),"per tick: %lf, goal: %f", dist_per_tick, goal->goal_position);
      output.r_pr = static_cast<uint8_t>((params.max_gap_ - goal->goal_position) / dist_per_tick);

      RCLCPP_INFO(this->get_logger(),"Setting goal position register to %hhu", output.r_pr);

      if (goal->force < 25)
      {
        output.r_fr = static_cast<uint8_t>(25);
        RCLCPP_INFO(this->get_logger(),"Setting force to the minimum working value of 25");
      }
      else
      {
        output.r_fr = static_cast<uint8_t>(goal->force);
      }

      goal_publisher_->publish(output);

      //the server needs to sleep for a bit, otherwise the current_status_.g_obj will not yet be updated and therefore not start thw while loop

      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      while (current_state_.g_obj == 0x0)
      {
        if (goal_handle->is_canceling()) 
        {
            result->final_position = (params.max_gap_-(current_state_.g_po * dist_per_tick));
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }

        feedback->current_position = (params.max_gap_-(current_state_.g_po * dist_per_tick));
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Publish feedback");

        loop_rate.sleep();
      }

      // Info about meanings of status
      //# g_obj
      //output += "g_obj = " + str(status.g_obj) + ": "
      //if status.g_obj == 0:
      //    output += "Fingers are in motion (only meaningful if gGTO = 1)\n"
      //if status.g_obj == 1:
      //    output += "Fingers have stopped due to a contact while opening\n"
      //if status.g_obj == 2:
      //    output += "Fingers have stopped due to a contact while closing \n"
      //if status.g_obj == 3:
      //    output += "Fingers are at requested position\n"

      // Check if goal is done
      if (rclcpp::ok()) 
      {
        if ((current_state_.g_obj == 0x1)||(current_state_.g_obj == 0x2))
        {
          result->final_position = (params.max_gap_-(current_state_.g_po * dist_per_tick));
          goal_handle->abort(result);
          RCLCPP_INFO(this->get_logger(), "Goal aborted, fingers where stopped due to object contact");
        }
        else if (current_state_.g_obj == 0x3)
        {
          result->final_position = (params.max_gap_-(current_state_.g_po * dist_per_tick));
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "Reached Goal");
        }
        
      }

    }
    
  };  // class Robotiq_TwoFinger_Gripper_ActionServer

}  // namespace robotiq_action_server

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<robotiq_two_finger_gripper_action_server_cpp::RobotiqTwoFingerGripperActionServer>();

  rclcpp::executors::MultiThreadedExecutor executor;
                                                        
  executor.add_node(action_server);
  executor.spin();

  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}


//RCLCPP_COMPONENTS_REGISTER_NODE(robotiq_two_finger_gripper_action_server_cpp::RobotiqTwoFingerGripperActionServer)