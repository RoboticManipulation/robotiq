#include <chrono>
#include <functional>
#include <memory>
#include <string>
//#include <sstream>

#include "rclcpp/rclcpp.hpp"

#include "robotiq_two_finger_gripper_action_client.cpp"


class RobotiqTwoFingerGripperActionClientTest : public rclcpp::Node
{
  public:
    RobotiqTwoFingerGripperActionClientTest(std::shared_ptr<class robotiq_two_finger_gripper_action_server_cpp::RobotiqTwoFingerGripperActionClient> action_client): Node("robotiq_two_finger_gripper_action_client_test"), count_(0)
    {
      // timer_ = this->create_wall_timer(
      // 500ms, std::bind(&MinimalPublisher::timer_callback, this));
      action_client_ = action_client;
      RCLCPP_INFO(this->get_logger(), "Hello");
      test();
      
    }

  void test()
  {
    RCLCPP_INFO(this->get_logger(), "Hello2");
    // using namespace std::placeholders;
    action_client_->send_goal();

    
    
  }

private:
  // rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
  std::shared_ptr<class robotiq_two_finger_gripper_action_server_cpp::RobotiqTwoFingerGripperActionClient> action_client_;

};  // class RobotiqTwoFingerGripperActionClientTest


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options = rclcpp::NodeOptions();

  auto action_client = std::make_shared<robotiq_two_finger_gripper_action_server_cpp::RobotiqTwoFingerGripperActionClient>();

  auto action_client_test = std::make_shared<RobotiqTwoFingerGripperActionClientTest>(action_client);

  //action_client_test->test();

  //action_client->send_goal();

  

  //rclcpp::spin(action_client_test);

  rclcpp::shutdown();
  return 0;
}