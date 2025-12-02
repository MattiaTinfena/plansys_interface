#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "algorithm"

using namespace std::chrono_literals;

class MoveAction : public plansys2::ActionExecutorClient
{
public:
  MoveAction()
  : plansys2::ActionExecutorClient("move", 250ms)
  {
    progress_ = 0.0;
    cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
  }

private:
  void do_work() override
  {
    if (progress_ < 1.0) {
      progress_ += 0.06;
      send_feedback(progress_, "Move running");
      
    auto args = get_arguments();
    if (args.size() < 3) {
        RCLCPP_ERROR(get_logger(), "Not enough arguments for move action");
        finish(false, 0.0, "Insufficient arguments");
        return;
    }
    auto wp_to_navigate = args[2];
    if (wp_to_navigate == "bathroom") {
      RCLCPP_INFO(get_logger(), "Start navigation to [%s]", wp_to_navigate.c_str());
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = 1.0;
      cmd.angular.z = 1.0; 
      cmd_vel_->publish(cmd);
    }
    else if (wp_to_navigate == "bedroom") {
      RCLCPP_INFO(get_logger(), "Start navigation to [%s]", wp_to_navigate.c_str());
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = 1.0;
      cmd.angular.z = -1.0; 
      cmd_vel_->publish(cmd);
    }}
    else {
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = 0.0;
      cmd_vel_->publish(cmd);

      finish(true, 1.0, "Move completed");
      progress_ = 0.0;
      std::cout << std::endl;
    }

    std::cout << "\r\e[K" << std::flush;
    std::cout << "Moving ... [" << std::min(100.0, progress_ * 100.0) << "%]  " << std::flush;
  }

  float progress_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "move"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::spin_some(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
